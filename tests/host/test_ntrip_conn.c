/* Host test for the NTRIP upload connection policy (main/interface/ntrip_conn.c).
 * gcc -Imain/include -Imain/include/interface tests/host/test_ntrip_conn.c main/interface/ntrip_conn.c
 */
#include "interface/ntrip_conn.h"

#include <stdio.h>
#include <string.h>

static int checks, failures;
#define CHECK(c) do { checks++; if (!(c)) { failures++; \
    printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static ntrip_reply_t parse(const char *s) { return ntrip_reply_parse(s, strlen(s), 511); }

/* Replays the session loop's bookkeeping: a session that dies right after
 * the handshake (or after one accepted send) must not reset the backoff. */
static uint32_t old_attempt_after_brief_session(uint32_t attempt) {
    (void)attempt;
    return 0;   /* old code: retry_reset() on the first accepted byte */
}

int main(void) {
    /* ---- backoff --------------------------------------------------- */
    CHECK(ntrip_backoff_ms(0, 12345) == 0);              /* first attempt after boot: at once */
    for (uint32_t a = 1; a <= 5; a++) {
        CHECK(ntrip_backoff_ms(a, 0) == NTRIP_BACKOFF_FLOOR_MS);
        uint32_t d = ntrip_backoff_ms(a, 0xFFFFFFFFu);
        CHECK(d >= NTRIP_BACKOFF_FLOOR_MS && d <= NTRIP_BACKOFF_FLOOR_MS * 5 / 4);
    }
    /* monotonic without jitter, capped, never zero after the first attempt */
    uint32_t prev = 0;
    for (uint32_t a = 1; a < 200; a++) {
        uint32_t d = ntrip_backoff_ms(a, 0);
        CHECK(d >= prev);
        CHECK(d >= NTRIP_BACKOFF_FLOOR_MS);
        CHECK(d <= NTRIP_BACKOFF_CAP_MS);
        prev = d;
    }
    CHECK(ntrip_backoff_ms(1000, 0) == NTRIP_BACKOFF_CAP_MS);
    CHECK(ntrip_backoff_ms(UINT32_MAX, 0xFFFFFFFFu) == NTRIP_BACKOFF_CAP_MS);
    /* the old table reached one hour: the cap is 5 min */
    CHECK(NTRIP_BACKOFF_CAP_MS == 300000u);
    /* reaches the cap within a bounded number of failures (~20 min total) */
    uint64_t total = 0; uint32_t to_cap = 0;
    for (uint32_t a = 1; ntrip_backoff_ms(a, 0) < NTRIP_BACKOFF_CAP_MS; a++) {
        total += ntrip_backoff_ms(a, 0); to_cap = a;
    }
    CHECK(to_cap < 20 && total < 20ull * 60 * 1000);
    /* jitter spreads units: 1000 random draws for one step are not all equal */
    {
        uint32_t lo = UINT32_MAX, hi = 0, x = 2463534242u;
        for (int i = 0; i < 1000; i++) {
            x ^= x << 13; x ^= x >> 17; x ^= x << 5;
            uint32_t d = ntrip_backoff_ms(10, x);
            if (d < lo) lo = d;
            if (d > hi) hi = d;
        }
        CHECK(hi > lo);
        CHECK(lo >= ntrip_backoff_ms(10, 0) && hi <= ntrip_backoff_ms(10, 0) * 5 / 4);
    }

    /* ---- backoff state across sessions -------------------------------- */
    /* N1: brief sessions keep escalating (old code reset to instant redial) */
    uint32_t att = 0;
    for (int i = 0; i < 12; i++) att = ntrip_backoff_next_attempt(att, false);
    CHECK(att == 12);
    CHECK(ntrip_backoff_ms(att, 0) > NTRIP_BACKOFF_FLOOR_MS);
    CHECK(ntrip_backoff_ms(old_attempt_after_brief_session(att), 0) == 0);  /* the storm */
    /* a healthy session resets to the floor, never to an instant redial */
    att = ntrip_backoff_next_attempt(att, true);
    CHECK(att == 1 && ntrip_backoff_ms(att, 0) == NTRIP_BACKOFF_FLOOR_MS);
    CHECK(ntrip_backoff_next_attempt(UINT32_MAX, false) == UINT32_MAX);

    /* ---- caster reply ---------------------------------------------- */
    CHECK(parse("ICY 200 OK\r\n") == NTRIP_REPLY_OK);
    CHECK(parse("ICY 200 OK\r\n\r\n") == NTRIP_REPLY_OK);
    CHECK(parse("ICY 200\r\n") == NTRIP_REPLY_OK);
    CHECK(parse("HTTP/1.1 200 OK\r\nServer: x\r\n\r\n") == NTRIP_REPLY_OK);
    CHECK(parse("HTTP/1.1 200 Connected\r\n") == NTRIP_REPLY_OK);   /* RFC 9112: any reason */
    CHECK(parse("HTTP/1.1 200\r\n") == NTRIP_REPLY_OK);
    CHECK(parse("HTTP/1.0 200 OK\r\n") == NTRIP_REPLY_OK);
    CHECK(parse("OK\r\n") == NTRIP_REPLY_OK);
    CHECK(parse("OK\n") == NTRIP_REPLY_OK);                         /* bare LF */
    /* rejections */
    CHECK(parse("ERROR - Bad Password\r\n") == NTRIP_REPLY_REJECTED);
    CHECK(parse("ERROR - Mount Point Taken or Invalid\r\n") == NTRIP_REPLY_REJECTED);
    CHECK(parse("HTTP/1.1 401 Unauthorized\r\n") == NTRIP_REPLY_REJECTED);
    CHECK(parse("HTTP/1.1 409 Conflict\r\n") == NTRIP_REPLY_REJECTED);
    CHECK(parse("HTTP/1.1 2000 OK\r\n") == NTRIP_REPLY_REJECTED);
    CHECK(parse("HTTP/1.1 20\r\n") == NTRIP_REPLY_REJECTED);
    CHECK(parse("ICY 2001\r\n") == NTRIP_REPLY_REJECTED);
    CHECK(parse("OKAY\r\n") == NTRIP_REPLY_REJECTED);
    CHECK(parse("SOURCETABLE 200 OK\r\n") == NTRIP_REPLY_REJECTED);  /* not an upload accept */
    CHECK(parse("\r\n") == NTRIP_REPLY_REJECTED);
    CHECK(parse(" ICY 200 OK\r\n") == NTRIP_REPLY_REJECTED);
    /* N5: split status line is INCOMPLETE until the line end arrives */
    {
        const char *full = "ICY 200 OK\r\n";
        for (size_t cut = 0; cut < strlen(full); cut++) {
            ntrip_reply_t r = ntrip_reply_parse(full, cut, 511);
            CHECK(r == NTRIP_REPLY_INCOMPLETE);
        }
        CHECK(ntrip_reply_parse(full, strlen(full), 511) == NTRIP_REPLY_OK);
        /* every split point of the HTTP form too */
        const char *http = "HTTP/1.1 200 OK\r\n";
        for (size_t cut = 0; cut < strlen(http); cut++)
            CHECK(ntrip_reply_parse(http, cut, 511) == NTRIP_REPLY_INCOMPLETE);
    }
    /* no line end within the cap: overflow, never a read past the buffer */
    {
        char junk[64];
        memset(junk, 'A', sizeof(junk));
        CHECK(ntrip_reply_parse(junk, sizeof(junk), sizeof(junk)) == NTRIP_REPLY_OVERFLOW);
        CHECK(ntrip_reply_parse(junk, 10, sizeof(junk)) == NTRIP_REPLY_INCOMPLETE);
        CHECK(ntrip_reply_parse(NULL, 0, 10) == NTRIP_REPLY_REJECTED);
    }
    /* status line for the log: no CR/LF, control bytes masked, bounded */
    {
        char out[16];
        const char *r = "ERROR - Bad\x01Password\r\nX";
        ntrip_reply_status_line(r, strlen(r), out, sizeof(out));
        CHECK(strcmp(out, "ERROR - Bad?Pas") == 0);
        ntrip_reply_status_line("OK\r\n", 4, out, sizeof(out));
        CHECK(strcmp(out, "OK") == 0);
        ntrip_reply_status_line(NULL, 0, out, sizeof(out));
        CHECK(out[0] == '\0');
        ntrip_reply_status_line("abc", 3, out, 1);
        CHECK(out[0] == '\0');
    }

    printf("%d ntrip connection policy checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
