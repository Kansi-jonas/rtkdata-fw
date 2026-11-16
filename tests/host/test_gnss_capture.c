/* Host tests for the UM980 response capture (main/gnss_capture.c).
 *
 * The defect (bench, 2026-11-06, K_EVK sample on 2.1.2): with satellites
 * tracked and MSM7 enabled, every ACK-gated command failed its first attempt
 * and the VERSIONA query after a reconfigure got no answer. The old capture
 * cleared the whole 1 KB buffer whenever a chunk did not fit, so an RTCM
 * burst wiped the response before the 20 ms poll read it. old_feed() below is
 * a copy of that code and the first test shows it losing the ACK.
 *
 * gcc -Imain/include tests/host/test_gnss_capture.c main/gnss_capture.c main/gnss_ack.c */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "gnss_capture.h"
#include "gnss_ack.h"

static int g_checks = 0, g_fails = 0;
#define CHECK(cond) do { g_checks++; if (!(cond)) { g_fails++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

/* Real UM980 replies (R4.10Build11833 bench captures; the checksum is XOR
 * including the '$', see gnss_ack.c). */
static const char *ACK_1077 = "$command,rtcm1077,com1,1,response: OK*36\r\n";
static const char *VERSIONA =
    "#VERSIONA,97,GPS,FINE,2443,481766000,0,0,18,630;\"UM980\",\"R4.10Build11833\","
    "\"HRPT00-S10C-P\",\"2310415000001-MD22A4252208815\",\"ff3b046ce93ffd9c\",\"2023/11/24\"*f581e33a\r\n";

/* ---- the old capture, for the regression proof ------------------------- */
#define OLD_SZ 1024
static char old_buf[OLD_SZ];
static size_t old_len;
static void old_feed(const uint8_t *data, size_t len) {
    if (old_len + len >= OLD_SZ) old_len = 0;   // keep the newest
    for (size_t i = 0; i < len && old_len < OLD_SZ - 1; i++) old_buf[old_len++] = (char)data[i];
    old_buf[old_len] = '\0';
}

/* ---- RTCM-like binary: D3 frames whose payload is full of '$', '#' and LF */
static unsigned g_seed = 12345;
static uint8_t rnd(void) { g_seed = g_seed * 1103515245u + 12345u; return (uint8_t)(g_seed >> 16); }
static size_t rtcm_frame(uint8_t *out, size_t payload) {
    size_t k = 0;
    out[k++] = 0xD3;
    out[k++] = (uint8_t)((payload >> 8) & 0x03);
    out[k++] = (uint8_t)(payload & 0xFF);
    for (size_t i = 0; i < payload; i++) {
        uint8_t b = rnd();
        if (i % 37 == 5) b = '$';
        if (i % 41 == 9) b = '#';
        if (i % 53 == 13) b = '\n';
        out[k++] = b;
    }
    for (int i = 0; i < 3; i++) out[k++] = rnd();
    return k;
}

static void feed_str(gnss_cap_t *c, const char *s) { gnss_cap_feed(c, (const uint8_t *)s, strlen(s)); }

/* Feed in UART-sized chunks, like uart_task does. */
static void feed_chunked(gnss_cap_t *c, const uint8_t *d, size_t n, size_t chunk) {
    for (size_t off = 0; off < n; off += chunk) {
        size_t m = n - off < chunk ? n - off : chunk;
        gnss_cap_feed(c, d + off, m);
        old_feed(d + off, m);
    }
}

static gnss_cap_t cap;
static uint8_t burst[16384];

int main(void) {
    /* 1. Regression: ACK, then 4 KB of MSM7 in 120-byte chunks. */
    gnss_cap_reset(&cap); old_len = 0;
    feed_chunked(&cap, (const uint8_t *)ACK_1077, strlen(ACK_1077), 120);
    size_t n = 0;
    while (n < 4000) n += rtcm_frame(burst + n, 200);
    feed_chunked(&cap, burst, n, 120);
    CHECK(gnss_ack_scan(old_buf, old_len, "rtcm1077,com1,1") == GNSS_ACK_NONE);  // the bug
    CHECK(gnss_ack_scan(cap.buf, cap.len, "rtcm1077,com1,1") == GNSS_ACK_OK);    // the fix

    /* 2. Only ASCII lines are kept: no binary byte reaches the buffer. */
    int binary = 0;
    for (size_t i = 0; i < cap.len; i++) {
        unsigned char ch = (unsigned char)cap.buf[i];
        if (!(ch == '\r' || ch == '\n' || (ch >= 0x20 && ch <= 0x7E))) binary++;
    }
    CHECK(binary == 0);

    /* 3. ACK arriving in the middle of a burst, split across chunks. */
    gnss_cap_reset(&cap);
    n = 0;
    for (int f = 0; f < 6; f++) n += rtcm_frame(burst + n, 300);
    memcpy(burst + n, ACK_1077, strlen(ACK_1077)); n += strlen(ACK_1077);
    for (int f = 0; f < 6; f++) n += rtcm_frame(burst + n, 300);
    feed_chunked(&cap, burst, n, 64);
    CHECK(gnss_ack_scan(cap.buf, cap.len, "rtcm1077,com1,1") == GNSS_ACK_OK);

    /* 4. VERSIONA survives a burst intact (CRC and line end included). */
    gnss_cap_reset(&cap);
    n = 0;
    memcpy(burst, VERSIONA, strlen(VERSIONA)); n = strlen(VERSIONA);
    for (int f = 0; f < 20; f++) n += rtcm_frame(burst + n, 400);
    feed_chunked(&cap, burst, n, 120);
    CHECK(strstr(cap.buf, VERSIONA) != NULL);

    /* 5. A line split byte by byte still commits. */
    gnss_cap_reset(&cap);
    for (size_t i = 0; i < strlen(ACK_1077); i++) gnss_cap_feed(&cap, (const uint8_t *)ACK_1077 + i, 1);
    CHECK(strcmp(cap.buf, ACK_1077) == 0);

    /* 6. A partial line is not visible until its line end arrives. */
    gnss_cap_reset(&cap);
    gnss_cap_feed(&cap, (const uint8_t *)ACK_1077, 20);
    CHECK(cap.len == 0);
    gnss_cap_feed(&cap, (const uint8_t *)ACK_1077 + 20, strlen(ACK_1077) - 20);
    CHECK(strcmp(cap.buf, ACK_1077) == 0);

    /* 7. Overflow drops the oldest WHOLE lines; the newest is always kept and
     *    the buffer starts at a line start. */
    gnss_cap_reset(&cap);
    char line[64];
    for (int i = 0; i < 200; i++) {
        snprintf(line, sizeof(line), "#BASEPOSA,%03d,PADDING-PADDING-PADDING*00\r\n", i);
        feed_str(&cap, line);
    }
    CHECK(cap.len < GNSS_CAP_SZ);
    CHECK(cap.buf[0] == '#');
    CHECK(strstr(cap.buf, "#BASEPOSA,199,") != NULL);
    CHECK(strstr(cap.buf, "#BASEPOSA,000,") == NULL);
    CHECK(cap.lines_dropped > 0);
    CHECK(cap.buf[cap.len] == '\0');

    /* 8. A line longer than the limit is dropped, the next one is kept. */
    gnss_cap_reset(&cap);
    char longl[GNSS_CAP_LINE_MAX + 50];
    longl[0] = '#';
    memset(longl + 1, 'A', sizeof(longl) - 4);
    longl[sizeof(longl) - 3] = '\r'; longl[sizeof(longl) - 2] = '\n'; longl[sizeof(longl) - 1] = '\0';
    feed_str(&cap, longl);
    CHECK(cap.len == 0);
    feed_str(&cap, ACK_1077);
    CHECK(strcmp(cap.buf, ACK_1077) == 0);

    /* 9. A binary byte inside a line voids it; a '$' resyncs a real record. */
    gnss_cap_reset(&cap);
    const uint8_t broken[] = { '$', 'c', 'o', 'm', 0xD3, 0x00, 0x10, '\n' };
    gnss_cap_feed(&cap, broken, sizeof(broken));
    CHECK(cap.len == 0);
    const char *junk_then_ack = "#garbage$command,rtcm1077,com1,1,response: OK*36\r\n";
    feed_str(&cap, junk_then_ack);
    CHECK(strcmp(cap.buf, ACK_1077) == 0);

    /* 10. A reset in the middle of a line forgets it. */
    gnss_cap_reset(&cap);
    gnss_cap_feed(&cap, (const uint8_t *)ACK_1077, 15);
    gnss_cap_reset(&cap);
    gnss_cap_feed(&cap, (const uint8_t *)ACK_1077 + 15, strlen(ACK_1077) - 15);
    CHECK(cap.len == 0);

    /* 11. Fuzz: random binary with embedded real ACKs; every ACK fed is found
     *     right after it was fed, as the 20 ms poll would see it. */
    int found = 0, fed = 0;
    gnss_cap_reset(&cap);
    for (int round = 0; round < 500; round++) {
        n = 0;
        int frames = 1 + rnd() % 8;
        for (int f = 0; f < frames; f++) n += rtcm_frame(burst + n, 50 + rnd() % 900);
        feed_chunked(&cap, burst, n, 1 + rnd() % 256);
        feed_chunked(&cap, (const uint8_t *)ACK_1077, strlen(ACK_1077), 1 + rnd() % 64);
        fed++;
        if (gnss_ack_scan(cap.buf, cap.len, "rtcm1077,com1,1") == GNSS_ACK_OK) found++;
        gnss_cap_reset(&cap);
    }
    CHECK(found == fed);

    printf("%s: %d checks, %d failed\n", g_fails ? "FAILED" : "all passed", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
