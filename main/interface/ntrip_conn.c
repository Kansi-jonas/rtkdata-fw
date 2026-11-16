/* NTRIP upload connection policy, see ntrip_conn.h. Platform-neutral C99. */

#include "interface/ntrip_conn.h"

#include <string.h>

/* Escalation after the floor phase. The first five retries stay at the floor
 * so a short outage (caster restart, DHCP renew) recovers within seconds. */
static const uint32_t s_steps_ms[] = {
    5000, 10000, 15000, 30000, 45000, 60000, 90000, 120000, 180000, 240000,
    NTRIP_BACKOFF_CAP_MS,
};
#define FLOOR_RETRIES 5u
#define STEP_COUNT (sizeof(s_steps_ms) / sizeof(s_steps_ms[0]))

uint32_t ntrip_backoff_ms(uint32_t attempt, uint32_t rnd) {
    if (attempt == 0) return 0;
    uint32_t base;
    if (attempt <= FLOOR_RETRIES) {
        base = NTRIP_BACKOFF_FLOOR_MS;
    } else {
        uint32_t i = attempt - FLOOR_RETRIES - 1;
        base = i < STEP_COUNT ? s_steps_ms[i] : NTRIP_BACKOFF_CAP_MS;
    }
    /* jitter in [0, base/4]; the cap stays a cap */
    uint32_t jitter = rnd % (base / 4u + 1u);
    uint32_t d = base + jitter;
    return d > NTRIP_BACKOFF_CAP_MS ? NTRIP_BACKOFF_CAP_MS : d;
}

uint32_t ntrip_backoff_next_attempt(uint32_t attempt, bool healthy_session) {
    if (healthy_session) return 1;
    return attempt == UINT32_MAX ? attempt : attempt + 1;
}

/* length of the status line (up to, not including, the first CR or LF) if a
 * LF has arrived; -1 if not yet complete */
static long status_line_len(const char *buf, size_t len) {
    const char *lf = memchr(buf, '\n', len);
    if (!lf) return -1;
    size_t n = (size_t)(lf - buf);
    if (n > 0 && buf[n - 1] == '\r') n--;
    return (long)n;
}

static bool starts(const char *s, size_t n, const char *prefix) {
    size_t p = strlen(prefix);
    return n >= p && memcmp(s, prefix, p) == 0;
}

/* "<proto> 200" followed by end of line or a space */
static bool code_200_after(const char *s, size_t n, const char *proto) {
    size_t p = strlen(proto);
    if (!starts(s, n, proto) || n < p + 4) return false;
    if (s[p] != ' ' || memcmp(s + p + 1, "200", 3) != 0) return false;
    return n == p + 4 || s[p + 4] == ' ';
}

ntrip_reply_t ntrip_reply_parse(const char *buf, size_t len, size_t cap) {
    if (!buf) return NTRIP_REPLY_REJECTED;
    long n = status_line_len(buf, len);
    if (n < 0) return len >= cap ? NTRIP_REPLY_OVERFLOW : NTRIP_REPLY_INCOMPLETE;
    size_t ln = (size_t)n;
    if (code_200_after(buf, ln, "ICY") ||
        code_200_after(buf, ln, "HTTP/1.0") ||
        code_200_after(buf, ln, "HTTP/1.1")) {
        return NTRIP_REPLY_OK;
    }
    /* bare "OK" (some v1 casters), exactly that line */
    if (ln == 2 && memcmp(buf, "OK", 2) == 0) return NTRIP_REPLY_OK;
    return NTRIP_REPLY_REJECTED;
}

void ntrip_reply_status_line(const char *buf, size_t len, char *out, size_t out_sz) {
    if (!out || out_sz == 0) return;
    out[0] = '\0';
    if (!buf) return;
    size_t n = 0;
    while (n < len && buf[n] != '\r' && buf[n] != '\n') n++;
    if (n >= out_sz) n = out_sz - 1;
    for (size_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)buf[i];
        out[i] = (c >= 0x20 && c < 0x7f) ? (char)c : '?';
    }
    out[n] = '\0';
}
