/* NTRIP upload connection policy: reconnect backoff and the caster's reply to
 * the SOURCE request.
 *
 * Platform-neutral C99 like ntrip_tx_core.h: compiled by ESP-IDF for the
 * device and by a host compiler for tests/host/test_ntrip_conn.c. No FreeRTOS,
 * no lwIP. Time and randomness are passed in.
 *
 * Why this exists (SOTA review 2026-11-06, WP2):
 *  - The backoff reset on the first accepted byte, so a caster that accepted
 *    SOURCE and dropped the link a moment later was redialled at once, in a
 *    loop and in lockstep across the fleet. It now resets only after a
 *    session that stayed healthy for NTRIP_BACKOFF_HEALTHY_MS, never drops
 *    below NTRIP_BACKOFF_FLOOR_MS after the first attempt, and is jittered.
 *  - The backoff grew to one hour. A station whose caster was down for a
 *    while then stayed dark for up to an hour after the caster came back.
 *    The cap is now NTRIP_BACKOFF_CAP_MS.
 *  - The reply was read with one read() and compared literally. A status
 *    line split across two segments, or "HTTP/1.1 200" with another reason
 *    phrase, was rejected as a failed connect.
 */

#ifndef NTRIP_CONN_H
#define NTRIP_CONN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NTRIP_BACKOFF_FLOOR_MS    2000u     /* every retry after the first */
#define NTRIP_BACKOFF_CAP_MS      300000u   /* 5 min */
#define NTRIP_BACKOFF_HEALTHY_MS  60000u    /* session length that resets it */

/* Delay before connection attempt number `attempt` (0 = the first attempt
 * after boot, which goes out at once). `rnd` is any 32-bit random value; up
 * to a quarter of the delay is added as jitter so units that lost the same
 * caster at the same moment do not redial in lockstep. */
uint32_t ntrip_backoff_ms(uint32_t attempt, uint32_t rnd);

/* The attempt counter after a session ended. A session that delivered data
 * for at least NTRIP_BACKOFF_HEALTHY_MS starts over at attempt 1 (floor
 * delay); anything shorter keeps escalating. */
uint32_t ntrip_backoff_next_attempt(uint32_t attempt, bool healthy_session);

typedef enum {
    NTRIP_REPLY_INCOMPLETE = 0,   /* no complete status line yet: read more */
    NTRIP_REPLY_OK,               /* caster accepted the upload */
    NTRIP_REPLY_REJECTED,         /* any other status line */
    NTRIP_REPLY_OVERFLOW,         /* cap bytes without a line end */
} ntrip_reply_t;

/* Judge the reply bytes received so far (buf need not be NUL-terminated).
 * Accepted status lines: "ICY 200 ...", "HTTP/1.0 200 ..." or
 * "HTTP/1.1 200 ..." with any or no reason phrase, and the bare "OK" some
 * NTRIP v1 casters send. */
ntrip_reply_t ntrip_reply_parse(const char *buf, size_t len, size_t cap);

/* Copy the status line (without CR/LF) into out for logging; always
 * NUL-terminates. */
void ntrip_reply_status_line(const char *buf, size_t len, char *out, size_t out_sz);

#endif /* NTRIP_CONN_H */
