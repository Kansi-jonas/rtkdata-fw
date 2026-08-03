/* See gnss_ack.h for the two response grammars this has to handle. */

#include <stdbool.h>
#include <string.h>

#include "gnss_ack.h"

#define PREFIX     "$command,"
#define PREFIX_LEN 9

/* Bounded substring search inside [from, n). Returns the START offset or -1. */
static int find_at(const char *hay, size_t n, size_t from, const char *needle) {
    size_t nlen = strlen(needle);
    if (nlen == 0 || n < nlen) return -1;
    for (size_t i = from; i + nlen <= n; i++) {
        if (hay[i] == needle[0] && memcmp(hay + i, needle, nlen) == 0) return (int)i;
    }
    return -1;
}

/* True when [s, s+len) equals cmd exactly. */
static bool span_equals(const char *s, size_t len, const char *cmd) {
    size_t clen = strlen(cmd);
    return len == clen && memcmp(s, cmd, clen) == 0;
}

static bool is_hex(char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

static unsigned hex_val(char c) {
    if (c >= '0' && c <= '9') return (unsigned)(c - '0');
    if (c >= 'a' && c <= 'f') return (unsigned)(c - 'a' + 10);
    return (unsigned)(c - 'A' + 10);
}

/* A record is complete only with "*" plus both checksum digits, and the
 * checksum must be CORRECT.
 *
 * The algorithm is XOR over the record INCLUDING the leading '$', which is
 * where an earlier analysis went wrong: our own $PESP sentences use the
 * standard NMEA convention that EXCLUDES '$', so a single test with the wrong
 * convention "disproved" XOR and the check was dropped as unknowable.
 * Recomputed on the real captures:
 *   $command,VERSIONA,response: OK                       -> incl. $ = 45  (capture *45)
 *   $command,rtcm1230,...NO MATCHING FUNC  RTCM1230      -> incl. $ = 11  (capture *11)
 *   $PESP,RTK,GNSS,CONFIG,13,14                          -> excl. $ = 73  (our own *73)
 * Both device captures match with the '$' included. */
static bool record_valid_at(const char *buf, size_t n, size_t rec, size_t star) {
    if (star + 2 >= n) return false;
    if (!is_hex(buf[star + 1]) || !is_hex(buf[star + 2])) return false;

    unsigned want = (hex_val(buf[star + 1]) << 4) | hex_val(buf[star + 2]);
    unsigned got = 0;
    for (size_t i = rec; i < star; i++) got ^= (unsigned char)buf[i];

    return got == want;
}

gnss_ack_verdict_t gnss_ack_scan(const char *buf, size_t n, const char *cmd) {
    gnss_ack_verdict_t verdict = GNSS_ACK_NONE;

    if (buf == NULL || cmd == NULL || n == 0) return GNSS_ACK_NONE;

    size_t pos = 0;
    for (;;) {
        int rec = find_at(buf, n, pos, PREFIX);
        if (rec < 0) break;

        size_t body = (size_t)rec + PREFIX_LEN;

        /* A record ends at its checksum delimiter. Without one the record is
         * still arriving and must not be judged: a truncated
         * "...PARSING FAIL" prefix would otherwise read as a verdict. Stop at
         * the next record start too, so a lost delimiter cannot swallow the
         * following record. */
        int next = find_at(buf, n, body, PREFIX);
        int star = -1;
        for (int s = find_at(buf, n, body, "*"); s >= 0;
             s = find_at(buf, n, (size_t)s + 1, "*")) {
            if (next >= 0 && s > next) break;          /* belongs to a later record */
            if (record_valid_at(buf, n, (size_t)rec, (size_t)s)) { star = s; break; }
        }
        if (star < 0 || (next >= 0 && next < star)) {
            pos = body;
            continue;
        }
        size_t end = (size_t)star;

        /* The documented failure form has no echo, so "response:" follows the
         * prefix directly and there is no comma to anchor on. */
        bool no_echo = (end - body) >= 9 && memcmp(buf + body, "response:", 9) == 0;
        int  resp    = no_echo ? (int)body : find_at(buf, end, body, ",response:");

        if (no_echo) {
            int func = find_at(buf, end, body, "NO MATCHING FUNC");
            if (func >= 0) {
                size_t cstart = (size_t)func + strlen("NO MATCHING FUNC");
                while (cstart < end && buf[cstart] == ' ') cstart++;
                if (span_equals(buf + cstart, end - cstart, cmd)) {
                    verdict = GNSS_ACK_REJECTED;
                }
            }
        } else if (resp >= 0) {
            /* Measured on the device (UM980 R4.10Build11833, 2026-08-03): the
             * command is echoed between "$command," and ",response:" in BOTH
             * outcomes, and only the verdict text after it differs:
             *   $command,rtcm1077,com1,1,response: OK*7a
             *   $command,rtcm1230,com1,10,response: PARSING FAILD NO MATCHING FUNC  RTCM1230*11
             * The repo's research note claims the failure form omits the echo
             * and carries the command at the end instead; the hardware does
             * not do that. Trusting the note cost one release cycle. */
            if (span_equals(buf + body, (size_t)resp - body, cmd)) {
                /* Verdict text = everything after ",response:", trimmed.
                 * Substring matching accepted "NOT OK" as success
                 * (review 2026-08-03), so success requires the verdict to BE
                 * "OK", not to contain it. Anything we do not recognise stays
                 * NONE: an unknown reply must time out loudly, not be guessed
                 * into an approval. */
                size_t v = (size_t)resp + strlen(",response:");
                while (v < end && buf[v] == ' ') v++;
                size_t vend = end;
                while (vend > v && (buf[vend - 1] == ' ' || buf[vend - 1] == '\r')) vend--;

                if (v + 12 <= vend && memcmp(buf + v, "PARSING FAIL", 12) == 0) {
                    verdict = GNSS_ACK_REJECTED;
                } else if (span_equals(buf + v, vend - v, "OK")) {
                    verdict = GNSS_ACK_OK;
                }
            }
        }

        pos = end;   /* keep scanning: the LAST verdict for cmd wins */
    }

    return verdict;
}
