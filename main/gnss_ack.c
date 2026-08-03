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
        int star = find_at(buf, n, body, "*");
        int next = find_at(buf, n, body, PREFIX);
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
                if (find_at(buf, end, (size_t)resp, "PARSING FAIL") >= 0) {
                    verdict = GNSS_ACK_REJECTED;
                } else if (find_at(buf, end, (size_t)resp, "OK") >= 0) {
                    verdict = GNSS_ACK_OK;
                }
            }
        }

        pos = end;   /* keep scanning: the LAST verdict for cmd wins */
    }

    return verdict;
}
