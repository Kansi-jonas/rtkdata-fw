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

        int fail = find_at(buf, end, body, "PARSING FAIL");
        if (fail >= 0) {
            /* failure: $command,response: PARSING FAILD NO MATCHING FUNC <cmd>*
             * The command sits AFTER the verdict, at the end of the record. */
            int func = find_at(buf, end, (size_t)fail, "NO MATCHING FUNC ");
            if (func >= 0) {
                size_t cstart = (size_t)func + strlen("NO MATCHING FUNC ");
                if (cstart <= end && span_equals(buf + cstart, end - cstart, cmd)) {
                    verdict = GNSS_ACK_REJECTED;
                }
            }
        } else {
            /* success: $command,<cmd>,response: OK*
             * The command sits BETWEEN the prefix and ",response:". */
            int resp = find_at(buf, end, body, ",response:");
            if (resp >= 0 && span_equals(buf + body, (size_t)resp - body, cmd)) {
                if (find_at(buf, end, (size_t)resp, "OK") >= 0) {
                    verdict = GNSS_ACK_OK;
                }
            }
        }

        pos = end;   /* keep scanning: the LAST verdict for cmd wins */
    }

    return verdict;
}
