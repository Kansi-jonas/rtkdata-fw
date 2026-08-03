/* UM980 command-response parsing. Platform-neutral C99 so the host tests can
 * compile the production code directly (tests/host/test_gnss_ack.c).
 *
 * The receiver answers with TWO different grammars, and they put the command
 * text on OPPOSITE sides of the verdict (docs/UM980-config-research.md):
 *
 *   success:  $command,<command>,response: OK*<crc>
 *   failure:  $command,response: PARSING FAILD NO MATCHING FUNC <command>*<crc>
 *
 * ("FAILD" is Unicore's typo; match on "PARSING FAIL".) A matcher that looks
 * for the verdict AFTER the command echo therefore misses every real reject,
 * and a global search for a bare "response: OK" lets a late reply to command A
 * confirm command B. Both were shipped in turn (review 2026-08-03), which is
 * why this now lives in one tested function.
 *
 * The buffer may contain binary RTCM interleaved with the ASCII lines, so
 * everything here is length-bounded; no NUL-terminated string functions. */

#ifndef RTKDATA_GNSS_ACK_H
#define RTKDATA_GNSS_ACK_H

#include <stddef.h>

typedef enum {
    GNSS_ACK_NONE = 0,   /* no verdict for this command in the buffer yet */
    GNSS_ACK_OK,         /* the receiver accepted THIS command */
    GNSS_ACK_REJECTED,   /* the receiver rejected THIS command */
} gnss_ack_verdict_t;

/* Scan `buf` (length `n`, may contain binary) for a verdict addressed to
 * `cmd` (the command text without CRLF, e.g. "rtcm1230,com1,10").
 * Only complete `$command,...*` records are considered, so a half-received
 * line never produces a verdict. The LAST matching record wins. */
gnss_ack_verdict_t gnss_ack_scan(const char *buf, size_t n, const char *cmd);

#endif /* RTKDATA_GNSS_ACK_H */
