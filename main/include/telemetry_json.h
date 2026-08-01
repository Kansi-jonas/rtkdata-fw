/* Heartbeat telemetry serialization. Platform-neutral C99 (host-tested in
 * tests/host/test_telemetry_json.c); no ESP, no cJSON, no allocation.
 *
 * Contract: ALL OR NOTHING. The builder either writes one complete,
 * well-formed JSON member into dst (NUL-terminated, returns its length), or
 * writes nothing at all and returns -1. A truncated JSON member is the
 * telemetry-sized edition of a truncated RTCM frame, and we do not ship that
 * failure mode twice. */

#ifndef RTKDATA_TELEMETRY_JSON_H
#define RTKDATA_TELEMETRY_JSON_H

#include <stddef.h>

#include "interface/ntrip.h"

/* Worst case, computed not guessed: 15 members, 185 bytes of key text,
 * 4 bytes of punctuation per member, 10 uint64 values at <= 20 digits,
 * 5 uint32 values at <= 10 digits, "tx":{ prefix and } suffix (7).
 * 185 + 60 + 200 + 50 + 7 = 502. The test suite proves the bound with every
 * counter at its maximum. */
#define TELEMETRY_TX_JSON_MAX 512

/* Writes `"tx":{...}` (a JSON member, no trailing comma) for the given
 * totals. Returns the length written, or -1 when dst/totals is NULL or the
 * result would not fit cap (dst is left untouched in every failure case). */
int telemetry_tx_json(char *dst, size_t cap, const ntrip_tx_totals_t *t,
                      unsigned long uart_event_post_drops);

#endif /* RTKDATA_TELEMETRY_JSON_H */
