/* See telemetry_json.h for the contract. */

#include <stdio.h>
#include <string.h>

#include "telemetry_json.h"

int telemetry_tx_json(char *dst, size_t cap, const ntrip_tx_totals_t *t,
                      unsigned long uart_event_post_drops) {

    if (dst == NULL || t == NULL) return -1;

    char local[TELEMETRY_TX_JSON_MAX];

    int n = snprintf(local, sizeof(local),
        "\"tx\":{"
        "\"frames_ok\":%llu,"
        "\"bytes_ok\":%llu,"
        "\"bytes_discarded\":%llu,"
        "\"crc_errors\":%llu,"
        "\"accepted_to_lwip\":%llu,"
        "\"sent_frames\":%llu,"
        "\"dropped_bytes\":%llu,"
        "\"skipped_bytes\":%llu,"
        "\"dropped_stale_bytes\":%llu,"
        "\"eagain\":%llu,"
        "\"reconnects\":%lu,"
        "\"max_queue_age_ms\":%lu,"
        "\"instances\":%lu,"
        "\"connected\":%lu,"
        "\"uart_event_post_drops\":%lu,"
        "\"req_fresh\":%s,"
        "\"req_missing\":%lu"
        "}",
        (unsigned long long)t->frames_ok,
        (unsigned long long)t->bytes_ok,
        (unsigned long long)t->bytes_discarded,
        (unsigned long long)t->crc_errors,
        (unsigned long long)t->accepted_to_lwip,
        (unsigned long long)t->sent_frames,
        (unsigned long long)t->dropped_bytes,
        (unsigned long long)t->skipped_bytes,
        (unsigned long long)t->dropped_stale_bytes,
        (unsigned long long)t->eagain_count,
        (unsigned long)t->reconnects,
        (unsigned long)t->max_queue_age_ms,
        (unsigned long)t->instances,
        (unsigned long)t->connected,
        uart_event_post_drops,
        t->req_fresh ? "true" : "false",
        (unsigned long)t->req_missing);

    /* n >= sizeof(local) cannot happen by the documented bound; checked
     * anyway so a future field addition fails loudly in tests, not quietly
     * on the wire. */
    if (n <= 0 || (size_t)n >= sizeof(local)) return -1;
    if ((size_t)n >= cap) return -1;

    memcpy(dst, local, (size_t)n + 1);
    return n;
}
