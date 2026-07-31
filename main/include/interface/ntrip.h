#ifndef ESP32_XBEE_NTRIP_H
#define ESP32_XBEE_NTRIP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NTRIP_GENERIC_NAME "RTKdata"

#define NTRIP_SERVER_NAME NTRIP_GENERIC_NAME "_Server"

#define NTRIP_PORT_DEFAULT 2101
#define NTRIP_MOUNTPOINT_DEFAULT "DEFAULT"
#define NTRIP_KEEP_ALIVE_THRESHOLD 10000
#define NTRIP_MAX_INSTANCES 10

#define NEWLINE "\r\n"
#define NEWLINE_LENGTH 2

void ntrip_server_init();
void ntrip_server_reconnect_all(void);

bool ntrip_response_ok(void *response);
bool ntrip_response_sourcetable_ok(void *response);

// Per-instance sender counters for /ntrip/tx_stats. accepted_to_lwip counts
// bytes the TCP stack accepted, NOT bytes on the wire.
typedef struct {
    int      index;
    bool     connected;
    uint64_t accepted_to_lwip;
    uint64_t sent_frames;
    uint64_t copied_bytes;
    uint64_t dropped_bytes;
    uint64_t skipped_bytes;
    uint32_t skipped_frames;
    uint32_t dropped_frames_stale;
    uint64_t dropped_stale_bytes;
    uint64_t eagain_count;
    uint32_t reconnects;
    uint32_t pending;
    uint32_t max_queue_age_ms;
    int      last_sock_errno;
} ntrip_tx_stats_t;

// Shared ingest-side counters (RTCM parser + frame ring).
typedef struct {
    uint64_t frames_ok;
    uint64_t bytes_ok;
    uint64_t bytes_discarded;
    uint64_t crc_errors;
    uint64_t ring_pushed_frames;
    uint64_t ring_cum_bytes;
} ntrip_ingest_stats_t;

size_t ntrip_server_tx_stats(ntrip_tx_stats_t *out, size_t max);
void ntrip_server_ingest_stats(ntrip_ingest_stats_t *out);

#endif //ESP32_XBEE_NTRIP_H
