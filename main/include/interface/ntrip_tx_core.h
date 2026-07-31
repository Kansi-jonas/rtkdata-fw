/* NTRIP TX core: RTCM3 framing between the GNSS UART and the caster sockets.
 *
 * Data path (see the 2026-07-31 send-path review in the integrity-engine repo):
 *
 *   UART chunks -> rtcm_parser (stateful, CRC24Q)
 *              -> ntrip_ring   (shared, fixed, complete frames only)
 *              -> ntrip_tx     (per NTRIP instance: cursor + partial-send state)
 *              -> non-blocking socket owned by that instance's server task
 *
 * The UART path never touches a socket; a socket is only written by the one
 * task that owns it. Frames enter the ring only after their CRC24Q verified,
 * so a reader can never emit a truncated or corrupt frame boundary: it either
 * finishes the frame it started or the connection dies.
 *
 * This file is platform-neutral C99 on purpose: it is compiled by ESP-IDF for
 * the device and by a host compiler for tests/host/test_ntrip_tx_core.c. No
 * FreeRTOS, no lwIP, no ESP headers. Time is passed in, sending is a callback,
 * locking is a pair of optional hooks on the ring.
 */

#ifndef NTRIP_TX_CORE_H
#define NTRIP_TX_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* RTCM3: 3 header bytes + max 1023 payload + 3 CRC bytes. */
#define RTCM_MAX_FRAME_B 1029
#define RTCM_RING_SLOTS  16

/* Sender tuning. Values are policy defaults; callers pass them explicitly so
 * host tests can exercise the edges. */
#define NTRIP_TX_MAX_SENDS_PER_POLL 2
#define NTRIP_TX_STALE_DROP_MS      2000
#define NTRIP_TX_PROGRESS_TIMEOUT_MS 5000

/* ---------------------------------------------------------------- CRC24Q -- */

uint32_t rtcm_crc24q(const uint8_t *data, size_t len);

/* ---------------------------------------------------------------- Parser -- */

typedef struct {
    uint8_t  buf[RTCM_MAX_FRAME_B];
    uint16_t have;             /* bytes accumulated in buf */
    uint16_t need;             /* full frame length once header parsed, else 0 */

    uint64_t frames_ok;
    uint64_t bytes_ok;         /* bytes of CRC-valid frames emitted */
    uint64_t bytes_discarded;  /* garbage between/inside broken frames */
    uint64_t crc_errors;
} rtcm_parser_t;

typedef void (*rtcm_frame_cb)(void *ctx, const uint8_t *frame, uint16_t len,
                              uint32_t now_ms);

void rtcm_parser_init(rtcm_parser_t *p);

/* Feed a chunk of raw UART bytes. Emits every complete CRC-valid frame via cb.
 * Chunk boundaries are arbitrary; state survives across calls. */
void rtcm_parser_feed(rtcm_parser_t *p, const uint8_t *data, size_t len,
                      uint32_t now_ms, rtcm_frame_cb cb, void *cb_ctx);

/* ------------------------------------------------------------------ Ring -- */

typedef struct {
    uint32_t seq;
    uint32_t received_ms;
    uint16_t len;
    uint64_t cum_bytes;        /* ring-wide valid bytes INCLUDING this frame */
    uint8_t  data[RTCM_MAX_FRAME_B];
} ntrip_ring_slot_t;

typedef void (*ntrip_ring_lock_fn)(void *ctx);

typedef struct {
    ntrip_ring_slot_t slot[RTCM_RING_SLOTS];
    uint32_t next_seq;         /* seq the next pushed frame will get */
    uint64_t cum_bytes;        /* total valid frame bytes ever pushed */
    uint64_t pushed_frames;

    ntrip_ring_lock_fn lock;   /* both may be NULL (single-threaded tests) */
    ntrip_ring_lock_fn unlock;
    void *lock_ctx;
} ntrip_ring_t;

void ntrip_ring_init(ntrip_ring_t *r, ntrip_ring_lock_fn lock,
                     ntrip_ring_lock_fn unlock, void *lock_ctx);

/* Writer side (UART path). len must be 6..RTCM_MAX_FRAME_B; the frame must
 * already be CRC-valid. Never blocks: the oldest slot is overwritten when the
 * ring is full, lagging readers detect the loss via seq. */
void ntrip_ring_push(ntrip_ring_t *r, const uint8_t *frame, uint16_t len,
                     uint32_t now_ms);

/* -------------------------------------------------------------- Sender ---- */

/* Send callback contract: attempt to send len bytes, return >0 bytes accepted,
 * or <0 with *out_errno set (EAGAIN/EWOULDBLOCK for "try later", anything else
 * is fatal for the connection). A 0 return is treated as fatal. */
typedef int (*ntrip_tx_send_fn)(void *ctx, const uint8_t *buf, size_t len,
                                int *out_errno);

typedef enum {
    NTRIP_TX_POLL_IDLE = 0,    /* nothing pending, all caught up */
    NTRIP_TX_POLL_PROGRESS,    /* accepted bytes this round */
    NTRIP_TX_POLL_WOULDBLOCK,  /* socket full; wait for next round */
    NTRIP_TX_POLL_ERROR,       /* hard socket error; reconnect */
} ntrip_tx_poll_result_t;

typedef struct {
    /* connection-scoped */
    uint32_t cursor_seq;       /* next ring seq to copy */
    uint64_t cum_through;      /* ring cum_bytes through last consumed frame */
    uint8_t  current[RTCM_MAX_FRAME_B];
    uint16_t current_len;
    uint16_t current_off;
    uint32_t last_progress_ms;

    /* cumulative over the device lifetime, 64-bit on purpose */
    uint64_t copied_bytes;         /* frames copied out of the ring */
    uint64_t accepted_to_lwip;     /* bytes the stack accepted (NOT on-wire) */
    uint64_t sent_frames;
    uint64_t dropped_bytes;        /* remainder lost on error/stall/reconnect */
    uint64_t skipped_bytes;        /* lost to ring overwrite while lagging */
    uint32_t skipped_frames;
    uint32_t dropped_frames_stale; /* older than stale_ms before first send */
    uint64_t dropped_stale_bytes;
    uint64_t eagain_count;
    uint32_t reconnects;
    uint32_t max_queue_age_ms;
    int      last_sock_errno;
} ntrip_tx_t;

void ntrip_tx_init(ntrip_tx_t *tx);

/* Call on every (re)connect, with the ring lock available: the new connection
 * starts at the NEXT complete frame; a partially sent frame from the previous
 * connection is counted into dropped_bytes and never resumed. */
void ntrip_tx_reset_conn(ntrip_tx_t *tx, ntrip_ring_t *ring, uint32_t now_ms);

/* Drive the sender: copy next frame(s) from the ring, attempt at most
 * max_sends send() calls, account everything. stale_ms > 0 drops frames older
 * than that before their first send attempt. */
ntrip_tx_poll_result_t ntrip_tx_poll(ntrip_tx_t *tx, ntrip_ring_t *ring,
                                     uint32_t now_ms, int max_sends,
                                     uint32_t stale_ms,
                                     ntrip_tx_send_fn send_fn, void *send_ctx);

/* True when a partially sent frame has made no progress for timeout_ms. */
bool ntrip_tx_stalled(const ntrip_tx_t *tx, uint32_t now_ms,
                      uint32_t timeout_ms);

/* Abandon the current frame (connection is going away): counts the unsent
 * remainder into dropped_bytes. Idempotent. */
void ntrip_tx_abort(ntrip_tx_t *tx);

/* Bytes currently copied out of the ring but not yet accepted by the stack. */
static inline uint32_t ntrip_tx_pending(const ntrip_tx_t *tx) {
    return (uint32_t)(tx->current_len - tx->current_off);
}

#endif /* NTRIP_TX_CORE_H */
