/* See ntrip_tx_core.h for the architecture. Platform-neutral C99: this file
 * must keep compiling on the host (tests/host/) as well as under ESP-IDF. */

#include <errno.h>
#include <string.h>

#include "interface/ntrip_tx_core.h"

/* ---------------------------------------------------------------- CRC24Q -- */

/* CRC-24Q (RTCM 10403 / Qualcomm), poly 0x1864CFB, init 0, no reflection. */
#define CRC24Q_POLY 0x1864CFBu

static uint32_t s_crc24q_table[256];
static bool s_crc24q_table_ready = false;

static void crc24q_build_table(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i << 16;
        for (int b = 0; b < 8; b++) {
            crc <<= 1;
            if (crc & 0x1000000u) crc ^= CRC24Q_POLY;
        }
        s_crc24q_table[i] = crc & 0xFFFFFFu;
    }
    s_crc24q_table_ready = true;
}

uint32_t rtcm_crc24q(const uint8_t *data, size_t len) {
    if (!s_crc24q_table_ready) crc24q_build_table();

    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = ((crc << 8) & 0xFFFFFFu) ^ s_crc24q_table[((crc >> 16) ^ data[i]) & 0xFFu];
    }
    return crc;
}

/* ---------------------------------------------------------------- Parser -- */

void rtcm_parser_init(rtcm_parser_t *p) {
    memset(p, 0, sizeof(*p));
    if (!s_crc24q_table_ready) crc24q_build_table();
}

/* Discard buffered bytes [0, from) plus everything up to the next 0xD3 at or
 * after `from`, counting all of it as garbage. Afterwards the buffer is empty
 * or starts with 0xD3. */
static void parser_hunt_in_buf(rtcm_parser_t *p, uint16_t from) {
    uint16_t i = from;
    while (i < p->have && p->buf[i] != 0xD3) i++;
    p->bytes_discarded += i;
    if (i < p->have) {
        memmove(p->buf, p->buf + i, (size_t)(p->have - i));
        p->have -= i;
    } else {
        p->have = 0;
    }
    p->need = 0;
}

/* Digest whatever is buffered until more input is required. On return, one of
 * these invariants holds:
 *   have == 0, or
 *   buf[0] == 0xD3 and have < 3, or
 *   buf[0] == 0xD3 and need is set and have < need.
 * rtcm_parser_feed relies on exactly this. */
static void parser_process(rtcm_parser_t *p, uint32_t now_ms,
                           rtcm_frame_cb cb, void *cb_ctx) {
    for (;;) {
        if (p->have < 3) {
            p->need = 0;
            return;
        }

        /* 6 reserved bits must be zero; a nonzero value is a false preamble,
         * not a frame. The CRC is the real gate below. */
        if (p->buf[1] & 0xFC) {
            parser_hunt_in_buf(p, 1);   /* buf[0] was a false 0xD3 */
            continue;
        }

        uint16_t need = (uint16_t)((((p->buf[1] & 0x03u) << 8) | p->buf[2]) + 6u);
        p->need = need;
        if (p->have < need) return;

        uint32_t want = ((uint32_t)p->buf[need - 3] << 16) |
                        ((uint32_t)p->buf[need - 2] << 8) |
                         (uint32_t)p->buf[need - 1];

        if (rtcm_crc24q(p->buf, (size_t)(need - 3)) == want) {
            p->frames_ok++;
            p->bytes_ok += need;
            if (cb) cb(cb_ctx, p->buf, need, now_ms);

            /* consume the frame; a resync may have buffered bytes beyond it */
            if (p->have > need) {
                memmove(p->buf, p->buf + need, (size_t)(p->have - need));
            }
            p->have -= need;
            p->need = 0;
            if (p->have > 0 && p->buf[0] != 0xD3) {
                parser_hunt_in_buf(p, 0);
            }
        } else {
            p->crc_errors++;
            parser_hunt_in_buf(p, 1);   /* false preamble; rescan remainder */
        }
    }
}

void rtcm_parser_feed(rtcm_parser_t *p, const uint8_t *data, size_t len,
                      uint32_t now_ms, rtcm_frame_cb cb, void *cb_ctx) {
    size_t i = 0;

    while (i < len) {
        if (p->have == 0) {
            /* hunt for the preamble directly in the input */
            while (i < len && data[i] != 0xD3) {
                p->bytes_discarded++;
                i++;
            }
            if (i >= len) return;
            p->buf[0] = 0xD3;
            p->have = 1;
            i++;
        }

        /* Top up toward the next decision point: 3 bytes for the header,
         * then `need` bytes for the whole frame. */
        uint16_t target = (p->have < 3) ? 3 : p->need;
        size_t   want   = (size_t)(target - p->have);
        size_t   avail  = len - i;
        size_t   take   = (want < avail) ? want : avail;

        memcpy(p->buf + p->have, data + i, take);
        p->have += (uint16_t)take;
        i += take;

        if (p->have == target) {
            parser_process(p, now_ms, cb, cb_ctx);
        }
    }
}

/* ------------------------------------------------------------------ Ring -- */

static void ring_lock(ntrip_ring_t *r)   { if (r->lock)   r->lock(r->lock_ctx); }
static void ring_unlock(ntrip_ring_t *r) { if (r->unlock) r->unlock(r->lock_ctx); }

void ntrip_ring_init(ntrip_ring_t *r, ntrip_ring_lock_fn lock,
                     ntrip_ring_lock_fn unlock, void *lock_ctx) {
    memset(r, 0, sizeof(*r));
    r->lock = lock;
    r->unlock = unlock;
    r->lock_ctx = lock_ctx;
}

void ntrip_ring_push(ntrip_ring_t *r, const uint8_t *frame, uint16_t len,
                     uint32_t now_ms) {
    if (len < 6 || len > RTCM_MAX_FRAME_B) return;

    ring_lock(r);

    ntrip_ring_slot_t *s = &r->slot[r->next_seq % RTCM_RING_SLOTS];
    s->seq = r->next_seq;
    s->received_ms = now_ms;
    s->len = len;
    memcpy(s->data, frame, len);
    r->cum_bytes += len;
    s->cum_bytes = r->cum_bytes;
    r->next_seq++;
    r->pushed_frames++;

    ring_unlock(r);
}

/* Oldest seq still resident. Caller holds the lock. */
static uint32_t ring_oldest_seq(const ntrip_ring_t *r) {
    return (r->next_seq > RTCM_RING_SLOTS) ? (r->next_seq - RTCM_RING_SLOTS) : 0;
}

/* -------------------------------------------------------------- Sender ---- */

void ntrip_tx_init(ntrip_tx_t *tx) {
    memset(tx, 0, sizeof(*tx));
}

void ntrip_tx_reset_conn(ntrip_tx_t *tx, ntrip_ring_t *ring, uint32_t now_ms) {
    ntrip_tx_abort(tx);

    ring_lock(ring);
    /* Complete frames that were waiting in the ring will never be sent on
     * the new connection; count them, they must not vanish from the byte
     * balance (review 2026-08-01, finding "waiting frames uncounted"). */
    if (tx->cursor_seq < ring->next_seq) {
        tx->skipped_frames += ring->next_seq - tx->cursor_seq;
        tx->skipped_bytes += ring->cum_bytes - tx->cum_through;
    }
    tx->cursor_seq = ring->next_seq;   /* start with the next NEW frame */
    tx->cum_through = ring->cum_bytes;
    ring_unlock(ring);

    tx->last_progress_ms = now_ms;
}

void ntrip_tx_abort(ntrip_tx_t *tx) {
    if (tx->current_off < tx->current_len) {
        tx->dropped_bytes += (uint64_t)(tx->current_len - tx->current_off);
    }
    tx->current_len = 0;
    tx->current_off = 0;
}

bool ntrip_tx_stalled(const ntrip_tx_t *tx, uint32_t now_ms,
                      uint32_t timeout_ms) {
    if (tx->current_off >= tx->current_len) return false;   /* nothing pending */
    return (uint32_t)(now_ms - tx->last_progress_ms) > timeout_ms;
}

bool ntrip_tx_frame_overdue(const ntrip_tx_t *tx, uint32_t now_ms,
                            uint32_t deadline_ms) {
    if (tx->current_off >= tx->current_len) return false;   /* nothing pending */
    return (uint32_t)(now_ms - tx->current_received_ms) > deadline_ms;
}

/* Copy the next eligible frame from the ring into tx->current.
 * Returns true if a frame was copied. */
static bool tx_copy_next(ntrip_tx_t *tx, ntrip_ring_t *ring, uint32_t now_ms,
                         uint32_t stale_ms) {
    bool copied = false;

    ring_lock(ring);

    /* Fell off the back of the ring? Jump to the oldest resident frame and
     * account exactly what was lost. Whole frames only, never a partial. */
    uint32_t oldest = ring_oldest_seq(ring);
    if (tx->cursor_seq < oldest) {
        const ntrip_ring_slot_t *o = &ring->slot[oldest % RTCM_RING_SLOTS];
        uint64_t cum_before_oldest = o->cum_bytes - o->len;
        tx->skipped_bytes += cum_before_oldest - tx->cum_through;
        tx->skipped_frames += oldest - tx->cursor_seq;
        tx->cum_through = cum_before_oldest;
        tx->cursor_seq = oldest;
    }

    /* Old corrections are worse than none: drop frames that aged past
     * stale_ms before their first send attempt. */
    while (tx->cursor_seq < ring->next_seq) {
        const ntrip_ring_slot_t *s = &ring->slot[tx->cursor_seq % RTCM_RING_SLOTS];
        uint32_t age = now_ms - s->received_ms;

        if (stale_ms > 0 && age > stale_ms) {
            tx->dropped_frames_stale++;
            tx->dropped_stale_bytes += s->len;
            tx->cum_through = s->cum_bytes;
            tx->cursor_seq++;
            continue;
        }

        memcpy(tx->current, s->data, s->len);
        tx->current_len = s->len;
        tx->current_off = 0;
        tx->current_received_ms = s->received_ms;
        /* The progress clock measures THIS transmission, not the idle time
         * before it: without the restart, the first EAGAIN after a long
         * quiet period would trip the stall check instantly (review
         * 2026-08-01, finding "idle then EAGAIN reconnects"). */
        tx->last_progress_ms = now_ms;
        tx->copied_bytes += s->len;
        tx->cum_through = s->cum_bytes;
        tx->cursor_seq++;
        if (age > tx->max_queue_age_ms) tx->max_queue_age_ms = age;
        copied = true;
        break;
    }

    ring_unlock(ring);
    return copied;
}

ntrip_tx_poll_result_t ntrip_tx_poll(ntrip_tx_t *tx, ntrip_ring_t *ring,
                                     uint32_t now_ms, int max_sends,
                                     uint32_t stale_ms,
                                     ntrip_tx_send_fn send_fn, void *send_ctx) {
    ntrip_tx_poll_result_t result = NTRIP_TX_POLL_IDLE;

    for (int round = 0; round < max_sends; round++) {
        if (tx->current_off >= tx->current_len) {
            if (!tx_copy_next(tx, ring, now_ms, stale_ms)) {
                return result;   /* all caught up */
            }
        }

        int err = 0;
        int n = send_fn(send_ctx, tx->current + tx->current_off,
                        (size_t)(tx->current_len - tx->current_off), &err);

        if (n > 0) {
            tx->current_off += (uint16_t)n;
            tx->accepted_to_lwip += (uint64_t)n;
            tx->last_progress_ms = now_ms;
            result = NTRIP_TX_POLL_PROGRESS;
            if (tx->current_off == tx->current_len) {
                tx->sent_frames++;
                tx->current_len = 0;
                tx->current_off = 0;
            }
        } else if (n < 0 && (err == EAGAIN || err == EWOULDBLOCK)) {
            tx->eagain_count++;
            return NTRIP_TX_POLL_WOULDBLOCK;
        } else {
            /* hard error, or a 0 return that a stream socket must not give */
            tx->last_sock_errno = (n < 0) ? err : ECONNRESET;
            ntrip_tx_abort(tx);
            return NTRIP_TX_POLL_ERROR;
        }
    }

    return result;
}
