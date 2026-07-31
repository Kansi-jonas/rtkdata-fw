/* Host tests for ntrip_tx_core (parser + ring + sender). Compiled and run on
 * the host, no ESP-IDF: gcc test_ntrip_tx_core.c ../../main/interface/ntrip_tx_core.c
 *
 * Mandatory coverage (send-path review 2026-07-31):
 *   1. every possible split of a frame across UART chunks
 *   2. real CRC24Q (known RTCM vector + independent bitwise implementation)
 *   3. 0xD3 0x00 inside a payload; false preambles spanning into real frames
 *   4. positive short send followed by EAGAIN
 *   5. positive short send followed by a hard error
 *   6. reconnect with a partially sent frame (never resumed, fully counted)
 *   7. ring overflow with fast and slow readers
 *   8. ten instances concurrently (schedule-interleaved, not thread-parallel)
 *   9. init/limit failure paths (push length guards, reserved-bit reject)
 *  10. property test: byte balance over a randomized garbage+frames stream
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "interface/ntrip_tx_core.h"

static int g_checks = 0;
static int g_fails = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        g_fails++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define CHECK_EQ_U64(a, b) do { \
    g_checks++; \
    unsigned long long _a = (unsigned long long)(a), _b = (unsigned long long)(b); \
    if (_a != _b) { \
        g_fails++; \
        printf("FAIL %s:%d: %s=%llu != %s=%llu\n", __FILE__, __LINE__, #a, _a, #b, _b); \
    } \
} while (0)

/* ------------------------------------------------------------------ PRNG -- */

static uint32_t g_rng = 0xC0FFEE01u;

static uint32_t prng(void) {
    uint32_t x = g_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_rng = x;
    return x;
}

/* ------------------------------------------- independent bitwise CRC24Q -- */

static uint32_t crc24q_bitwise(const uint8_t *d, size_t len) {
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint32_t)d[i]) << 16;
        for (int b = 0; b < 8; b++) {
            crc <<= 1;
            if (crc & 0x1000000u) crc ^= 0x1864CFBu;
        }
    }
    return crc & 0xFFFFFFu;
}

/* Build a valid frame: header + patterned payload + CRC (bitwise, so the
 * table implementation under test is cross-checked by construction). */
static uint16_t make_frame(uint8_t *out, uint16_t payload_len, uint32_t tag) {
    out[0] = 0xD3;
    out[1] = (uint8_t)((payload_len >> 8) & 0x03);
    out[2] = (uint8_t)(payload_len & 0xFF);
    for (uint16_t i = 0; i < payload_len; i++) {
        out[3 + i] = (uint8_t)(0x40u + ((tag * 31u + i * 7u) & 0x3Fu)); /* no 0xD3 */
    }
    uint32_t crc = crc24q_bitwise(out, (size_t)(3 + payload_len));
    out[3 + payload_len]     = (uint8_t)(crc >> 16);
    out[3 + payload_len + 1] = (uint8_t)(crc >> 8);
    out[3 + payload_len + 2] = (uint8_t)crc;
    return (uint16_t)(payload_len + 6);
}

/* ------------------------------------------------------- frame collector -- */

#define COLLECT_MAX 64

typedef struct {
    uint32_t count;
    uint64_t bytes;
    uint16_t lens[COLLECT_MAX];
    uint8_t  frames[COLLECT_MAX][RTCM_MAX_FRAME_B];
} collector_t;

static void collect_cb(void *ctx, const uint8_t *frame, uint16_t len,
                       uint32_t now_ms) {
    (void)now_ms;
    collector_t *c = (collector_t *)ctx;
    if (c->count < COLLECT_MAX) {
        memcpy(c->frames[c->count], frame, len);
        c->lens[c->count] = len;
    }
    c->count++;
    c->bytes += len;
}

/* counting collector for big streams (no storage) */
typedef struct {
    uint64_t count;
    uint64_t bytes;
    uint64_t crc_bad;   /* emitted frames failing the independent CRC */
} counter_t;

static void count_cb(void *ctx, const uint8_t *frame, uint16_t len,
                     uint32_t now_ms) {
    (void)now_ms;
    counter_t *c = (counter_t *)ctx;
    c->count++;
    c->bytes += len;
    uint32_t want = ((uint32_t)frame[len - 3] << 16) |
                    ((uint32_t)frame[len - 2] << 8) | frame[len - 1];
    if (frame[0] != 0xD3 || crc24q_bitwise(frame, (size_t)(len - 3)) != want) {
        c->crc_bad++;
    }
}

/* ------------------------------------------------------------- mock send -- */

typedef struct { int kind; int val; } send_act_t;  /* 0 accept, 1 EAGAIN, 2 err */

typedef struct {
    send_act_t acts[16];
    int n_acts;
    int next;
    int random_mode;       /* script exhausted: 0 = accept all, 1 = randomized */
    uint64_t accepted;
    uint64_t calls;
} mock_send_t;

static void mock_init(mock_send_t *m) { memset(m, 0, sizeof(*m)); }

static void mock_script(mock_send_t *m, int kind, int val) {
    m->acts[m->n_acts].kind = kind;
    m->acts[m->n_acts].val = val;
    m->n_acts++;
}

static int mock_send(void *ctx, const uint8_t *buf, size_t len, int *out_errno) {
    (void)buf;
    mock_send_t *m = (mock_send_t *)ctx;
    m->calls++;

    if (m->next < m->n_acts) {
        send_act_t a = m->acts[m->next++];
        if (a.kind == 1) { *out_errno = EAGAIN; return -1; }
        if (a.kind == 2) { *out_errno = a.val; return -1; }
        int n = (a.val <= 0 || (size_t)a.val > len) ? (int)len : a.val;
        m->accepted += (uint64_t)n;
        return n;
    }

    if (m->random_mode) {
        uint32_t r = prng();
        if ((r % 100u) < 15u) { *out_errno = EAGAIN; return -1; }
        int n = (int)(1u + (r >> 8) % (uint32_t)len);
        m->accepted += (uint64_t)n;
        return n;
    }

    m->accepted += (uint64_t)len;
    return (int)len;
}

/* --------------------------------------------------------- lock counting -- */

static int g_locks = 0, g_unlocks = 0;
static void count_lock(void *ctx)   { (void)ctx; g_locks++; }
static void count_unlock(void *ctx) { (void)ctx; g_unlocks++; }

/* ------------------------------------------------------------- the tests -- */

/* The classic RTCM 10403 example frame (message 1005). Payload byte 3 is
 * 0xD3: doubles as an embedded-preamble test. */
static const uint8_t KNOWN_FRAME[25] = {
    0xD3, 0x00, 0x13, 0x3E, 0xD7, 0xD3, 0x02, 0x02, 0x98, 0x0E, 0xDE, 0xEF,
    0x34, 0xB4, 0xBD, 0x62, 0xAC, 0x09, 0x41, 0x98, 0x6F, 0x33, 0x36, 0x0B,
    0x98
};

static void test_crc_known_vector(void) {
    CHECK_EQ_U64(crc24q_bitwise(KNOWN_FRAME, 22), 0x360B98u);
    CHECK_EQ_U64(rtcm_crc24q(KNOWN_FRAME, 22), 0x360B98u);

    /* table and bitwise must agree on arbitrary data */
    uint8_t buf[257];
    for (int i = 0; i < 257; i++) buf[i] = (uint8_t)prng();
    for (size_t l = 0; l <= 257; l += 13) {
        CHECK_EQ_U64(rtcm_crc24q(buf, l), crc24q_bitwise(buf, l));
    }

    rtcm_parser_t p;
    collector_t col;
    memset(&col, 0, sizeof(col));
    rtcm_parser_init(&p);
    rtcm_parser_feed(&p, KNOWN_FRAME, sizeof(KNOWN_FRAME), 0, collect_cb, &col);
    CHECK_EQ_U64(col.count, 1);
    CHECK_EQ_U64(col.lens[0], 25);
    CHECK(memcmp(col.frames[0], KNOWN_FRAME, 25) == 0);
    CHECK_EQ_U64(p.bytes_discarded, 0);
    CHECK_EQ_U64(p.crc_errors, 0);
}

static void test_all_chunk_splits(void) {
    uint8_t f1[64], f2[128], f3[512], stream[704];
    uint16_t l1 = make_frame(f1, 19, 1);
    uint16_t l2 = make_frame(f2, 94, 2);
    uint16_t l3 = make_frame(f3, 294, 3);
    size_t total = (size_t)l1 + l2 + l3;
    memcpy(stream, f1, l1);
    memcpy(stream + l1, f2, l2);
    memcpy(stream + l1 + l2, f3, l3);

    /* every two-chunk split */
    for (size_t s = 0; s <= total; s++) {
        rtcm_parser_t p;
        collector_t col;
        memset(&col, 0, sizeof(col));
        rtcm_parser_init(&p);
        rtcm_parser_feed(&p, stream, s, 0, collect_cb, &col);
        rtcm_parser_feed(&p, stream + s, total - s, 0, collect_cb, &col);
        if (col.count != 3 || p.bytes_ok != total || p.bytes_discarded != 0) {
            CHECK(0 && "two-chunk split failed");
            printf("  at split %zu: frames=%u ok=%llu disc=%llu\n", s,
                   (unsigned)col.count, (unsigned long long)p.bytes_ok,
                   (unsigned long long)p.bytes_discarded);
            return;
        }
    }
    g_checks++;   /* the loop above counts as one big passed check */

    /* byte-by-byte */
    rtcm_parser_t p;
    collector_t col;
    memset(&col, 0, sizeof(col));
    rtcm_parser_init(&p);
    for (size_t i = 0; i < total; i++) {
        rtcm_parser_feed(&p, stream + i, 1, 0, collect_cb, &col);
    }
    CHECK_EQ_U64(col.count, 3);
    CHECK_EQ_U64(p.bytes_ok, total);
    CHECK(memcmp(col.frames[2], f3, l3) == 0);
}

static void test_embedded_and_false_preambles(void) {
    /* 0xD3 0x00 inside a payload: overwrite pattern bytes, re-CRC */
    uint8_t f[64];
    uint16_t l = make_frame(f, 30, 7);
    f[8] = 0xD3; f[9] = 0x00; f[10] = 0x07;
    uint32_t crc = crc24q_bitwise(f, 33);
    f[33] = (uint8_t)(crc >> 16); f[34] = (uint8_t)(crc >> 8); f[35] = (uint8_t)crc;

    rtcm_parser_t p;
    collector_t col;
    memset(&col, 0, sizeof(col));
    rtcm_parser_init(&p);
    rtcm_parser_feed(&p, f, l, 0, collect_cb, &col);
    CHECK_EQ_U64(col.count, 1);
    CHECK_EQ_U64(p.crc_errors, 0);

    /* false preamble whose declared length spans INTO two real frames */
    uint8_t f1[64], f2[128], stream[256];
    uint16_t l1 = make_frame(f1, 19, 11);
    uint16_t l2 = make_frame(f2, 94, 12);
    size_t n = 0;
    stream[n++] = 0xD3; stream[n++] = 0x00; stream[n++] = 0x20;  /* need=38 */
    for (int i = 0; i < 8; i++) stream[n++] = 0x11;              /* junk, no D3 */
    memcpy(stream + n, f1, l1); n += l1;
    memcpy(stream + n, f2, l2); n += l2;

    memset(&col, 0, sizeof(col));
    rtcm_parser_init(&p);
    rtcm_parser_feed(&p, stream, n, 0, collect_cb, &col);
    CHECK_EQ_U64(col.count, 2);
    CHECK_EQ_U64(p.crc_errors, 1);
    CHECK_EQ_U64(p.bytes_discarded, 11);
    CHECK_EQ_U64(p.bytes_ok, (uint64_t)l1 + l2);
    CHECK(memcmp(col.frames[0], f1, l1) == 0);
    CHECK(memcmp(col.frames[1], f2, l2) == 0);

    /* reserved-bit reject: D3 with nonzero high-6 bits of byte 1 */
    uint8_t bad[3] = { 0xD3, 0x40, 0x05 };
    memset(&col, 0, sizeof(col));
    rtcm_parser_init(&p);
    rtcm_parser_feed(&p, bad, 3, 0, collect_cb, &col);
    rtcm_parser_feed(&p, f1, l1, 0, collect_cb, &col);
    CHECK_EQ_U64(col.count, 1);          /* real frame recovered */
    CHECK_EQ_U64(p.bytes_discarded, 3);  /* the fake header */
}

static void test_max_size_frame(void) {
    static uint8_t f[RTCM_MAX_FRAME_B];
    uint16_t l = make_frame(f, 1023, 42);
    CHECK_EQ_U64(l, RTCM_MAX_FRAME_B);

    rtcm_parser_t p;
    counter_t cnt;
    memset(&cnt, 0, sizeof(cnt));
    rtcm_parser_init(&p);
    /* feed in awkward chunks */
    rtcm_parser_feed(&p, f, 2, 0, count_cb, &cnt);
    rtcm_parser_feed(&p, f + 2, 1000, 0, count_cb, &cnt);
    rtcm_parser_feed(&p, f + 1002, (size_t)l - 1002, 0, count_cb, &cnt);
    CHECK_EQ_U64(cnt.count, 1);
    CHECK_EQ_U64(cnt.bytes, RTCM_MAX_FRAME_B);
    CHECK_EQ_U64(cnt.crc_bad, 0);
}

static void test_ring_guards_and_locks(void) {
    ntrip_ring_t ring;
    g_locks = g_unlocks = 0;
    ntrip_ring_init(&ring, count_lock, count_unlock, NULL);

    uint8_t f[64];
    uint16_t l = make_frame(f, 19, 5);

    ntrip_ring_push(&ring, f, 5, 0);                    /* too short: rejected */
    CHECK_EQ_U64(ring.next_seq, 0);
    ntrip_ring_push(&ring, f, RTCM_MAX_FRAME_B + 1, 0); /* too long: rejected */
    CHECK_EQ_U64(ring.next_seq, 0);
    ntrip_ring_push(&ring, f, l, 123);
    CHECK_EQ_U64(ring.next_seq, 1);
    CHECK_EQ_U64(ring.cum_bytes, l);
    CHECK(g_locks == 1 && g_unlocks == 1);   /* rejected pushes never locked */
}

static void test_sender_short_then_eagain(void) {
    ntrip_ring_t ring;
    ntrip_tx_t tx;
    mock_send_t m;
    uint8_t f[128];
    uint16_t l = make_frame(f, 94, 21);   /* 100 bytes */

    ntrip_ring_init(&ring, NULL, NULL, NULL);
    ntrip_tx_init(&tx);
    ntrip_tx_reset_conn(&tx, &ring, 0);
    ntrip_ring_push(&ring, f, l, 10);

    mock_init(&m);
    mock_script(&m, 0, 10);   /* accept 10 */
    mock_script(&m, 1, 0);    /* EAGAIN */

    ntrip_tx_poll_result_t r =
        ntrip_tx_poll(&tx, &ring, 20, 2, 0, mock_send, &m);
    CHECK(r == NTRIP_TX_POLL_WOULDBLOCK);
    CHECK_EQ_U64(tx.accepted_to_lwip, 10);
    CHECK_EQ_U64(tx.eagain_count, 1);
    CHECK_EQ_U64(ntrip_tx_pending(&tx), (uint64_t)l - 10);
    CHECK(!ntrip_tx_stalled(&tx, 20, 5000));
    CHECK(ntrip_tx_stalled(&tx, 5021, 5000));

    r = ntrip_tx_poll(&tx, &ring, 30, 2, 0, mock_send, &m);
    CHECK(r == NTRIP_TX_POLL_PROGRESS);
    CHECK_EQ_U64(tx.accepted_to_lwip, l);
    CHECK_EQ_U64(tx.sent_frames, 1);
    CHECK_EQ_U64(tx.dropped_bytes, 0);
    CHECK_EQ_U64(tx.copied_bytes, tx.accepted_to_lwip + ntrip_tx_pending(&tx) + tx.dropped_bytes);
}

static void test_sender_short_then_hard_error(void) {
    ntrip_ring_t ring;
    ntrip_tx_t tx;
    mock_send_t m;
    uint8_t f[128];
    uint16_t l = make_frame(f, 94, 22);

    ntrip_ring_init(&ring, NULL, NULL, NULL);
    ntrip_tx_init(&tx);
    ntrip_tx_reset_conn(&tx, &ring, 0);
    ntrip_ring_push(&ring, f, l, 10);

    mock_init(&m);
    mock_script(&m, 0, 10);
    mock_script(&m, 2, EPIPE);

    ntrip_tx_poll_result_t r =
        ntrip_tx_poll(&tx, &ring, 20, 4, 0, mock_send, &m);
    CHECK(r == NTRIP_TX_POLL_ERROR);
    CHECK_EQ_U64(tx.accepted_to_lwip, 10);
    CHECK_EQ_U64(tx.dropped_bytes, (uint64_t)l - 10);
    CHECK(tx.last_sock_errno == EPIPE);
    CHECK_EQ_U64(ntrip_tx_pending(&tx), 0);
    CHECK_EQ_U64(tx.copied_bytes, tx.accepted_to_lwip + tx.dropped_bytes);
}

static void test_reconnect_with_partial_frame(void) {
    ntrip_ring_t ring;
    ntrip_tx_t tx;
    mock_send_t m;
    uint8_t f1[128], f2[128];
    uint16_t l1 = make_frame(f1, 94, 31);
    uint16_t l2 = make_frame(f2, 44, 32);

    ntrip_ring_init(&ring, NULL, NULL, NULL);
    ntrip_tx_init(&tx);
    ntrip_tx_reset_conn(&tx, &ring, 0);
    ntrip_ring_push(&ring, f1, l1, 10);

    mock_init(&m);
    mock_script(&m, 0, 40);
    mock_script(&m, 1, 0);
    ntrip_tx_poll(&tx, &ring, 20, 2, 0, mock_send, &m);
    CHECK_EQ_U64(tx.accepted_to_lwip, 40);
    CHECK_EQ_U64(ntrip_tx_pending(&tx), (uint64_t)l1 - 40);

    /* connection dies; reconnect. The partial frame is dropped, never resumed,
     * and the new connection starts at the NEXT frame. */
    ntrip_tx_reset_conn(&tx, &ring, 1000);
    CHECK_EQ_U64(tx.dropped_bytes, (uint64_t)l1 - 40);
    CHECK_EQ_U64(ntrip_tx_pending(&tx), 0);

    ntrip_ring_push(&ring, f2, l2, 1010);
    mock_init(&m);
    ntrip_tx_poll_result_t r =
        ntrip_tx_poll(&tx, &ring, 1020, 4, 0, mock_send, &m);
    CHECK(r == NTRIP_TX_POLL_PROGRESS);
    CHECK_EQ_U64(tx.sent_frames, 1);                 /* only f2 */
    CHECK_EQ_U64(tx.accepted_to_lwip, 40u + l2);     /* f1 prefix + all of f2 */
    CHECK_EQ_U64(tx.copied_bytes,
                 tx.accepted_to_lwip + ntrip_tx_pending(&tx) + tx.dropped_bytes);
}

static void test_ring_overflow_fast_slow(void) {
    ntrip_ring_t ring;
    ntrip_tx_t fast, slow;
    mock_send_t mf, ms;
    uint8_t f[64];

    ntrip_ring_init(&ring, NULL, NULL, NULL);
    ntrip_tx_init(&fast);
    ntrip_tx_init(&slow);
    ntrip_tx_reset_conn(&fast, &ring, 0);
    ntrip_tx_reset_conn(&slow, &ring, 0);
    mock_init(&mf);
    mock_init(&ms);

    const int N = 40;
    uint16_t l = make_frame(f, 44, 50);   /* 50 bytes each */
    for (int i = 0; i < N; i++) {
        f[4] = (uint8_t)i;   /* vary content; re-CRC */
        uint32_t crc = crc24q_bitwise(f, (size_t)(l - 3));
        f[l - 3] = (uint8_t)(crc >> 16);
        f[l - 2] = (uint8_t)(crc >> 8);
        f[l - 1] = (uint8_t)crc;
        ntrip_ring_push(&ring, f, l, (uint32_t)(100 + i));
        /* fast reader drains every push; stale_ms=0 disables age drops */
        while (ntrip_tx_poll(&fast, &ring, (uint32_t)(100 + i), 4, 0,
                             mock_send, &mf) == NTRIP_TX_POLL_PROGRESS) {}
    }
    CHECK_EQ_U64(fast.sent_frames, N);
    CHECK_EQ_U64(fast.skipped_frames, 0);
    CHECK_EQ_U64(fast.accepted_to_lwip, (uint64_t)N * l);

    /* slow reader wakes up after everything was pushed: the ring holds the
     * last RTCM_RING_SLOTS frames, the rest must be counted as skipped */
    while (ntrip_tx_poll(&slow, &ring, 200, 4, 0, mock_send, &ms) ==
           NTRIP_TX_POLL_PROGRESS) {}
    CHECK_EQ_U64(slow.sent_frames, RTCM_RING_SLOTS);
    CHECK_EQ_U64(slow.skipped_frames, N - RTCM_RING_SLOTS);
    CHECK_EQ_U64(slow.skipped_bytes, (uint64_t)(N - RTCM_RING_SLOTS) * l);
    CHECK_EQ_U64(slow.copied_bytes + slow.skipped_bytes, ring.cum_bytes);
}

static void test_stale_frame_drop(void) {
    ntrip_ring_t ring;
    ntrip_tx_t tx;
    mock_send_t m;
    uint8_t f[64];
    uint16_t l = make_frame(f, 19, 60);

    ntrip_ring_init(&ring, NULL, NULL, NULL);
    ntrip_tx_init(&tx);
    ntrip_tx_reset_conn(&tx, &ring, 0);
    mock_init(&m);

    ntrip_ring_push(&ring, f, l, 1000);
    ntrip_ring_push(&ring, f, l, 3500);

    /* at t=4000 with stale_ms=2000: first frame (age 3000) dropped, second
     * (age 500) sent */
    ntrip_tx_poll_result_t r = ntrip_tx_poll(&tx, &ring, 4000, 4,
                                             NTRIP_TX_STALE_DROP_MS,
                                             mock_send, &m);
    CHECK(r == NTRIP_TX_POLL_PROGRESS);
    CHECK_EQ_U64(tx.dropped_frames_stale, 1);
    CHECK_EQ_U64(tx.dropped_stale_bytes, l);
    CHECK_EQ_U64(tx.sent_frames, 1);
    CHECK_EQ_U64(tx.copied_bytes + tx.skipped_bytes + tx.dropped_stale_bytes,
                 ring.cum_bytes);
}

static void test_ten_instances(void) {
    static ntrip_ring_t ring;
    static ntrip_tx_t tx[10];
    static mock_send_t m[10];
    uint8_t f[512];

    ntrip_ring_init(&ring, count_lock, count_unlock, NULL);
    g_locks = g_unlocks = 0;
    for (int k = 0; k < 10; k++) {
        ntrip_tx_init(&tx[k]);
        ntrip_tx_reset_conn(&tx[k], &ring, 0);
        mock_init(&m[k]);
        m[k].random_mode = 1;
    }

    for (int i = 0; i < 200; i++) {
        uint16_t pl = (uint16_t)(10 + prng() % 300);
        uint16_t l = make_frame(f, pl, (uint32_t)i);
        ntrip_ring_push(&ring, f, l, (uint32_t)(1000 + i));

        /* random subset of instances polls, with randomized partial sends */
        for (int k = 0; k < 10; k++) {
            if (prng() % 3 == 0) continue;
            ntrip_tx_poll(&tx[k], &ring, (uint32_t)(1000 + i), 2, 0,
                          mock_send, &m[k]);
        }
    }

    /* final drain: everyone catches up (accept-all) */
    for (int k = 0; k < 10; k++) {
        m[k].random_mode = 0;
        m[k].n_acts = m[k].next = 0;
        for (int spin = 0; spin < 1000; spin++) {
            if (ntrip_tx_poll(&tx[k], &ring, 2000, 4, 0, mock_send, &m[k]) ==
                NTRIP_TX_POLL_IDLE) break;
        }
        /* per-instance byte balance */
        CHECK_EQ_U64(tx[k].copied_bytes,
                     tx[k].accepted_to_lwip + ntrip_tx_pending(&tx[k]) +
                     tx[k].dropped_bytes);
        /* everything pushed is either copied or skipped (stale disabled) */
        CHECK_EQ_U64(tx[k].copied_bytes + tx[k].skipped_bytes, ring.cum_bytes);
        CHECK_EQ_U64(tx[k].accepted_to_lwip, m[k].accepted);
    }
    CHECK(g_locks > 0 && g_locks == g_unlocks);
}

static void test_property_byte_balance(void) {
    enum { STREAM_MAX = 262144 };
    static uint8_t stream[STREAM_MAX + 4096];
    static uint8_t f[RTCM_MAX_FRAME_B];
    size_t n = 0;
    uint64_t n_valid = 0;

    g_rng = 0xDEADBEEFu;

    while (n < STREAM_MAX) {
        uint32_t r = prng() % 100u;
        if (r < 55) {
            /* a valid frame */
            uint16_t pl = (uint16_t)(1 + prng() % 300);
            if (prng() % 50 == 0) pl = 1023;
            uint16_t l = make_frame(f, pl, prng());
            memcpy(stream + n, f, l);
            n += l;
            n_valid++;
        } else if (r < 80) {
            /* garbage, 0xD3 explicitly allowed */
            uint32_t g = 1 + prng() % 40;
            for (uint32_t i = 0; i < g; i++) stream[n++] = (uint8_t)prng();
        } else {
            /* a corrupted frame: flip one byte after CRC computation */
            uint16_t pl = (uint16_t)(1 + prng() % 200);
            uint16_t l = make_frame(f, pl, prng());
            uint16_t at = (uint16_t)(prng() % l);
            f[at] ^= (uint8_t)(1u << (prng() % 8));
            memcpy(stream + n, f, l);
            n += l;
        }
    }
    /* flush tail so no valid frame stays buffered awaiting a false `need` */
    memset(stream + n, 0, 2100);
    size_t total = n + 2100;

    rtcm_parser_t p;
    counter_t cnt;
    memset(&cnt, 0, sizeof(cnt));
    rtcm_parser_init(&p);

    size_t i = 0;
    while (i < total) {
        size_t chunk = 1 + prng() % 700;
        if (chunk > total - i) chunk = total - i;
        rtcm_parser_feed(&p, stream + i, chunk, 0, count_cb, &cnt);
        i += chunk;
    }

    /* the invariants that matter */
    CHECK_EQ_U64(p.bytes_ok + p.bytes_discarded + p.have, total);
    CHECK_EQ_U64(cnt.count, n_valid);
    CHECK_EQ_U64(cnt.bytes, p.bytes_ok);
    CHECK_EQ_U64(cnt.crc_bad, 0);
    CHECK(p.crc_errors > 0);          /* the corrupted frames were seen */
    printf("  property: %llu valid frames, %llu crc errors, %llu discarded bytes\n",
           (unsigned long long)n_valid, (unsigned long long)p.crc_errors,
           (unsigned long long)p.bytes_discarded);
}

static void test_progress_timeout_wraparound(void) {
    ntrip_tx_t tx;
    ntrip_tx_init(&tx);
    tx.current_len = 100;
    tx.current_off = 10;
    tx.last_progress_ms = 0xFFFFF000u;      /* close to uint32 wrap */
    CHECK(!ntrip_tx_stalled(&tx, 0xFFFFF800u, 5000));  /* 2048ms elapsed */
    CHECK(ntrip_tx_stalled(&tx, 0x00001000u, 5000));   /* 8192ms, wrapped */
}

int main(void) {
    test_crc_known_vector();
    test_all_chunk_splits();
    test_embedded_and_false_preambles();
    test_max_size_frame();
    test_ring_guards_and_locks();
    test_sender_short_then_eagain();
    test_sender_short_then_hard_error();
    test_reconnect_with_partial_frame();
    test_ring_overflow_fast_slow();
    test_stale_frame_drop();
    test_ten_instances();
    test_property_byte_balance();
    test_progress_timeout_wraparound();

    printf("%d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
