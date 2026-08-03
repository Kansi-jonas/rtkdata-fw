/* Host tests for telemetry_json: the all-or-nothing contract and the
 * documented worst-case bound.
 * gcc -Imain/include test_telemetry_json.c ../../main/telemetry_json.c */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "telemetry_json.h"

static int g_checks = 0;
static int g_fails = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        g_fails++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

static void test_known_values(void) {
    ntrip_tx_totals_t t;
    memset(&t, 0, sizeof(t));
    t.frames_ok = 7;
    t.bytes_ok = 1234;
    t.crc_errors = 1;
    t.accepted_to_lwip = 1200;
    t.sent_frames = 6;
    t.eagain_count = 2;
    t.reconnects = 1;
    t.max_queue_age_ms = 250;
    t.instances = 1;
    t.connected = 1;
    t.req_fresh = true;
    t.req_missing = 0;

    char buf[TELEMETRY_TX_JSON_MAX];
    int n = telemetry_tx_json(buf, sizeof(buf), &t, 3);
    CHECK(n > 0);
    CHECK((int)strlen(buf) == n);

    const char *expect =
        "\"tx\":{\"frames_ok\":7,\"bytes_ok\":1234,\"bytes_discarded\":0,"
        "\"crc_errors\":1,\"accepted_to_lwip\":1200,\"sent_frames\":6,"
        "\"dropped_bytes\":0,\"skipped_bytes\":0,\"dropped_stale_bytes\":0,"
        "\"eagain\":2,\"reconnects\":1,\"max_queue_age_ms\":250,"
        "\"instances\":1,\"connected\":1,\"uart_event_post_drops\":3,"
        "\"req_fresh\":true,\"req_missing\":0}";
    CHECK(strcmp(buf, expect) == 0);
}

static void test_worst_case_bound(void) {
    ntrip_tx_totals_t t;
    memset(&t, 0xFF, sizeof(t));   /* every counter at its maximum */
    t.req_fresh = true;            /* 0xFF is not a valid _Bool (UBSan) */

    char buf[TELEMETRY_TX_JSON_MAX];
    int n = telemetry_tx_json(buf, sizeof(buf), &t, 0xFFFFFFFFul);
    CHECK(n > 0);
    CHECK(n < TELEMETRY_TX_JSON_MAX);

    /* well-formed member: prefix, balanced braces, terminated */
    CHECK(strncmp(buf, "\"tx\":{", 6) == 0);
    CHECK(buf[n - 1] == '}');
    int braces = 0;
    for (int i = 0; i < n; i++) {
        if (buf[i] == '{') braces++;
        if (buf[i] == '}') braces--;
    }
    CHECK(braces == 0);
}

static void test_all_or_nothing(void) {
    ntrip_tx_totals_t t;
    memset(&t, 0, sizeof(t));

    char probe[TELEMETRY_TX_JSON_MAX];
    int full = telemetry_tx_json(probe, sizeof(probe), &t, 0);
    CHECK(full > 0);

    /* cap exactly one too small: -1 and the buffer stays untouched */
    char buf[TELEMETRY_TX_JSON_MAX];
    memset(buf, 0xAA, sizeof(buf));
    CHECK(telemetry_tx_json(buf, (size_t)full, &t, 0) == -1);
    int untouched = 1;
    for (size_t i = 0; i < sizeof(buf); i++) {
        if ((unsigned char)buf[i] != 0xAA) untouched = 0;
    }
    CHECK(untouched);

    /* cap exactly big enough: success */
    CHECK(telemetry_tx_json(buf, (size_t)full + 1, &t, 0) == full);

    /* NULL contracts */
    CHECK(telemetry_tx_json(NULL, 64, &t, 0) == -1);
    CHECK(telemetry_tx_json(buf, sizeof(buf), NULL, 0) == -1);
}

int main(void) {
    test_known_values();
    test_worst_case_bound();
    test_all_or_nothing();
    printf("%d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
