"""Regress the production UART/receiver-epoch observation adapter on a host.

Production function bodies and the frame context are extracted unchanged.
Only RTOS, clock and notification services are replaced. One parser-call hook
places a receiver reset after the UART chunk captured its epoch but before its
callback runs, making that race deterministic without threads or hardware.
"""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

from test_station_health import function

ROOT = Path(__file__).resolve().parents[2]

PRELUDE = r'''
#include "rtcm_observation_fixture.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

typedef uint32_t EventBits_t;
typedef struct { void *ev; void *task_server; } ntrip_instance_t;
static rtcm_parser_t g_rtcm_parser;
static ntrip_ring_t g_frame_ring;
static int g_ring_mutex = 1, g_instances_mutex;
static size_t g_instance_count;
static ntrip_instance_t *g_instances[1];
static uint32_t clock_ms, site_generation = 1;
static bool upload_allowed = true, reset_during_parse;
static unsigned liveness_notes, checks;
#define NTRIP_REQUIRED_MSG_COUNT 4
static const uint16_t ntrip_required_msgs[NTRIP_REQUIRED_MSG_COUNT] = {1005,1033,1077,1087};
static uint32_t s_msg_last_ms[NTRIP_REQUIRED_MSG_COUNT];
static uint32_t s_msg_epoch[NTRIP_REQUIRED_MSG_COUNT];
static volatile uint32_t s_epoch = 1, s_epoch_started_ms;
static volatile bool s_ingest_reset_req;
static rtcm_observation_monitor_t s_observations;
static uint32_t s_observation_last_ms, s_observation_epoch;

static uint32_t ntrip_now_ms(void) { return clock_ms; }
static bool gnss_site_upload_allowed(void) { return upload_allowed; }
static uint32_t gnss_site_upload_generation(void) { return site_generation; }
static void supervisor_note_gnss_rx(void) { ++liveness_notes; }
#define portMAX_DELAY 0
static void xSemaphoreTake(int mutex, int delay) { (void)mutex; (void)delay; }
static void xSemaphoreGive(int mutex) { (void)mutex; }
static EventBits_t xEventGroupGetBits(void *event) { (void)event; return 0; }
static bool ntrip_server_update_data_state(ntrip_instance_t *inst, EventBits_t bits) {
    (void)inst; (void)bits; return false;
}
static void xTaskNotifyGive(void *task) { (void)task; }
#define CHECK(c) do { ++checks; if (!(c)) { \
    printf("FAIL %d: %s\n", __LINE__, #c); return 1; } } while (0)
'''

PARSE_HOOK = r'''
static void review_parser_feed(rtcm_parser_t *parser, const uint8_t *bytes, size_t len,
                               uint32_t now, rtcm_frame_cb callback, void *ctx) {
    if (reset_during_parse) {
        reset_during_parse = false;
        ntrip_server_msg_epoch_reset();
    }
    rtcm_parser_feed(parser, bytes, len, now, callback, ctx);
}
#define rtcm_parser_feed review_parser_feed
'''

TESTS = r'''
#undef rtcm_parser_feed
int main(void) {
    uint8_t frame[128];
    rtcm_parser_init(&g_rtcm_parser);
    ntrip_ring_init(&g_frame_ring, NULL, NULL, NULL);
    rtcm_observation_init(&s_observations);
    CHECK(ntrip_server_observations_silent_ms() == UINT32_MAX);

    size_t len = observation_frame(frame, 1077, 1000, OBS_VALID);
    clock_ms = 1000;
    ntrip_server_ingest_uart(frame, len);
    CHECK(ntrip_server_observations_silent_ms() == 0);
    CHECK(g_frame_ring.next_seq == 1 && liveness_notes == 1);

    // A healthy receiver epoch may run for more than 2^31 milliseconds.
    // signed(last-epoch_started) wrongly rejects this real current sample.
    len = observation_frame(frame, 1077, 2000, OBS_VALID);
    clock_ms = UINT32_C(0x80000010);
    ntrip_server_ingest_uart(frame, len);
    clock_ms++;
    CHECK(ntrip_server_observations_silent_ms() == 1);
    CHECK(g_frame_ring.next_seq == 2);

    // A uint32 monotonic clock wraps after49.7days. Comparing frame age with
    // unsigned epoch age would also reject this healthy sample across wrap.
    len = observation_frame(frame, 1077, 3000, OBS_VALID);
    clock_ms = UINT32_MAX - 100;
    ntrip_server_ingest_uart(frame, len);
    clock_ms = 50;
    CHECK(ntrip_server_observations_silent_ms() == 151);
    CHECK(g_frame_ring.next_seq == 3);

    // Reset after production ingest captured its context, before the actual
    // parser invokes production on_frame: this old chunk must stay old.
    uint32_t prior_epoch = s_epoch;
    len = observation_frame(frame, 1077, 4000, OBS_VALID);
    clock_ms = 60;
    reset_during_parse = true;
    ntrip_server_ingest_uart(frame, len);
    CHECK(s_epoch != prior_epoch);
    CHECK(s_observation_epoch == prior_epoch);
    CHECK(ntrip_server_observations_silent_ms() == UINT32_MAX);

    // The following chunk resets parser + per-family epoch history. A fresh
    // receiver can legitimately restart at an earlier observation epoch.
    len = observation_frame(frame, 1077, 1000, OBS_VALID);
    clock_ms = 70;
    ntrip_server_ingest_uart(frame, len);
    CHECK(s_observation_epoch == s_epoch);
    CHECK(ntrip_server_observations_silent_ms() == 0);
    CHECK(!s_ingest_reset_req);

    // Neither the tail of a pre-reset partial frame nor metadata can become
    // evidence for the new receiver epoch.
    len = observation_frame(frame, 1077, 2000, OBS_VALID);
    clock_ms = 80;
    ntrip_server_ingest_uart(frame, len / 2);
    CHECK(g_rtcm_parser.have > 0);
    uint32_t before = g_frame_ring.next_seq;
    clock_ms = 90;
    ntrip_server_msg_epoch_reset();
    ntrip_server_ingest_uart(frame + len / 2, len - len / 2);
    CHECK(g_frame_ring.next_seq == before);
    CHECK(ntrip_server_observations_silent_ms() == UINT32_MAX);
    len = observation_frame(frame, 1005, 2000, OBS_VALID);
    clock_ms = 100;
    ntrip_server_ingest_uart(frame, len);
    CHECK(g_frame_ring.next_seq == before + 1);
    CHECK(ntrip_server_observations_silent_ms() == UINT32_MAX);
    len = observation_frame(frame, 1077, 2000, OBS_VALID);
    clock_ms = 110;
    ntrip_server_ingest_uart(frame, len);
    CHECK(ntrip_server_observations_silent_ms() == 0);

    // Model the reset writer after incrementing its epoch but before setting
    // the reset-request flag. Epoch identity alone must drop the old partial.
    len = observation_frame(frame, 1077, 3000, OBS_VALID);
    clock_ms = 120;
    ntrip_server_ingest_uart(frame, len / 2);
    CHECK(g_rtcm_parser.have > 0);
    before = g_frame_ring.next_seq;
    ++s_epoch;
    s_ingest_reset_req = false;
    clock_ms = 130;
    ntrip_server_ingest_uart(frame + len / 2, len - len / 2);
    CHECK(g_frame_ring.next_seq == before);
    CHECK(ntrip_server_observations_silent_ms() == UINT32_MAX);
    clock_ms = 140;
    ntrip_server_ingest_uart(frame, len);
    CHECK(g_frame_ring.next_seq == before + 1);
    CHECK(ntrip_server_observations_silent_ms() == 0);

    // Upload holds prevent enqueue, but independent receiver observations
    // remain observable; the site runtime decides whether output is expected.
    upload_allowed = false;
    len = observation_frame(frame, 1077, 4000, OBS_VALID);
    before = g_frame_ring.next_seq;
    clock_ms = 150;
    ntrip_server_ingest_uart(frame, len);
    CHECK(g_frame_ring.next_seq == before);
    CHECK(ntrip_server_observations_silent_ms() == 0);
    frame[len - 1] ^= 1;
    clock_ms = 151;
    ntrip_server_ingest_uart(frame, len);
    CHECK(ntrip_server_observations_silent_ms() == 1);

    // Counter wrap cannot accidentally revive an old receiver's evidence.
    s_epoch = UINT32_MAX;
    len = observation_frame(frame, 1077, 1000, OBS_VALID);
    clock_ms = 160;
    ntrip_server_ingest_uart(frame, len);
    CHECK(s_observation_epoch == UINT32_MAX);
    clock_ms = 170;
    ntrip_server_msg_epoch_reset();
    CHECK(s_epoch == 1 && ntrip_server_observations_silent_ms() == UINT32_MAX);
    clock_ms = 180;
    ntrip_server_ingest_uart(frame, len);
    CHECK(s_observation_epoch == 1 && ntrip_server_observations_silent_ms() == 0);
    printf("%u observation adapter checks, 0 failures\n", checks);
    return 0;
}
'''


def main():
    compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not compiler:
        raise SystemExit("Host C compiler required")
    server = (ROOT / "main/interface/ntrip_server.c").read_text(encoding="utf-8")
    context = re.search(
        r"typedef struct \{\s*uint32_t site_generation;\s*uint32_t receiver_epoch;\s*\} ntrip_ingest_context_t;",
        server,
    )
    if not context:
        raise AssertionError("Production receiver-epoch UART context not found")
    bodies = [function(server, name) for name in (
        "ntrip_server_msg_epoch_reset", "ntrip_site_ring_guard",
        "ntrip_server_observations_silent_ms", "ntrip_server_on_frame",
    )]
    source_text = (PRELUDE + context.group() + "\n" + "\n".join(bodies)
                   + PARSE_HOOK + function(server, "ntrip_server_ingest_uart") + TESTS)
    with tempfile.TemporaryDirectory(prefix="rtkdata-observation-adapter-") as folder:
        source = Path(folder) / "test.c"
        binary = Path(folder) / ("test.exe" if os.name == "nt" else "test")
        source.write_text(source_text, encoding="utf-8")
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        f"-I{ROOT / 'main/include'}", f"-I{ROOT / 'tests/host'}",
                        str(source), str(ROOT / "main/interface/ntrip_tx_core.c"),
                        "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
