"""Fault-inject the actual firmware control functions without ESP-IDF/hardware.

The C function bodies are extracted unchanged from production sources; only
UART/NVS/time/RTOS services are replaced. Run: python tests/host/test_station_health.py
"""

import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]


def function(source, name):
    match = re.search(r"^[^\n;{}]*\b" + name + r"\([^;{}]*\)\s*\{", source, re.M)
    if match is None:
        raise AssertionError(f"Production function not found: {name}")
    start = source.index("{", match.start())
    depth = 0
    # Ignore braces in comments and string literals when locating the body.
    tokens = r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\x27(?:\\.|[^\x27\\])*\x27|[{}]'
    for token in re.finditer(tokens, source[start:]):
        if token.group() == "{":
            depth += 1
        elif token.group() == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():start + token.end()]
    raise AssertionError(f"Unterminated production function: {name}")


PRELUDE = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "supervisor.h"
#include "gnss_site.h"
#define TAG "test"
#define ACK_RETRIES 3
#define GNSS_BASE_RETRY_COOLDOWN_US (60LL * 1000 * 1000)
#define TH_GNSS_S 10
#define TH_CASTER_S 30
typedef struct { double lat, lon, h; } gnss_base_fix_t;
static bool s_capturing, s_base_acked_valid, s_base_saved;
static gnss_base_fix_t s_base_acked_coord, s_base_fail_coord;
static int64_t s_base_fail_us, clock_us = 1000000;
static bool s_gnss_seen, s_caster_seen, s_caster_progress, s_ip_seen, s_link_up;
static int64_t s_t_gnss, s_t_caster, s_t_ip;
static uint32_t s_rec_gnss, s_rec_caster, s_rec_wifi, s_supervised_reboots;
static bool zero_ok, mode_ok, save_ok, has_persisted;
static gnss_base_fix_t persisted;
static int zero_calls, mode_calls, save_calls, nvs_writes, lock_depth, checks;
static int reset_calls;
static bool expected_output;
static uint32_t observation_age = UINT32_MAX;
static void mock_log(const char *fmt, ...) { (void)fmt; }
#define ESP_LOGI(...) mock_log(__VA_ARGS__)
#define ESP_LOGW(...) mock_log(__VA_ARGS__)
#define ESP_LOGE(...) mock_log(__VA_ARGS__)
#define ESP_LOGD(...) mock_log(__VA_ARGS__)
#define uart_nmea(...) mock_log(__VA_ARGS__)
static void txn_take(void) { lock_depth++; }
static void txn_give(void) { lock_depth--; }
static int64_t esp_timer_get_time(void) { return clock_us; }
bool gnss_site_upload_allowed(void) { return true; }
bool gnss_site_needs_recovery(void) { return false; }
bool gnss_site_output_expected(void) { return expected_output; }
uint32_t ntrip_server_observations_silent_ms(void) { return observation_age; }
static void gnss_reset_pulse(void) { reset_calls++; }
bool gnss_site_get_desired(gnss_site_desired_t *d) { (void)d; return false; }
bool gnss_site_legacy_allowed(void) { return true; }
void gnss_site_configured(gnss_site_mode_t m, double la, double lo, uint32_t rev, bool ok) {
    (void)m; (void)la; (void)lo; (void)rev; (void)ok;
}
static bool send_cmd_acked(const char *cmd, int retries) {
    (void)retries;
    if (!strcmp(cmd, "CONFIG UNDULATION 0.0\r\n")) { zero_calls++; return zero_ok; }
    if (!strncmp(cmd, "mode base ", 10)) { mode_calls++; return mode_ok; }
    if (!strcmp(cmd, "saveconfig\r\n")) { save_calls++; return save_ok; }
    return true;
}
static bool gnss_load_fixed_base(double *lat, double *lon, double *h) {
    if (!has_persisted) return false;
    *lat = persisted.lat; *lon = persisted.lon; *h = persisted.h;
    return true;
}
static bool gnss_save_fixed_base(double lat, double lon, double h) {
    nvs_writes++; has_persisted = true;
    persisted = (gnss_base_fix_t){lat, lon, h}; return true;
}
static void reset_base(void) {
    s_base_acked_valid = s_base_saved = false; s_base_fail_us = 0; has_persisted = false;
    zero_ok = mode_ok = save_ok = true;
    zero_calls = mode_calls = save_calls = nvs_writes = lock_depth = 0;
    clock_us = 1000000;
}
#define CHECK(c) do { checks++; if (!(c)) { \
    printf("FAIL line %d: %s\n", __LINE__, #c); return 1; } } while (0)
'''

TESTS = r'''
int main(void) {
    // Failed geoid-mode setup must never be followed by a fixed-mode command,
    // a persisted coordinate, or the latch that skips subsequent correction.
    reset_base(); zero_ok = false;
    CHECK(!gnss_set_fixed_base(52, 13, 42));
    CHECK(zero_calls == 1 && mode_calls == 0 && save_calls == 0);
    CHECK(!s_base_acked_valid && nvs_writes == 0 && lock_depth == 0);
    CHECK(!gnss_set_fixed_base(52, 13, 42) && zero_calls == 1);
    clock_us += GNSS_BASE_RETRY_COOLDOWN_US; zero_ok = true;
    CHECK(gnss_set_fixed_base(52, 13, 42));
    CHECK(mode_calls == 1 && s_base_acked_valid && nvs_writes == 1);
    CHECK(gnss_set_fixed_base(52, 13, 42));
    CHECK(mode_calls == 1 && zero_calls == 2 && lock_depth == 0);

    // Boot restore has the same dependency even if an older latch exists.
    reset_base(); has_persisted = true; persisted = (gnss_base_fix_t){52, 13, 42};
    zero_ok = false; s_base_acked_valid = true;
    config_gnss_base();
    CHECK(zero_calls == 1 && mode_calls == 0 && !s_base_acked_valid);
    CHECK(!s_capturing && lock_depth == 0);
    zero_ok = true; config_gnss_base();
    CHECK(mode_calls == 1 && s_base_acked_valid && lock_depth == 0);
    CHECK(gnss_set_fixed_base(52, 13, 42) && mode_calls == 1);

    // MODE rejection cannot persist or latch the intended coordinate either.
    reset_base(); mode_ok = false;
    CHECK(!gnss_set_fixed_base(52, 13, 42));
    CHECK(!s_base_acked_valid && save_calls == 0 && nvs_writes == 0);
    CHECK(!s_capturing && lock_depth == 0);

    // A SAVE failure still leaves the running receiver correctly configured:
    // a later beat retries SAVE only, and cannot ACK until SAVE succeeds.
    reset_base(); save_ok = false;
    CHECK(!gnss_set_fixed_base(52, 13, 42));
    CHECK(s_base_acked_valid && nvs_writes == 0);
    CHECK(!gnss_set_fixed_base(52, 13, 42) && mode_calls == 1);
    save_ok = true;
    CHECK(gnss_set_fixed_base(52, 13, 42));
    CHECK(mode_calls == 1 && nvs_writes == 1 && lock_depth == 0);

    // Healthy before first upload was the production false-positive. A bare
    // accepted SOURCE handshake arms recovery but is not payload progress.
    supervisor_health_t h;
    supervisor_note_gnss_rx(); supervisor_note_sta_ip(true);
    supervisor_health(&h);
    CHECK(h.gnss_ok && h.link_ok && !h.caster_ok);
    CHECK(!h.obs_expected && h.obs_silent_s == -1);
    supervisor_note_caster_tx(0); supervisor_health(&h);
    CHECK(s_caster_seen && !h.caster_ok);
    supervisor_note_caster_tx(100); supervisor_health(&h);
    CHECK(h.caster_ok);
    // The callback revalidates after acquiring the receiver transaction lock.
    // A new intentional hold/warmup with live BASEPOS must cancel an old fault.
    supervisor_note_gnss_rx(); expected_output = false;
    gnss_recover();
    CHECK(reset_calls == 0 && lock_depth == 0);
    expected_output = true; observation_age = 10000;
    supervisor_health(&h);
    CHECK(!h.gnss_ok);
    CHECK(h.gnss_silent_s == 0 && h.obs_expected && h.obs_silent_s == 10);
    gnss_recover();
    CHECK(reset_calls == 1 && lock_depth == 0);
    observation_age = 0; gnss_recover();
    CHECK(reset_calls == 1 && lock_depth == 0);
    supervisor_health(&h);
    CHECK(h.gnss_ok && h.obs_expected && h.obs_silent_s == 0);
    clock_us += 30 * 1000000LL; supervisor_health(&h);
    CHECK(!h.caster_ok);
    supervisor_note_caster_tx(0); supervisor_health(&h);
    CHECK(!h.caster_ok);
    supervisor_note_caster_tx(100); supervisor_health(&h);
    CHECK(h.caster_ok);
    printf("%d station health fault-injection checks, 0 failures\n", checks);
    return 0;
}
'''


def main():
    compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not compiler:
        raise SystemExit("A host C compiler (gcc/clang or CC) is required")
    gnss = (ROOT / "main/gnss.c").read_text(encoding="utf-8")
    supervisor = (ROOT / "main/supervisor.c").read_text(encoding="utf-8")
    bodies = [function(gnss, name) for name in (
        "gnss_base_ack_invalidate", "gnss_coord_valid", "gnss_height_valid",
        "config_gnss_base", "gnss_set_fixed_base", "gnss_recover",
    )]
    bodies += [function(supervisor, name) for name in (
        "now_us", "age_s", "supervisor_note_gnss_rx", "supervisor_note_caster_tx",
        "supervisor_note_sta_ip", "gnss_observation_fault", "supervisor_health",
    )]
    with tempfile.TemporaryDirectory(prefix="rtkdata-station-health-") as folder:
        source = Path(folder) / "test.c"
        binary = Path(folder) / ("test.exe" if os.name == "nt" else "test")
        source.write_text(PRELUDE + "\n".join(bodies) + TESTS, encoding="utf-8")
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        f"-I{ROOT / 'main/include'}", str(source), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
