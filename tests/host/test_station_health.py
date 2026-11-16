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
static bool s_survey_acked;
static uint32_t s_survey_rev;
static gnss_base_fix_t s_base_acked_coord, s_base_fail_coord;
static int64_t s_base_fail_us, clock_us = 1000000;
static bool s_gnss_seen, s_caster_seen, s_caster_progress, s_ip_seen, s_link_up;
static volatile uint32_t s_t_gnss, s_t_caster, s_t_ip;
static uint32_t s_rec_gnss, s_rec_caster, s_rec_wifi, s_supervised_reboots;
static bool zero_ok, mode_ok, save_ok;
#ifndef CONFIG_RTK_SITE_LEGACY
static bool has_persisted;
#endif
static gnss_base_fix_t persisted;
static int zero_calls, save_calls, nvs_writes, lock_depth, checks;
#ifndef CONFIG_RTK_SITE_LEGACY
static int mode_calls;
#endif
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
static int configured_calls;
static gnss_site_mode_t site_mode = GNSS_SITE_UNKNOWN, configured_mode;
static bool configured_applied;
void gnss_site_configured(gnss_site_mode_t m, double la, double lo, uint32_t rev, bool ok) {
    (void)la; (void)lo; (void)rev;
    configured_calls++; configured_mode = m; configured_applied = ok;
    site_mode = ok ? m : GNSS_SITE_UNKNOWN;
}
#ifdef CONFIG_RTK_SITE_LEGACY
/* Legacy relocation guard: the runtime half is covered by
 * test_gnss_site_runtime.py "relocate"; here its verdicts drive gnss.c. */
#define GNSS_NVS_NS "gnss"
#define GNSS_KEY_BASE "base"
typedef int nvs_handle_t;
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NVS_NOT_FOUND 0x1102
#define NVS_READWRITE 1
static uint32_t s_legacy_moved_mm;
static bool s_drop_erase_pending, s_drop_mode_pending;
static gnss_site_agreement_t agreement = GNSS_SITE_AGREEMENT_AGREE;
static uint32_t agreement_mm, relocated_mm;
static bool relocated, erase_ok = true;
static int erase_calls, mode_calls_at_erase = -1;
gnss_site_agreement_t gnss_site_legacy_agreement(double la, double lo, uint32_t *mm) {
    (void)la; (void)lo;
    if (mm) *mm = agreement_mm;
    return agreement;
}
bool gnss_site_legacy_relocated(uint32_t *mm) {
    if (mm) *mm = relocated ? relocated_mm : 0;
    return relocated;
}
void gnss_site_status(gnss_site_status_t *o) { memset(o, 0, sizeof(*o)); o->mode = site_mode; }
static esp_err_t nvs_open(const char *ns, int mode, nvs_handle_t *h) {
    (void)ns; (void)mode; *h = 1; return ESP_OK;
}
static int mode_calls;
static bool has_persisted;
static esp_err_t nvs_erase_key(nvs_handle_t h, const char *key) {
    (void)h; (void)key; erase_calls++; mode_calls_at_erase = mode_calls;
    if (!erase_ok) return ESP_FAIL;
    if (!has_persisted) return ESP_ERR_NVS_NOT_FOUND;
    has_persisted = false;
    return ESP_OK;
}
static esp_err_t nvs_commit(nvs_handle_t h) { (void)h; return ESP_OK; }
static void nvs_close(nvs_handle_t h) { (void)h; }
#endif
bool gnss_site_desired_valid(const gnss_site_desired_t *d) { return d != NULL; }
static int epoch_resets;
void ntrip_server_msg_epoch_reset(void) { epoch_resets++; }
static bool s_survey_mode_acked_now;
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
    // MCU NVS already holds the coordinate the receiver runs (review 13.11.
    // L1, R5: written right after the MODE ACK, before SAVECONFIG).
    reset_base(); save_ok = false;
    CHECK(!gnss_set_fixed_base(52, 13, 42));
    CHECK(s_base_acked_valid && nvs_writes == 1);
    CHECK(has_persisted && persisted.lat == 52 && persisted.h == 42);
    CHECK(!gnss_set_fixed_base(52, 13, 42) && mode_calls == 1);
    save_ok = true;
    CHECK(gnss_set_fixed_base(52, 13, 42));
    CHECK(mode_calls == 1 && nvs_writes == 1 && lock_depth == 0);

    // A survey whose saveconfig failed is running anyway: the identical
    // request retries SAVE alone. Re-sending MODE restarted the 300 s survey
    // on every heartbeat while SAVE kept failing (review 2026-11-06, G2).
    {
        reset_base(); s_survey_acked = false; save_ok = false;
        gnss_site_desired_t sv = { .mode = GNSS_SITE_SURVEY, .revision = 5 };
        CHECK(!gnss_apply_site_revision(&sv) && mode_calls == 1 && save_calls == 1);
        CHECK(s_survey_acked && s_survey_rev == 5);
        CHECK(!gnss_apply_site_revision(&sv) && mode_calls == 1 && save_calls == 2);
        save_ok = true;
        CHECK(gnss_apply_site_revision(&sv) && mode_calls == 1 && save_calls == 3);
        CHECK(gnss_apply_site_revision(&sv) && mode_calls == 1);   // saved: SAVE only
        sv.revision = 6;                                         // a NEW survey request
        CHECK(gnss_apply_site_revision(&sv) && mode_calls == 2 && s_survey_rev == 6);
        // a fixed MODE ends the survey: the same survey revision applies MODE again
        CHECK(gnss_set_fixed_base(52, 13, 42) && !s_survey_acked);
        CHECK(gnss_apply_site_revision(&sv) && mode_calls == 4);
        // a rejected survey MODE latches nothing
        reset_base(); s_survey_acked = false; mode_ok = false; sv.revision = 7;
        CHECK(!gnss_apply_site_revision(&sv) && !s_survey_acked && save_calls == 0);
        CHECK(lock_depth == 0);
    }

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

    // ip_down counts the OUTAGE, not the time since the lease was acquired.
    // Before, a blip after a minute online already exceeded TH_LINK_S and
    // restarted the Wi-Fi driver at once (bench 2026-11-06).
    supervisor_note_sta_ip(true);
    clock_us += 120 * 1000000LL;
    supervisor_note_sta_ip(false); supervisor_health(&h);
    CHECK(!h.link_ok && h.ip_down_s == 0);
    clock_us += 61 * 1000000LL;
    supervisor_note_sta_ip(false);            // repeated "down" keeps the clock
    supervisor_health(&h);
    CHECK(!h.link_ok && h.ip_down_s == 61);
    supervisor_note_sta_ip(true); supervisor_health(&h);
    CHECK(h.link_ok && h.ip_down_s == 0);

    // Ages are 32-bit millisecond deltas: correct across the 2^32 ms wrap
    // (49.7 days), and no 64-bit timestamp can tear between the two cores.
    clock_us = (int64_t)UINT32_MAX * 1000 - 2000000;   // 2 s before the wrap
    supervisor_note_gnss_rx();
    clock_us += 5 * 1000000LL;                         // 3 s past the wrap
    supervisor_health(&h);
    CHECK(h.gnss_silent_s == 5);

#ifdef CONFIG_RTK_SITE_LEGACY
    // Review 13.11. L1 R1: a new IE coordinate the receiver's own position
    // disagrees with is refused before any UART or NVS work.
    reset_base(); agreement = GNSS_SITE_AGREEMENT_DISAGREE; agreement_mm = 120000;
    CHECK(!gnss_set_fixed_base(52, 13, 42));
    CHECK(zero_calls == 0 && mode_calls == 0 && nvs_writes == 0 && lock_depth == 0);
    CHECK(gnss_legacy_moved_m() == 120 && !s_base_acked_valid);
    agreement = GNSS_SITE_AGREEMENT_UNKNOWN;          // no own position yet: wait
    CHECK(!gnss_set_fixed_base(52, 13, 42) && mode_calls == 0 && nvs_writes == 0);
    agreement = GNSS_SITE_AGREEMENT_AGREE;            // in place: applied, persisted
    CHECK(gnss_set_fixed_base(52, 13, 42));
    CHECK(mode_calls == 1 && nvs_writes == 1 && s_base_acked_valid && gnss_legacy_moved_m() == 0);
    CHECK(configured_mode == GNSS_SITE_FIXED && configured_applied);
    // The coordinate the receiver already runs is not re-judged per reply;
    // leaving it is the tick's job (below).
    agreement = GNSS_SITE_AGREEMENT_DISAGREE;
    CHECK(gnss_set_fixed_base(52, 13, 42) && mode_calls == 1);

    // Moved for 5 min: forget the coordinate FIRST, then survey here.
    reset_base(); has_persisted = true; s_base_acked_valid = true;
    site_mode = GNSS_SITE_FIXED; relocated = true; relocated_mm = 1100000;
    supervisor_note_gnss_rx();
    gnss_legacy_tick();
    CHECK(erase_calls == 1 && !has_persisted && mode_calls_at_erase == 0);
    CHECK(mode_calls == 1 && save_calls == 1 && !s_base_acked_valid);
    CHECK(configured_mode == GNSS_SITE_SURVEY && configured_applied && s_survey_acked);
    CHECK(gnss_legacy_moved_m() == 1100 && lock_depth == 0 && !s_capturing);
    relocated = false;                                 // the survey ended the run
    gnss_legacy_tick();
    CHECK(mode_calls == 1);                            // nothing more to do

    // A NACKed survey MODE is retried every 60 s without a new run.
    reset_base(); has_persisted = true; erase_calls = 0;
    site_mode = GNSS_SITE_FIXED; relocated = true; mode_ok = false;
    gnss_legacy_tick();
    CHECK(mode_calls == 1 && site_mode == GNSS_SITE_UNKNOWN && !has_persisted);
    relocated = false;
    clock_us += 30 * 1000000LL; supervisor_note_gnss_rx();
    gnss_legacy_tick();
    CHECK(mode_calls == 1);                            // cooldown
    clock_us += 31 * 1000000LL; supervisor_note_gnss_rx(); mode_ok = true;
    gnss_legacy_tick();
    // only the unfinished half: the erase had worked the first time
    CHECK(mode_calls == 2 && site_mode == GNSS_SITE_SURVEY && erase_calls == 1);
    clock_us += 61 * 1000000LL; supervisor_note_gnss_rx();
    gnss_legacy_tick();
    CHECK(mode_calls == 2);                            // done, no further retry

    // Review 13.11. L2 #3: an erase that keeps failing is retried alone; the
    // running survey is not restarted every minute.
    reset_base(); has_persisted = true; erase_calls = 0; erase_ok = false;
    site_mode = GNSS_SITE_FIXED; relocated = true;
    gnss_legacy_tick();
    CHECK(erase_calls == 1 && mode_calls == 1 && site_mode == GNSS_SITE_SURVEY);
    relocated = false;
    for (int i = 0; i < 3; ++i) {
        clock_us += 61 * 1000000LL; supervisor_note_gnss_rx(); gnss_legacy_tick();
    }
    CHECK(erase_calls == 4 && mode_calls == 1 && has_persisted);
    erase_ok = true;
    clock_us += 61 * 1000000LL; supervisor_note_gnss_rx(); gnss_legacy_tick();
    CHECK(erase_calls == 5 && !has_persisted && mode_calls == 1);
    clock_us += 61 * 1000000LL; supervisor_note_gnss_rx(); gnss_legacy_tick();
    CHECK(erase_calls == 5);                           // finished

    // ... and a correct coordinate accepted in between cancels the drop: the
    // pending erase must not delete the new coordinate.
    reset_base(); has_persisted = true; erase_calls = 0; erase_ok = false;
    site_mode = GNSS_SITE_FIXED; relocated = true;
    gnss_legacy_tick();
    relocated = false; erase_ok = true; agreement = GNSS_SITE_AGREEMENT_AGREE;
    CHECK(gnss_set_fixed_base(49.5, 7.1, 300));        // the IE's re-survey, in place
    CHECK(has_persisted && persisted.lat == 49.5 && site_mode == GNSS_SITE_FIXED);
    int modes = mode_calls;
    clock_us += 61 * 1000000LL; supervisor_note_gnss_rx(); gnss_legacy_tick();
    CHECK(erase_calls == 1 && has_persisted && mode_calls == modes);

    // L3 #1: while the erase of the left coordinate is pending, a receiver
    // recovery (or the reset button) surveys instead of restoring it.
    reset_base(); has_persisted = true; persisted = (gnss_base_fix_t){52, 13, 42};
    s_drop_erase_pending = true;
    config_gnss_base();
    CHECK(configured_mode == GNSS_SITE_SURVEY && configured_applied && !s_base_acked_valid);
    s_drop_erase_pending = false;
    config_gnss_base();
    CHECK(configured_mode == GNSS_SITE_FIXED && configured_applied && s_base_acked_valid);

    // A survey that left its own 1005 (L2 #5): nothing in NVS to forget,
    // the survey restarts here.
    reset_base(); erase_calls = 0; site_mode = GNSS_SITE_SURVEY; relocated = true;
    gnss_legacy_tick();
    CHECK(erase_calls == 1 && mode_calls == 1 && configured_mode == GNSS_SITE_SURVEY);
    relocated = false;
    clock_us += 61 * 1000000LL; supervisor_note_gnss_rx(); gnss_legacy_tick();
    CHECK(mode_calls == 1);

    // Review 13.11. L1 R4: mode UNKNOWN with a live receiver is reconfigured
    // after 5 min, then backs off.
    reset_base(); reset_calls = 0; site_mode = GNSS_SITE_UNKNOWN; mode_ok = false;
    supervisor_note_gnss_rx(); gnss_legacy_tick();     // arms
    clock_us += 240 * 1000000LL; supervisor_note_gnss_rx(); gnss_legacy_tick();
    CHECK(reset_calls == 0);
    clock_us += 61 * 1000000LL; supervisor_note_gnss_rx(); gnss_legacy_tick();
    CHECK(reset_calls == 1 && zero_calls >= 1 && lock_depth == 0);
    supervisor_note_gnss_rx(); gnss_legacy_tick();     // re-arms, now 10 min
    clock_us += 301 * 1000000LL; supervisor_note_gnss_rx(); gnss_legacy_tick();
    CHECK(reset_calls == 1);
    clock_us += 300 * 1000000LL; supervisor_note_gnss_rx(); gnss_legacy_tick();
    CHECK(reset_calls == 2);
    // A known mode or a receiver the supervisor already owns: hands off.
    site_mode = GNSS_SITE_SURVEY; clock_us += 3600 * 1000000LL;
    supervisor_note_gnss_rx(); gnss_legacy_tick(); gnss_legacy_tick();
    CHECK(reset_calls == 2);
#endif
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
        "config_gnss_base", "gnss_set_fixed_base", "gnss_start_site_survey",
        "gnss_apply_site_revision", "gnss_recover",
    )]
    bodies += [function(supervisor, name) for name in (
        "now_ms", "age_s", "supervisor_note_gnss_rx", "supervisor_note_caster_tx",
        "supervisor_note_sta_ip", "gnss_observation_fault", "supervisor_health",
    )]
    with tempfile.TemporaryDirectory(prefix="rtkdata-station-health-") as folder:
        source = Path(folder) / "test.c"
        binary = Path(folder) / ("test.exe" if os.name == "nt" else "test")
        source.write_text(PRELUDE + "\n".join(bodies) + TESTS, encoding="utf-8")
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        f"-I{ROOT / 'main/include'}", str(source), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        # The legacy-IE firmware (Kconfig RTK_SITE_LEGACY): its relocation
        # guard and unknown-mode reconfiguration live in gnss.c.
        legacy_bodies = bodies + [function(gnss, name) for name in (
            "gnss_legacy_moved_m", "gnss_forget_fixed_base", "gnss_legacy_tick",
        )]
        legacy_source = Path(folder) / "legacy.c"
        legacy = Path(folder) / ("legacy.exe" if os.name == "nt" else "legacy")
        legacy_source.write_text(PRELUDE + "\n".join(legacy_bodies) + TESTS, encoding="utf-8")
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-DCONFIG_RTK_SITE_LEGACY=1",
                        f"-I{ROOT / 'main/include'}", str(legacy_source), "-o", str(legacy)], check=True)
        subprocess.run([str(legacy)], check=True)


if __name__ == "__main__":
    main()
