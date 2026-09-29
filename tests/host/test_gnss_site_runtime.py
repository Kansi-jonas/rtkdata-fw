"""Execute the actual site runtime with NVS/UART/clock fault injection.

Only platform includes are replaced. The production runtime body and BASEPOS
parser are compiled unchanged. No ESP-IDF install or hardware is needed.
"""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
from test_station_health import function

ROOT = Path(__file__).resolve().parents[2]
MOCKS = r'''
#include "gnss_site.h"
#include "rtcm_observation_fixture.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
typedef int portMUX_TYPE;
typedef int SemaphoreHandle_t;
typedef int nvs_handle_t;
typedef int esp_err_t;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portMAX_DELAY 0
#define ESP_OK 0
#define ESP_ERR_NVS_NOT_FOUND 2
#define NVS_READONLY 0
#define NVS_READWRITE 1
static unsigned critical, mutex_depth, flushes, gnss_calls, alive_calls, commits, checks;
static uint32_t now_ms, epoch_ms;
static rtcm_observation_monitor_t observation_monitor;
#define TH_GNSS_S 10
uint32_t ntrip_server_observations_silent_ms(void) {
    return rtcm_observation_silent_ms(&observation_monitor, now_ms);
}
static bool durable_exists, legacy_exists, pending_erase, commit_fail, gnss_ack = true;
static gnss_site_desired_t durable, pending;
static void mock_log(const char *tag, const char *fmt, ...) { (void)tag; (void)fmt; }
#define ESP_LOGE(...) mock_log(__VA_ARGS__)
#define portENTER_CRITICAL(m) do { (void)(m); critical++; } while (0)
#define portEXIT_CRITICAL(m) do { (void)(m); critical--; } while (0)
static int xSemaphoreCreateMutex(void) { return 1; }
static void xSemaphoreTake(int handle, int timeout) { (void)handle; (void)timeout; mutex_depth++; }
static void xSemaphoreGive(int handle) { (void)handle; mutex_depth--; }
static uint32_t esp_random(void) { return 123; }
static int64_t esp_timer_get_time(void) { return (int64_t)now_ms * 1000; }
static int nvs_open(const char *ns, int mode, int *handle) {
    (void)ns; (void)mode; *handle = 1; return ESP_OK;
}
static int nvs_get_blob(int handle, const char *key, void *dst, size_t *size) {
    (void)handle; (void)key;
    if (!durable_exists) return ESP_ERR_NVS_NOT_FOUND;
    if (*size < sizeof(durable)) return 3;
    *size = sizeof(durable); memcpy(dst, &durable, sizeof(durable)); return ESP_OK;
}
static int nvs_set_blob(int handle, const char *key, const void *data, size_t size) {
    (void)handle; (void)key;
    if (size != sizeof(pending) || critical) abort();
    memcpy(&pending, data, size); return ESP_OK;
}
static int nvs_erase_key(int handle, const char *key) {
    (void)handle;
    if (strcmp(key, "base")) abort();
    pending_erase = true; return legacy_exists ? ESP_OK : ESP_ERR_NVS_NOT_FOUND;
}
static int nvs_commit(int handle) {
    (void)handle; commits++;
    if (commit_fail) return 4;
    durable = pending; durable_exists = true;
    if (pending_erase) legacy_exists = false;
    pending_erase = false; return ESP_OK;
}
static void nvs_close(int handle) { (void)handle; }
void ntrip_server_site_flush(void) { if (critical) abort(); flushes++; }
void supervisor_note_gnss_rx(void) { if (critical) abort(); alive_calls++; }
bool gnss_apply_site_revision(const gnss_site_desired_t *d) {
    if (critical || gnss_site_upload_allowed() || !durable_exists ||
        durable.revision != d->revision || durable.mode != d->mode ||
        (d->mode == GNSS_SITE_SURVEY && legacy_exists)) abort();
    gnss_calls++;
    gnss_site_configured((gnss_site_mode_t)d->mode, d->lat_e9 / 1e9,
                         d->lon_e9 / 1e9, d->revision, gnss_ack);
    return gnss_ack;
}
#define CHECK(c) do { checks++; if (!(c)) { \
    printf("FAIL line %d: %s\n", __LINE__, #c); return 1; } } while (0)
'''

TESTS = r'''
static void sample(double lat, double lon, bool quality) {
    char line[GNSS_SITE_LINE_MAX];
    now_ms += 1000; epoch_ms += 1000;
    int n = snprintf(line, sizeof(line),
        "#BASEPOSA,96,GPS,FINE,2440,%u,0,0,18,20;"
        "SOL_COMPUTED,%s,%.11f,%.11f,40,2,WGS84,1,1,2,"
        "\"0\",0,0,30,12,28,0,16,12,01,51,"
        "SOL_COMPUTED,DOPPLER_VELOCITY,0,0,0,0,0,0,0",
        epoch_ms, quality ? "SINGLE" : "FIXEDPOS", lat, lon);
    uint32_t crc = gnss_site_crc32((const uint8_t *)line+1, (size_t)n-1);
    n += snprintf(line+n, sizeof(line)-(size_t)n, "*%08x\r\n", crc);
    gnss_site_runtime_ingest((const uint8_t *)line, (size_t)n);
}
static void stable(double lat, double lon) {
    for (unsigned i = 0; i < 65; ++i) sample(lat, lon, true);
}
static gnss_site_control_t survey(uint32_t revision, bool allow) {
    return (gnss_site_control_t){.boot_id=123, .revision=revision,
        .mode=GNSS_SITE_SURVEY, .allow_upload=allow};
}
static gnss_site_control_t fixed(uint32_t revision, bool allow) {
    return (gnss_site_control_t){.boot_id=123, .revision=revision,
        .mode=GNSS_SITE_FIXED, .allow_upload=allow,
        .lat_e9=INT64_C(52000000000), .lon_e9=INT64_C(13000000000), .h_mm=42000};
}
int main(int argc, char **argv) {
    const char *scenario = argc > 1 ? argv[1] : "basic";
    if (!strcmp(scenario, "corrupt")) {
        durable_exists = legacy_exists = true;
        durable = (gnss_site_desired_t){.magic=1,.revision=100,.mode=GNSS_SITE_FIXED};
        gnss_site_runtime_init();
        CHECK(!gnss_site_legacy_allowed());
        stable(52,13); gnss_site_control_t c = survey(101,true);
        CHECK(!gnss_site_apply_control(&c));
        CHECK(!gnss_site_upload_allowed() && gnss_calls == 0);
    } else if (!strcmp(scenario, "reboot")) {
        durable_exists = legacy_exists = true;
        durable = (gnss_site_desired_t){.magic=GNSS_SITE_NVS_MAGIC,
            .revision=7,.mode=GNSS_SITE_SURVEY};
        gnss_site_runtime_init();
        gnss_site_desired_t loaded;
        CHECK(gnss_site_get_desired(&loaded) && loaded.mode == GNSS_SITE_SURVEY);
        CHECK(!gnss_site_legacy_allowed()); /* leftover legacy cannot resurrect */
        stable(52,13);
        gnss_site_configured(GNSS_SITE_SURVEY,0,0,7,true);
        CHECK(!gnss_site_upload_allowed()); /* fresh proof + restored ACK ≠ grant */
        gnss_site_status_t s; gnss_site_status(&s);
        CHECK(s.applied_revision == 7 && s.mode == GNSS_SITE_SURVEY && s.valid);
    } else if (!strcmp(scenario, "observations")) {
        uint8_t frame[128];
        gnss_site_runtime_init();
        rtcm_observation_init(&observation_monitor);
        CHECK(!gnss_site_output_expected() && !gnss_observation_fault());
        stable(52,13);
        gnss_site_control_t c = fixed(1,true);
        gnss_ack = false;
        CHECK(!gnss_site_apply_control(&c) && !gnss_site_output_expected());
        gnss_ack = true;
        CHECK(gnss_site_apply_control(&c) && gnss_site_upload_allowed());
        unsigned calls = gnss_calls;
        for (unsigned i=0; i<29; ++i) sample(52,13,true);
        CHECK(!gnss_site_output_expected() && !gnss_observation_fault());
        CHECK(gnss_site_apply_control(&c) && gnss_calls == calls);
        sample(52,13,true);
        CHECK(gnss_site_output_expected() && gnss_observation_fault());
        CHECK(!gnss_site_needs_recovery()); /* fresh BASEPOS hides no MSM silence */
        const uint16_t metadata[] = {1005,1033,1230};
        for (unsigned i=0; i<3; ++i) {
            size_t len=observation_frame(frame,metadata[i],1000,OBS_VALID);
            CHECK(!rtcm_observation_note(&observation_monitor,frame,len,now_ms));
        }
        CHECK(gnss_observation_fault());
        // Zak's empty NavIC stream must not invalidate a healthy GPS family.
        for (unsigned i=1; i<=20; ++i) {
            sample(52,13,true);
            size_t len=observation_frame(frame,1077,i*1000,OBS_VALID);
            if (!rtcm_observation_note(&observation_monitor,frame,len,now_ms)) abort();
            len=observation_frame(frame,1137,i*1000,OBS_EMPTY);
            if (rtcm_observation_note(&observation_monitor,frame,len,now_ms) ||
                gnss_observation_fault()) abort();
        }
        CHECK(!gnss_observation_fault());
        for (unsigned i=0; i<10; ++i) {
            sample(52,13,true);
            size_t len=observation_frame(frame,1077,20000,OBS_VALID);
            if (rtcm_observation_note(&observation_monitor,frame,len,now_ms)) abort();
        }
        CHECK(gnss_observation_fault()); /* repeated epochs cannot conceal stall */
        c = survey(2,true);
        CHECK(gnss_site_apply_control(&c) && gnss_site_upload_allowed());
        calls = gnss_calls;
        for (unsigned i=0; i<329; ++i) {
            sample(52,13,true);
            if (!gnss_site_apply_control(&c) || gnss_site_output_expected() ||
                gnss_observation_fault()) abort();
        }
        CHECK(gnss_calls == calls && !gnss_observation_fault());
        sample(52,13,true);
        CHECK(gnss_site_output_expected() && gnss_observation_fault());
        c.allow_upload = false;
        CHECK(gnss_site_apply_control(&c) && !gnss_observation_fault());
        c.allow_upload = true;
        CHECK(gnss_site_apply_control(&c) && gnss_observation_fault());
        c = (gnss_site_control_t){.boot_id=123,.mode=GNSS_SITE_HOLD};
        CHECK(gnss_site_apply_control(&c) && !gnss_observation_fault());
        c = survey(2,true);
        CHECK(gnss_site_apply_control(&c) && gnss_observation_fault());
        now_ms += 5001;
        CHECK(!gnss_site_output_expected() && !gnss_observation_fault());
        stable(52,13);
        CHECK(gnss_site_apply_control(&c) && gnss_observation_fault());
        sample(52.001,13,true);
        CHECK(!gnss_site_output_expected() && !gnss_observation_fault());
        stable(52,13);
        CHECK(gnss_site_apply_control(&c) && gnss_observation_fault());
        gnss_site_receiver_reset();
        CHECK(!gnss_site_output_expected() && !gnss_observation_fault());
        stable(52,13);
        CHECK(!gnss_site_output_expected()); /* no mode ACK/grant after reset */
        CHECK(gnss_site_apply_control(&c) && !gnss_site_output_expected());
        for (unsigned i=0; i<329; ++i) sample(52,13,true);
        CHECK(!gnss_site_output_expected()); /* actual reset starts new warmup */
        sample(52,13,true);
        CHECK(gnss_site_output_expected() && gnss_observation_fault());
        CHECK(alive_calls > 700 && critical == 0 && mutex_depth == 0);
        // A full uint32 clock cycle must not restart an elapsed MODE grace.
        // Re-establish fresh BASEPOS after the simulated jump, without MODE.
        now_ms = s_mode_applied_ms + 1000U;
        stable(52,13); calls = gnss_calls;
        CHECK(gnss_site_apply_control(&c) && gnss_calls == calls &&
              gnss_site_output_expected());
        gnss_site_receiver_reset();
        stable(52,13);
        CHECK(gnss_site_apply_control(&c) && !gnss_site_output_expected());
    } else {
        gnss_site_runtime_init();
        CHECK(!gnss_site_upload_allowed() && gnss_site_legacy_allowed());
        CHECK(!gnss_site_needs_recovery());
        now_ms = 60000;
        CHECK(gnss_site_needs_recovery()); /* RTCM alone cannot hide lost BASEPOS */
        sample(52,13,false);
        CHECK(!gnss_site_needs_recovery() && !gnss_site_upload_allowed());
        stable(52,13);
        gnss_site_status_t s; gnss_site_status(&s);
        CHECK(s.valid && s.boot_id == 123 && !s.upload_allowed);
        gnss_site_control_t c = fixed(1,true);
        c.boot_id = 124;
        CHECK(!gnss_site_apply_control(&c) && gnss_calls == 0 && commits == 0);
        c.boot_id = 123; commit_fail = true;
        CHECK(!gnss_site_apply_control(&c) && gnss_calls == 0 && !gnss_site_upload_allowed());
        commit_fail = false; gnss_ack = false;
        CHECK(!gnss_site_apply_control(&c) && durable.revision == 1);
        gnss_site_status(&s);
        CHECK(s.desired_revision == 1 && s.applied_revision == 0 && !s.upload_allowed);
        gnss_ack = true;
        CHECK(gnss_site_apply_control(&c) && gnss_site_upload_allowed());
        gnss_site_status(&s);
        CHECK(s.desired_revision == 1 && s.applied_revision == 1 && s.mode == GNSS_SITE_FIXED);
        unsigned calls = gnss_calls, fl = flushes, com = commits;
        uint32_t gen = gnss_site_upload_generation();
        CHECK(gnss_site_apply_control(&c));
        CHECK(gnss_calls == calls && flushes == fl && commits == com &&
              gnss_site_upload_generation() == gen);
        c.lat_e9 += 1;
        CHECK(!gnss_site_apply_control(&c) && gnss_site_upload_allowed()); /* conflict ignored */
        c = fixed(0,true);
        CHECK(!gnss_site_apply_control(&c) && gnss_calls == calls);
        // A single trustworthy position outlier immediately holds (before median moves).
        sample(52.001,13,true);
        CHECK(!gnss_site_upload_allowed() && flushes > fl);
        c = fixed(1,true);
        CHECK(gnss_site_apply_control(&c) && !gnss_site_upload_allowed());
        stable(52.001,13);
        CHECK(gnss_site_apply_control(&c) && !gnss_site_upload_allowed());
        // Fence authorization for new survey is persisted/tombstones old fixed first.
        legacy_exists = true; c = survey(2,true);
        CHECK(gnss_site_apply_control(&c) && gnss_site_upload_allowed());
        CHECK(!legacy_exists && durable.mode == GNSS_SITE_SURVEY && durable.revision == 2);
        gen = gnss_site_upload_generation();
        c.allow_upload = false;
        CHECK(gnss_site_apply_control(&c) && !gnss_site_upload_allowed());
        CHECK(gnss_site_upload_generation() > gen);
        c.allow_upload = true;
        CHECK(gnss_site_apply_control(&c) && gnss_site_upload_allowed());
        // Even a survey grant from before a second move cannot reopen at new site.
        stable(52.002,13);
        CHECK(!gnss_site_upload_allowed());
        CHECK(gnss_site_apply_control(&c) && !gnss_site_upload_allowed());
        c = survey(3,true);
        CHECK(gnss_site_apply_control(&c) && gnss_site_upload_allowed());
        now_ms += 5001;
        CHECK(!gnss_site_upload_allowed()); /* direct send guard, no heartbeat needed */
        CHECK(!gnss_site_needs_recovery()); /* outage grace before reconfiguration */
        gnss_site_status(&s);
        CHECK(!s.valid && !s.upload_allowed);
        CHECK(gnss_site_apply_control(&c) && !gnss_site_upload_allowed());
        sample(52.002,13,true);
        CHECK(gnss_site_apply_control(&c) && !gnss_site_upload_allowed());
        stable(52.002,13);
        CHECK(gnss_site_apply_control(&c) && gnss_site_upload_allowed());
        sample(52.002,13,false);
        CHECK(!gnss_site_upload_allowed());
        // HOLD cannot overwrite desired revision or trigger hardware/NVS changes.
        calls = gnss_calls; com = commits;
        c = (gnss_site_control_t){.boot_id=123,.mode=GNSS_SITE_HOLD,.revision=999};
        CHECK(gnss_site_apply_control(&c) && gnss_calls == calls && commits == com);
        gnss_site_status(&s);
        CHECK(s.desired_revision == 3 && s.applied_revision == 3);
        // A recovery snapshotted an older intent before a newer one was persisted.
        gnss_site_configured(GNSS_SITE_FIXED,52,13,2,true);
        gnss_site_status(&s);
        CHECK(s.applied_revision == 0 && s.mode == GNSS_SITE_UNKNOWN);
        gnss_site_receiver_reset(); gnss_site_status(&s);
        CHECK(s.desired_revision == 3 && s.applied_revision == 0 && !s.valid && !s.upload_allowed);
        CHECK(!gnss_site_needs_recovery()); /* receiver reset gets its own grace */
        CHECK(s.seq > 0); /* receiver reset does not replay the device boot's seq */
        uint32_t prior_seq = s.seq;
        stable(52.003,13); gnss_site_status(&s);
        CHECK(s.seq > prior_seq);
        c = survey(3,true);
        CHECK(gnss_site_apply_control(&c) && !gnss_site_upload_allowed());
        CHECK(s_reference_valid); /* restored MODE cannot erase an old survey anchor */
        c = survey(4,true);
        CHECK(gnss_site_apply_control(&c) && gnss_site_upload_allowed());
        CHECK(alive_calls >= 65 && critical == 0 && mutex_depth == 0);
    }
    printf("%s: %u site runtime fault-injection checks, 0 failures\n", scenario, checks);
    return 0;
}
'''

def main():
    compiler = os.environ.get("CC") or shutil.which("gcc") or shutil.which("clang")
    if not compiler:
        raise SystemExit("Host C compiler required")
    runtime = (ROOT / "main/gnss_site_runtime.c").read_text(encoding="utf-8")
    runtime = re.sub(r"^#include .*$", "", runtime, flags=re.M)
    supervisor = (ROOT / "main/supervisor.c").read_text(encoding="utf-8")
    fault = function(supervisor, "gnss_observation_fault")
    with tempfile.TemporaryDirectory(prefix="rtkdata-site-runtime-") as temp:
        source = Path(temp) / "test.c"
        binary = Path(temp) / ("test.exe" if os.name == "nt" else "test")
        source.write_text(MOCKS + runtime + fault + TESTS, encoding="utf-8")
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        f"-I{ROOT / 'main/include'}", f"-I{ROOT / 'tests/host'}", str(source),
                        str(ROOT / "main/gnss_site.c"),
                        str(ROOT / "main/interface/ntrip_tx_core.c"), "-lm", "-o", str(binary)], check=True)
        for scenario in ("basic", "corrupt", "reboot", "observations"):
            subprocess.run([str(binary), scenario], check=True)

if __name__ == "__main__":
    main()
