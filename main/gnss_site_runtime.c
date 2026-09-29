/* Device adapter for the independently tested BASEPOS/site-v1 core.
 * Upload grants are volatile, per boot, and never survive receiver reset. */
#include "gnss_site.h"
#include "gnss.h"
#include "interface/ntrip.h"
#include "supervisor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <nvs.h>
#include <math.h>
#include <string.h>

#define TAG "GNSS_SITE"
#define SITE_NVS_NS "gnss"
#define SITE_NVS_KEY "site_v1"

static gnss_site_monitor_t s_monitor;
static gnss_site_sample_t s_latest;
static gnss_site_status_t s_measurement;
static gnss_site_desired_t s_desired;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_control_mutex;
static SemaphoreHandle_t s_monitor_mutex;
static bool s_initialized, s_managed, s_storage_fault;
static uint32_t s_boot_id, s_applied_revision;
static gnss_site_mode_t s_mode;
static uint32_t s_mode_applied_ms;
static bool s_mode_warmup_done;
static bool s_reference_valid;
static double s_reference_lat, s_reference_lon;
static bool s_allowed;       /* atomic access: also read by the socket send callback */
static uint32_t s_generation;
static uint32_t s_last_sample_ms, s_sample_age_ms;
static bool s_have_sample;
static uint32_t s_receiver_start_ms, s_last_report_ms;
static bool s_have_report;

static uint32_t site_now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

/* Called while s_mux is held. No RTOS mutex/socket call in the UART critical path. */
static void close_locked(void) {
    __atomic_store_n(&s_allowed, false, __ATOMIC_RELEASE);
    __atomic_add_fetch(&s_generation, 1, __ATOMIC_ACQ_REL);
}

static bool store_desired(const gnss_site_desired_t *desired) {
    nvs_handle_t handle;
    if (nvs_open(SITE_NVS_NS, NVS_READWRITE, &handle) != ESP_OK) return false;
    esp_err_t err = nvs_set_blob(handle, SITE_NVS_KEY, desired, sizeof(*desired));
    /* The authoritative survey tombstone and removal of the old coordinate
     * are committed before any receiver command can be issued. */
    if (err == ESP_OK && desired->mode == GNSS_SITE_SURVEY) {
        err = nvs_erase_key(handle, "base");
        if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;
    }
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err == ESP_OK;
}

void gnss_site_runtime_init(void) {
    s_control_mutex = xSemaphoreCreateMutex();
    s_monitor_mutex = xSemaphoreCreateMutex();
    s_boot_id = esp_random();
    if (!s_boot_id) s_boot_id = 1;
    gnss_site_monitor_init(&s_monitor);
    __atomic_store_n(&s_receiver_start_ms, site_now_ms(), __ATOMIC_RELAXED);
    memset(&s_measurement, 0, sizeof(s_measurement));
    s_measurement.age_ms = UINT32_MAX;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(SITE_NVS_NS, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        size_t size = sizeof(s_desired);
        err = nvs_get_blob(handle, SITE_NVS_KEY, &s_desired, &size);
        if (err == ESP_OK) {
            s_managed = true;
            if (size != sizeof(s_desired) || !gnss_site_desired_valid(&s_desired))
                s_storage_fault = true;
        } else if (err != ESP_ERR_NVS_NOT_FOUND) {
            s_storage_fault = true;
        }
        nvs_close(handle);
    } else if (err != ESP_ERR_NVS_NOT_FOUND) {
        s_storage_fault = true;
    }
    if (!s_control_mutex || !s_monitor_mutex) s_storage_fault = true;
    __atomic_store_n(&s_allowed, false, __ATOMIC_RELEASE);
    __atomic_store_n(&s_generation, 1, __ATOMIC_RELEASE);
    __atomic_store_n(&s_initialized, true, __ATOMIC_RELEASE);
    if (s_storage_fault) ESP_LOGE(TAG, "site control storage unavailable; upload held");
}

void gnss_site_receiver_reset(void) {
    if (!__atomic_load_n(&s_initialized, __ATOMIC_ACQUIRE)) return;
    if (s_monitor_mutex) xSemaphoreTake(s_monitor_mutex, portMAX_DELAY);
    portENTER_CRITICAL(&s_mux);
    close_locked();
    uint32_t seq = s_monitor.seq;
    gnss_site_monitor_init(&s_monitor);
    s_monitor.seq = seq;
    memset(&s_latest, 0, sizeof(s_latest));
    memset(&s_measurement, 0, sizeof(s_measurement));
    s_measurement.seq = seq;
    s_measurement.age_ms = UINT32_MAX;
    s_have_sample = false;
    __atomic_store_n(&s_receiver_start_ms, site_now_ms(), __ATOMIC_RELAXED);
    __atomic_store_n(&s_have_report, false, __ATOMIC_RELEASE);
    s_applied_revision = 0;
    s_mode = GNSS_SITE_UNKNOWN;
    s_mode_warmup_done = false;
    /* A receiver reset is not a new physical site and does not authorize
     * forgetting the survey anchor. Only a newer persisted revision does. */
    portEXIT_CRITICAL(&s_mux);
    if (s_monitor_mutex) xSemaphoreGive(s_monitor_mutex);
    ntrip_server_site_flush();
}

void gnss_site_runtime_ingest(const uint8_t *bytes, size_t len) {
    if (!__atomic_load_n(&s_initialized, __ATOMIC_ACQUIRE) ||
        !s_monitor_mutex || !bytes || !len) return;
    uint32_t now = site_now_ms();
    bool close = false;
    /* Parsing strtod and statistical math must run with interrupts enabled:
     * newlib conversion may allocate. UART owns this monitor; the separate
     * short-lived mutex only serializes explicit receiver reset. */
    xSemaphoreTake(s_monitor_mutex, portMAX_DELAY);
    uint32_t old_seq = s_monitor.seq;
    bool alive = gnss_site_monitor_feed(&s_monitor, bytes, len, now);
    if (alive) {
        __atomic_store_n(&s_last_report_ms, now, __ATOMIC_RELAXED);
        __atomic_store_n(&s_have_report, true, __ATOMIC_RELEASE);
    }
    if (old_seq != s_monitor.seq) {
        gnss_site_status_t measurement;
        gnss_site_monitor_snapshot(&s_monitor, now, &measurement);
        portENTER_CRITICAL(&s_mux);
        s_measurement = measurement;
        s_latest = s_monitor.latest;
        s_have_sample = true;
        __atomic_store_n(&s_last_sample_ms, s_monitor.latest.received_ms, __ATOMIC_RELAXED);
        __atomic_store_n(&s_sample_age_ms, s_monitor.latest.solution_age_ms, __ATOMIC_RELAXED);
        bool displaced = s_reference_valid && s_monitor.latest.quality &&
            gnss_site_displaced(s_monitor.latest.lat, s_monitor.latest.lon,
                                s_monitor.latest.h_sigma_mm,
                                s_reference_lat, s_reference_lon);
        if (__atomic_load_n(&s_allowed, __ATOMIC_ACQUIRE) &&
            (!s_measurement.valid || displaced)) {
            close_locked();
            close = true;
        }
        portEXIT_CRITICAL(&s_mux);
    }
    xSemaphoreGive(s_monitor_mutex);
    /* BASEPOS continues when a distant fixed coordinate suppresses RTCM.
     * Receiver liveness must not reboot this deliberate held state forever. */
    if (alive) supervisor_note_gnss_rx();
    if (close) ntrip_server_site_flush();
}

bool gnss_site_upload_allowed(void) {
    if (!__atomic_load_n(&s_allowed, __ATOMIC_ACQUIRE)) return false;
    /* A stalled UART must close the effective gate without waiting for the
     * next heartbeat or another incoming byte. */
    uint32_t now = site_now_ms();
    uint32_t received = __atomic_load_n(&s_last_sample_ms, __ATOMIC_RELAXED);
    uint32_t age = __atomic_load_n(&s_sample_age_ms, __ATOMIC_RELAXED);
    return (uint32_t)(now - received) <= GNSS_SITE_MAX_AGE_MS &&
        (uint64_t)(uint32_t)(now - received) + age <= GNSS_SITE_MAX_AGE_MS;
}

bool gnss_site_needs_recovery(void) {
    if (!__atomic_load_n(&s_initialized, __ATOMIC_ACQUIRE)) return false;
    uint32_t now = site_now_ms();
    /* Give boot/recovery configuration and the first solution time to settle.
     * RTCM can remain healthy while BASEPOS logging was rejected/lost. */
    if ((uint32_t)(now - __atomic_load_n(&s_receiver_start_ms, __ATOMIC_RELAXED)) < 60000)
        return false;
    return !__atomic_load_n(&s_have_report, __ATOMIC_ACQUIRE) ||
        (uint32_t)(now - __atomic_load_n(&s_last_report_ms, __ATOMIC_RELAXED)) >= 10000;
}

bool gnss_site_output_expected(void) {
    portENTER_CRITICAL(&s_mux);
    gnss_site_mode_t mode = s_mode;
    /* MODE BASE TIME optimizes for 300 seconds before output is expected.
     * Fixed MODE can pause RTCM for about 18 seconds; give it 30. Only actual
     * acknowledged mode application starts this clock, not grants/reconnects. */
    uint32_t warmup = mode == GNSS_SITE_SURVEY ? 330000 : 30000;
    bool known_mode = mode == GNSS_SITE_FIXED || mode == GNSS_SITE_SURVEY;
    if (known_mode && (uint32_t)(site_now_ms() - s_mode_applied_ms) >= warmup)
        s_mode_warmup_done = true;
    /* Keep elapsed warmup latched through holds and the 49.7-day clock wrap. */
    bool ready = known_mode && s_mode_warmup_done;
    portEXIT_CRITICAL(&s_mux);
    return ready && gnss_site_upload_allowed();
}

uint32_t gnss_site_upload_generation(void) {
    return __atomic_load_n(&s_generation, __ATOMIC_ACQUIRE);
}

void gnss_site_status(gnss_site_status_t *out) {
    bool close = false;
    uint32_t now = site_now_ms();
    portENTER_CRITICAL(&s_mux);
    *out = s_measurement;
    if (s_have_sample) {
        uint64_t age = (uint32_t)(now - s_last_sample_ms);
        age += s_sample_age_ms;
        out->age_ms = age >= UINT32_MAX ? UINT32_MAX : (uint32_t)age;
    }
    if (out->age_ms > GNSS_SITE_MAX_AGE_MS) out->valid = false;
    if (!out->valid && __atomic_load_n(&s_allowed, __ATOMIC_ACQUIRE)) {
        close_locked();
        close = true;
    }
    out->boot_id = s_boot_id;
    out->desired_revision = s_managed && !s_storage_fault ? s_desired.revision : 0;
    out->applied_revision = s_applied_revision;
    out->mode = s_mode;
    out->upload_allowed = __atomic_load_n(&s_allowed, __ATOMIC_ACQUIRE);
    portEXIT_CRITICAL(&s_mux);
    if (close) ntrip_server_site_flush();
}

bool gnss_site_get_desired(gnss_site_desired_t *out) {
    portENTER_CRITICAL(&s_mux);
    bool valid = s_managed && !s_storage_fault && gnss_site_desired_valid(&s_desired);
    if (valid) *out = s_desired;
    portEXIT_CRITICAL(&s_mux);
    return valid;
}

bool gnss_site_legacy_allowed(void) {
    return !s_managed && !s_storage_fault;
}

void gnss_site_configured(gnss_site_mode_t mode, double lat, double lon,
                          uint32_t expected_revision, bool applied) {
    portENTER_CRITICAL(&s_mux);
    /* A concurrent receiver recovery may have snapshotted an older desired
     * revision before the heartbeat persisted its replacement. Never ACK the
     * replacement on that old hardware transaction's behalf. */
    if ((s_managed ? s_desired.revision : 0) != expected_revision) applied = false;
    s_mode = applied ? mode : GNSS_SITE_UNKNOWN;
    s_mode_warmup_done = false;
    s_applied_revision = applied && s_managed && !s_storage_fault ? expected_revision : 0;
    if (applied) s_mode_applied_ms = site_now_ms();
    if (applied && mode == GNSS_SITE_FIXED) {
        s_reference_valid = true;
        s_reference_lat = lat;
        s_reference_lon = lon;
    }
    portEXIT_CRITICAL(&s_mux);
}

bool gnss_site_apply_control(const gnss_site_control_t *control) {
    if (!control || !__atomic_load_n(&s_initialized, __ATOMIC_ACQUIRE) ||
        !s_control_mutex || control->boot_id != s_boot_id)
        return false;
    xSemaphoreTake(s_control_mutex, portMAX_DELAY);
    if (control->mode == GNSS_SITE_HOLD) {
        portENTER_CRITICAL(&s_mux);
        close_locked();
        portEXIT_CRITICAL(&s_mux);
        ntrip_server_site_flush();
        xSemaphoreGive(s_control_mutex);
        return true;
    }
    gnss_site_desired_t desired = {
        .magic = GNSS_SITE_NVS_MAGIC, .revision = control->revision,
        .mode = (uint32_t)control->mode, .lat_e9 = control->lat_e9,
        .lon_e9 = control->lon_e9, .h_mm = control->h_mm
    };
    if (!gnss_site_desired_valid(&desired) || s_storage_fault ||
        (s_managed && (desired.revision < s_desired.revision ||
         (desired.revision == s_desired.revision && !gnss_site_control_matches(&s_desired, control))))) {
        xSemaphoreGive(s_control_mutex);
        return false;
    }
    bool changed = !s_managed || desired.revision != s_desired.revision;
    portENTER_CRITICAL(&s_mux);
    bool applied = !changed && s_applied_revision == desired.revision &&
                   s_mode == (gnss_site_mode_t)desired.mode;
    portEXIT_CRITICAL(&s_mux);
    /* Repeating an unchanged grant must not interrupt the SOURCE connection
     * every heartbeat. A locally tripped/stale gate still needs fresh proof. */
    if (applied && control->allow_upload && gnss_site_upload_allowed()) {
        xSemaphoreGive(s_control_mutex);
        return true;
    }
    portENTER_CRITICAL(&s_mux);
    close_locked();
    portEXIT_CRITICAL(&s_mux);
    ntrip_server_site_flush();
    if (changed) {
        if (!store_desired(&desired)) {
            ESP_LOGE(TAG, "cannot persist site revision; command not applied");
            xSemaphoreGive(s_control_mutex);
            return false;
        }
        portENTER_CRITICAL(&s_mux);
        s_desired = desired;
        s_managed = true;
        s_applied_revision = 0;
        s_reference_valid = false;
        portEXIT_CRITICAL(&s_mux);
    }
    if (!applied) {
        applied = gnss_apply_site_revision(&desired);
    }
    if (applied && control->allow_upload) {
        /* Clear any old complete/partial upload frames while still held.
         * The final check and publication of the grant share the UART lock,
         * so a concurrently received outlier always wins. */
        ntrip_server_site_flush();
        portENTER_CRITICAL(&s_mux);
        uint32_t now = site_now_ms();
        bool fresh = s_have_sample &&
            (uint64_t)(uint32_t)(now - s_last_sample_ms) + s_sample_age_ms <= GNSS_SITE_MAX_AGE_MS;
        /* An old survey grant cannot reopen at a moved site. Only a newer
         * persisted survey revision clears its previous physical anchor. */
        bool agreement = !s_reference_valid ||
            !gnss_site_displaced(s_latest.lat, s_latest.lon,
                                 s_latest.h_sigma_mm,
                                 s_reference_lat, s_reference_lon);
        if (desired.mode == GNSS_SITE_FIXED)
            agreement = !gnss_site_displaced(s_latest.lat, s_latest.lon,
                s_latest.h_sigma_mm, desired.lat_e9 / 1e9, desired.lon_e9 / 1e9);
        if (fresh && s_measurement.valid && s_latest.quality && agreement &&
            s_applied_revision == desired.revision &&
            s_mode == (gnss_site_mode_t)desired.mode) {
            s_reference_valid = true;
            s_reference_lat = desired.mode == GNSS_SITE_FIXED ? desired.lat_e9 / 1e9 :
                s_measurement.lat_e7 / 1e7;
            s_reference_lon = desired.mode == GNSS_SITE_FIXED ? desired.lon_e9 / 1e9 :
                s_measurement.lon_e7 / 1e7;
            __atomic_add_fetch(&s_generation, 1, __ATOMIC_ACQ_REL);
            __atomic_store_n(&s_allowed, true, __ATOMIC_RELEASE);
        }
        portEXIT_CRITICAL(&s_mux);
    }
    xSemaphoreGive(s_control_mutex);
    return applied;
}
