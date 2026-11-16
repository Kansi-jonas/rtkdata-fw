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
/* Accepted samples since the receiver (re)started, saturating at 2. Two
 * accepted samples prove the solution epoch advanced (accept_sample takes
 * only newer epochs); the frozen-navigation rule needs that proof, or a
 * receiver that never had a solution (one no-fix record with epoch 0, then
 * the same again) was reset every minute during cold acquisition (R6). */
static uint8_t s_nav_samples;
static uint32_t s_receiver_start_ms, s_last_report_ms;
static bool s_have_report;
#ifdef CONFIG_RTK_SITE_LEGACY
/* Survey clock of the legacy observation check: when the receiver first
 * showed valid evidence after the last MODE or receiver reset (s_mux). */
static bool s_evidence_seen, s_survey_grace_done;
static uint32_t s_evidence_since_ms;
/* Relocation run (s_mux): since when every valid window has placed the
 * receiver away from the fixed reference, and how far the last one did. */
static bool s_away;
static uint32_t s_away_since_ms, s_away_mm;
/* The reference point the receiver broadcasts (last 1005/1006, s_mux),
 * cleared by every new MODE and by a receiver reset. */
static bool s_arp_valid;
static double s_arp_lat, s_arp_lon;
static uint32_t s_arp_ms;
#endif

static uint32_t site_now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

#ifdef CONFIG_RTK_SITE_LEGACY
/* Called with s_mux held after a new window snapshot. Only a receiver that
 * runs on an acknowledged reference can have left it; an invalid window (too
 * few samples, sigma above the limit) breaks the run instead of extending
 * it, so a drop needs five minutes of uninterrupted evidence.
 * The reference is the fixed coordinate, or in a survey the point the
 * receiver broadcasts in its 1005 (review 13.11. L2 #5, L3 #2): a finished
 * survey broadcasts the place it measured, and a unit moved while powered,
 * before any IE coordinate, kept broadcasting that place. Only the 1005
 * says which place that is; an SPP median taken later can already be the
 * new one. Time is read here, under the lock (L3 #3). */
static bool legacy_reference_locked(uint32_t now, double *lat, double *lon) {
    if (s_mode == GNSS_SITE_FIXED && s_reference_valid) {
        *lat = s_reference_lat;
        *lon = s_reference_lon;
        return true;
    }
    if (s_mode == GNSS_SITE_SURVEY && s_arp_valid &&
        (uint32_t)(now - s_arp_ms) <= GNSS_SITE_ARP_MAX_AGE_MS) {
        *lat = s_arp_lat;
        *lon = s_arp_lon;
        return true;
    }
    return false;
}

static void legacy_track_away_locked(void) {
    uint32_t now = site_now_ms();
    double ref_lat = 0, ref_lon = 0;
    if (!legacy_reference_locked(now, &ref_lat, &ref_lon) || !s_measurement.valid) {
        s_away = false;
        return;
    }
    double lat = s_measurement.lat_e7 / 1e7, lon = s_measurement.lon_e7 / 1e7;
    if (!gnss_site_displaced(lat, lon, s_measurement.h_sigma_mm, ref_lat, ref_lon)) {
        s_away = false;
        return;
    }
    s_away_mm = gnss_site_horizontal_mm(lat, lon, ref_lat, ref_lon);
    if (!s_away) {
        s_away = true;
        s_away_since_ms = now;
    }
}

/* Called with s_mux held. A survey counts its 300 s from the first good fix,
 * not from MODE: a late acquisition was still surveying when the MODE clock
 * ran out and got reset, again and again (review 13.11. R2, P1). The first
 * valid evidence window starts the survey clock, latched like the warmup.
 * Shared by the observation check and the OTA hold (L5). */
static void legacy_survey_clock_locked(uint32_t now, bool evidence) {
    if (evidence && !s_evidence_seen) {
        s_evidence_seen = true;
        s_evidence_since_ms = now;
    }
    if (s_evidence_seen && (uint32_t)(now - s_evidence_since_ms) >= 330000)
        s_survey_grace_done = true;
}

/* Called with s_mux held: the window describes the receiver right now. */
static bool legacy_window_fresh_locked(uint32_t now) {
    if (!s_have_sample || !s_measurement.valid) return false;
    uint64_t age = (uint64_t)(uint32_t)(now - s_last_sample_ms) + s_sample_age_ms;
    return age <= GNSS_SITE_MAX_AGE_MS;
}
#endif

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
#ifndef CONFIG_RTK_SITE_LEGACY
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
#else
    /* Legacy IE: the coordinate belongs to the legacy "base" key alone. A
     * site_v1 blob left by an earlier site-v1 build would otherwise win at
     * every boot and recovery over the coordinate the legacy IE pushes, and
     * block the runtime notification of that push (review 13.11., P1). It is
     * ignored, not erased: a later site-v1 build re-reads it, and the IE that
     * enables site-v1 sends a fresh revision anyway. */
#endif
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
    __atomic_store_n(&s_nav_samples, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&s_receiver_start_ms, site_now_ms(), __ATOMIC_RELAXED);
    __atomic_store_n(&s_have_report, false, __ATOMIC_RELEASE);
    s_applied_revision = 0;
    s_mode = GNSS_SITE_UNKNOWN;
    s_mode_warmup_done = false;
#ifdef CONFIG_RTK_SITE_LEGACY
    s_evidence_seen = false;
    s_survey_grace_done = false;
    s_away = false;
    s_arp_valid = false;
#endif
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
        /* Count accepted epochs, not ingest calls: one UART chunk can carry
         * two records (review R7), and the second one is the progress proof. */
        uint32_t adv = s_monitor.seq - old_seq;
        uint32_t nav = (uint32_t)s_nav_samples + adv;
        __atomic_store_n(&s_nav_samples, (uint8_t)(nav > 2 ? 2 : nav), __ATOMIC_RELEASE);
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
#ifdef CONFIG_RTK_SITE_LEGACY
        legacy_track_away_locked();
#endif
        portEXIT_CRITICAL(&s_mux);
    }
    xSemaphoreGive(s_monitor_mutex);
    /* BASEPOS continues when a distant fixed coordinate suppresses RTCM.
     * Receiver liveness must not reboot this deliberate held state forever. */
    if (alive) supervisor_note_gnss_rx();
    if (close) ntrip_server_site_flush();
}

#ifdef CONFIG_RTK_SITE_LEGACY
/* Legacy only, called with s_mux held: the receiver shows that RTCM should
 * be flowing. A fresh, computed, valid BASEPOS window that is not displaced
 * from the fixed coordinate in use; the same evidence the site-v1 grant
 * requires, without the IE half. */
static bool legacy_evidence_locked(uint32_t now) {
    if (!s_have_sample || !s_measurement.valid || !s_latest.quality) return false;
    uint64_t age = (uint64_t)(uint32_t)(now - s_last_sample_ms) + s_sample_age_ms;
    if (age > GNSS_SITE_MAX_AGE_MS) return false;
    return !(s_reference_valid &&
             gnss_site_displaced(s_latest.lat, s_latest.lon, s_latest.h_sigma_mm,
                                 s_reference_lat, s_reference_lon));
}
#endif

void gnss_site_runtime_note_frame(const uint8_t *frame, size_t len) {
#ifdef CONFIG_RTK_SITE_LEGACY
    double lat, lon;
    if (!__atomic_load_n(&s_initialized, __ATOMIC_ACQUIRE) ||
        !gnss_site_parse_arp(frame, len, &lat, &lon)) return;
    portENTER_CRITICAL(&s_mux);
    s_arp_lat = lat;
    s_arp_lon = lon;
    s_arp_ms = site_now_ms();
    s_arp_valid = true;
    portEXIT_CRITICAL(&s_mux);
#else
    (void)frame; (void)len;
#endif
}

gnss_site_agreement_t gnss_site_legacy_agreement(double lat, double lon,
                                                 uint32_t *distance_mm) {
#ifdef CONFIG_RTK_SITE_LEGACY
    gnss_site_agreement_t verdict = GNSS_SITE_AGREEMENT_UNKNOWN;
    uint32_t mm = 0;
    portENTER_CRITICAL(&s_mux);
    uint32_t now = site_now_ms();
    if (legacy_window_fresh_locked(now)) {
        double rlat = s_measurement.lat_e7 / 1e7, rlon = s_measurement.lon_e7 / 1e7;
        mm = gnss_site_horizontal_mm(rlat, rlon, lat, lon);
        verdict = gnss_site_displaced(rlat, rlon, s_measurement.h_sigma_mm, lat, lon) ?
            GNSS_SITE_AGREEMENT_DISAGREE : GNSS_SITE_AGREEMENT_AGREE;
    }
    portEXIT_CRITICAL(&s_mux);
    if (distance_mm) *distance_mm = mm;
    return verdict;
#else
    (void)lat; (void)lon;
    if (distance_mm) *distance_mm = 0;
    return GNSS_SITE_AGREEMENT_UNKNOWN;
#endif
}

bool gnss_site_legacy_relocated(uint32_t *distance_mm) {
#ifdef CONFIG_RTK_SITE_LEGACY
    portENTER_CRITICAL(&s_mux);
    uint32_t now = site_now_ms();
    bool moved = s_away && legacy_window_fresh_locked(now) &&
        (uint32_t)(now - s_away_since_ms) >= GNSS_SITE_RELOCATE_MS;
    /* The run was judged at the last window; a 1005 or MODE since then may
     * have moved the reference to where the unit is (review 13.11. L4).
     * Judge the current reference again before anything is dropped. */
    double ref_lat, ref_lon;
    if (moved && (!legacy_reference_locked(now, &ref_lat, &ref_lon) ||
                  !gnss_site_displaced(s_measurement.lat_e7 / 1e7, s_measurement.lon_e7 / 1e7,
                                       s_measurement.h_sigma_mm, ref_lat, ref_lon)))
        moved = false;
    uint32_t mm = s_away ? s_away_mm : 0;
    portEXIT_CRITICAL(&s_mux);
    if (distance_mm) *distance_mm = mm;
    return moved;
#else
    if (distance_mm) *distance_mm = 0;
    return false;
#endif
}

bool gnss_site_mode_settling(void) {
#ifdef CONFIG_RTK_SITE_LEGACY
    portENTER_CRITICAL(&s_mux);
    uint32_t now = site_now_ms();
    /* A survey that only found its first fix late (poor sky) still averages
     * for 330 s from that fix, however old its MODE is (review 13.11. L5):
     * the same once-per-MODE grace the observation check grants. */
    legacy_survey_clock_locked(now, legacy_evidence_locked(now));
    bool known = s_mode == GNSS_SITE_FIXED || s_mode == GNSS_SITE_SURVEY;
    bool settling = known &&
        ((uint32_t)(now - s_mode_applied_ms) < GNSS_SITE_SETTLE_MAX_MS ||
         (s_mode == GNSS_SITE_SURVEY && s_evidence_seen && !s_survey_grace_done));
    portEXIT_CRITICAL(&s_mux);
    return settling;
#else
    return false;
#endif
}

bool gnss_site_upload_allowed(void) {
#ifdef CONFIG_RTK_SITE_LEGACY
    /* Legacy IE (no site-v1): nobody can ever grant the gate, so it is open
     * from boot like on the 1.1.9 fleet. The grant flag stays false, which
     * also keeps every site-driven close path below inert (bench 13.11.). */
    return true;
#endif
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
    /* Frozen navigation (SOTA review 2026-11-06, G3): valid BASEPOS records
     * keep arriving, so the receiver looks alive, but none has carried a new
     * solution for a minute after solutions had been flowing. The upload gate
     * closes after 5 s of sample age, which also disables the observation
     * check, so before this nothing ever reset such a receiver: no upload,
     * gnss_ok true, forever. A receiver reset clears s_have_sample, so one
     * freeze costs one reset, never a loop (also when the antenna is lost).
     * Armed only after the epoch has advanced at least once since the last
     * receiver reset, so a receiver still acquiring is never reset by it. */
    if (__atomic_load_n(&s_nav_samples, __ATOMIC_ACQUIRE) >= 2 &&
        (uint32_t)(now - __atomic_load_n(&s_last_sample_ms, __ATOMIC_RELAXED)) >= 60000)
        return true;
    return !__atomic_load_n(&s_have_report, __ATOMIC_ACQUIRE) ||
        (uint32_t)(now - __atomic_load_n(&s_last_report_ms, __ATOMIC_RELAXED)) >= 10000;
}

/* Called with s_mux held: the acknowledged receiver mode has had its warmup. */
static bool mode_ready_locked(uint32_t now) {
    gnss_site_mode_t mode = s_mode;
    /* MODE BASE TIME optimizes for 300 seconds before output is expected.
     * Fixed MODE can pause RTCM for about 18 seconds; give it 30. Only actual
     * acknowledged mode application starts this clock, not grants/reconnects. */
    uint32_t warmup = mode == GNSS_SITE_SURVEY ? 330000 : 30000;
    bool known_mode = mode == GNSS_SITE_FIXED || mode == GNSS_SITE_SURVEY;
    if (known_mode && (uint32_t)(now - s_mode_applied_ms) >= warmup)
        s_mode_warmup_done = true;
    /* Keep elapsed warmup latched through holds and the 49.7-day clock wrap. */
    return known_mode && s_mode_warmup_done;
}

bool gnss_site_output_expected(void) {
    portENTER_CRITICAL(&s_mux);
    /* Time is read under the lock: a caller preempted between an earlier read
     * and the lock saw "now" before another caller's newer timestamp, and the
     * unsigned difference latched the grace at once (review 13.11. R3, P1). */
    uint32_t now = site_now_ms();
    bool ready = mode_ready_locked(now);
#ifdef CONFIG_RTK_SITE_LEGACY
    /* The open legacy gate says nothing about whether RTCM should exist: use
     * the receiver's own evidence, or a receiver still acquiring, or one that
     * suppresses RTCM because it sits >300 m from its fixed coordinate, was
     * reset by the observation check over and over (review 13.11., P1). */
    bool evidence = legacy_evidence_locked(now);
    legacy_survey_clock_locked(now, evidence);
    bool expected = ready && evidence &&
        (s_mode != GNSS_SITE_SURVEY || s_survey_grace_done);
    portEXIT_CRITICAL(&s_mux);
    return expected;
#else
    portEXIT_CRITICAL(&s_mux);
    return ready && gnss_site_upload_allowed();
#endif
}

bool gnss_site_log_repair_expected(void) {
#ifdef CONFIG_RTK_SITE_LEGACY
    /* A log repair is a plain log enable that never pauses the stream: it
     * needs proof of streaming, not position evidence. Advancing observations
     * after the mode warmup are that proof, also when BASEPOS quality keeps
     * the evidence window invalid (review 13.11. R2, P2). */
    portENTER_CRITICAL(&s_mux);
    bool ready = mode_ready_locked(site_now_ms());   /* time under the lock, see above */
    portEXIT_CRITICAL(&s_mux);
    return ready && ntrip_server_observations_silent_ms() < 10000U;
#else
    return gnss_site_output_expected();
#endif
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
#ifdef CONFIG_RTK_SITE_LEGACY
    /* Transport is always open in legacy (gnss_site_upload_allowed). Here the
     * field carries the receiver evidence instead, so the OTA health gate's
     * "verified site hold" (valid BASEPOS, RTCM deliberately suppressed at a
     * displaced fixed base) still works and does not roll back a healthy
     * pending image (review 13.11., P1). */
    out->upload_allowed = legacy_evidence_locked(now);
#else
    out->upload_allowed = __atomic_load_n(&s_allowed, __ATOMIC_ACQUIRE);
#endif
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
#ifdef CONFIG_RTK_SITE_LEGACY
    s_evidence_seen = false;
    s_survey_grace_done = false;
    s_away = false;
    s_arp_valid = false;   /* a new MODE: the old 1005 no longer says where */
    /* Legacy: s_reference is the fixed coordinate the receiver runs on and
     * nothing else (a survey is judged against its own 1005). A new survey
     * (first boot, or the relocation drop) starts without one; keeping the
     * old one judged the new survey "displaced" and switched the
     * observation check off for good. */
    if (applied && mode != GNSS_SITE_FIXED) s_reference_valid = false;
#endif
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
