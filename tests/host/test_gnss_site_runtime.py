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
/* unused in the legacy build, which never reads the site_v1 blob */
static __attribute__((unused)) int nvs_get_blob(int handle, const char *key, void *dst, size_t *size) {
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
/* The receiver repeats its last solution epoch: a CRC-valid BASEPOS that
 * proves liveness but carries no new navigation (review 2026-11-06, G3). */
static void repeat_epoch(double lat, double lon) {
    epoch_ms -= 1000;
    sample(lat, lon, true);
}
/* A recorded UM980 1005 (test_gnss_site.c): it broadcasts 49.43932637 N
 * 6.85348914 E. */
static const uint8_t ARP_FRAME[] = {0xd3,0x00,0x13,0x3e,0xd3,0xf0,0x03,0x89,0x9b,0x42,0x29,0x43,0x81,0x27,0x93,0xdc,0xd9,0x0b,0x3a,0x8f,0xb2,0xa9,0x88,0x94,0x86};
#define ARP_LAT 49.43932637
#define ARP_LON 6.85348914
__attribute__((unused)) static void arp(void) {
    gnss_site_runtime_note_frame(ARP_FRAME, sizeof(ARP_FRAME));
}
/* n receiver seconds at (lat, lon), with the receiver's 1005 every 10 s */
__attribute__((unused)) static void broadcast(double lat, double lon, unsigned n) {
    for (unsigned i = 0; i < n; ++i) {
        if (i % 10 == 0) arp();
        sample(lat, lon, true);
    }
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
    } else if (!strcmp(scenario, "legacy")) {
#ifdef CONFIG_RTK_SITE_LEGACY
        /* Legacy IE build (bench 13.11.): no IE can ever grant the gate, so it
         * is open from boot, through a receiver reset and a lost BASEPOS. */
        gnss_site_runtime_init();
        gnss_site_status_t s;
        CHECK(gnss_site_upload_allowed());
        gnss_site_status(&s);
        CHECK(!s.upload_allowed);             /* the field carries evidence: none yet */
        stable(52,13);
        sample(52.01,13,true);                /* a displaced sample closes nothing */
        CHECK(gnss_site_upload_allowed());
        gnss_site_receiver_reset();
        now_ms += 20000;                      /* BASEPOS stale for 20 s */
        CHECK(gnss_site_upload_allowed());
        gnss_site_status(&s);
        CHECK(!s.valid && !s.upload_allowed); /* field = evidence, transport stays open */

        /* Review 13.11. P1: a site_v1 blob from an earlier site-v1 build must
         * not take the coordinate away from the legacy "base" key. */
        durable_exists = legacy_exists = true;
        durable = (gnss_site_desired_t){.magic=GNSS_SITE_NVS_MAGIC,
            .revision=9,.mode=GNSS_SITE_FIXED,.lat_e9=INT64_C(48000000000),
            .lon_e9=INT64_C(7000000000),.h_mm=1000};
        gnss_site_runtime_init();
        gnss_site_desired_t ignored;
        CHECK(gnss_site_legacy_allowed() && !gnss_site_get_desired(&ignored));

        /* Review 13.11. P1: observations are expected only with the receiver's
         * own evidence, never because the legacy gate is open. */
        CHECK(!gnss_site_output_expected());            /* no mode, no evidence */
        gnss_site_configured(GNSS_SITE_FIXED, 52, 13, 0, true); /* legacy IE push */
        now_ms += 31000;                                 /* past the fixed warmup */
        CHECK(!gnss_site_output_expected());            /* still acquiring */
        stable(52,13);
        CHECK(gnss_site_output_expected());             /* fresh, valid, in place */
        gnss_site_status(&s);
        CHECK(s.upload_allowed && s.mode == GNSS_SITE_FIXED);
        stable(52.01,13);                                /* antenna moved ~1.1 km */
        CHECK(!gnss_site_output_expected());            /* UM980 suppresses RTCM */
        gnss_site_status(&s);
        CHECK(s.valid && !s.upload_allowed);            /* = OTA verified site hold */
        CHECK(gnss_site_upload_allowed());              /* transport still open */
        now_ms += 20000;
        CHECK(!gnss_site_output_expected());            /* stale evidence */

        /* R2 P1: a late first fix is not supervised from the MODE clock. */
        gnss_site_receiver_reset();
        gnss_site_configured(GNSS_SITE_SURVEY, 0, 0, 0, true);
        now_ms += 400000;                                /* MODE clock long over, no fix */
        CHECK(!gnss_site_output_expected());
        stable(52,13);                                   /* first valid evidence */
        CHECK(!gnss_site_output_expected());            /* survey clock starts now */
        for (unsigned i = 0; i < 270; ++i) sample(52,13,true);
        CHECK(!gnss_site_output_expected());            /* 270 s into the survey */
        for (unsigned i = 0; i < 70; ++i) sample(52,13,true);
        CHECK(gnss_site_output_expected());             /* 340 s after first evidence */

        /* R2 P2: log repair needs flowing observations, not position evidence. */
        rtcm_observation_init(&observation_monitor);
        CHECK(!gnss_site_log_repair_expected());        /* nothing observed yet */
        uint8_t frame[128];
        for (unsigned i = 1; i <= 3; ++i) {
            size_t len = observation_frame(frame, 1077, i * 1000, OBS_VALID);
            if (!rtcm_observation_note(&observation_monitor, frame, len, now_ms)) abort();
            now_ms += 1000;
        }
        CHECK(gnss_site_log_repair_expected());         /* settled, MSM advancing */
        now_ms += 20000;                                 /* BASEPOS and MSM both stale */
        CHECK(!gnss_site_output_expected());
        CHECK(!gnss_site_log_repair_expected());        /* observations silent */
#else
        CHECK(!"legacy scenario needs -DCONFIG_RTK_SITE_LEGACY");
#endif
    } else if (!strcmp(scenario, "relocate")) {
#ifdef CONFIG_RTK_SITE_LEGACY
        /* Review 13.11. L1 R1: the legacy relocation guard. The receiver's
         * own BASEPOS window judges every new IE coordinate, and a fixed
         * coordinate the unit has left for 5 min is dropped. */
        gnss_site_runtime_init();
        uint32_t mm = 0;
        CHECK(gnss_site_legacy_agreement(52, 13, &mm) == GNSS_SITE_AGREEMENT_UNKNOWN);
        stable(52, 13);
        CHECK(gnss_site_legacy_agreement(52, 13, &mm) == GNSS_SITE_AGREEMENT_AGREE && mm < 1000);
        /* 5.6 m: the IE coordinate vs a 1 to 3 m single-point median */
        CHECK(gnss_site_legacy_agreement(52.00005, 13, &mm) == GNSS_SITE_AGREEMENT_AGREE);
        /* 111 m: an office or garden coordinate on the roof */
        CHECK(gnss_site_legacy_agreement(52.001, 13, &mm) == GNSS_SITE_AGREEMENT_DISAGREE);
        CHECK(mm > 110000 && mm < 112000);
        now_ms += 20000;                                 /* window stale */
        CHECK(gnss_site_legacy_agreement(52, 13, &mm) == GNSS_SITE_AGREEMENT_UNKNOWN);

        /* A survey is judged against the point its 1005 broadcasts (review
         * 13.11. L2 #5, L3 #2). No 1005 yet: no reference, no run. */
        gnss_site_configured(GNSS_SITE_SURVEY, 0, 0, 0, true);
        for (unsigned i = 0; i < 400; ++i) sample(52.01, 13, true);
        CHECK(!gnss_site_legacy_relocated(&mm));
        /* Broadcasting where it is: no run. */
        broadcast(ARP_LAT, ARP_LON, 400);
        CHECK(!gnss_site_legacy_relocated(&mm));
        /* Moved while powered, before any IE coordinate. */
        broadcast(ARP_LAT + 0.01, ARP_LON, 64 + 310);
        CHECK(gnss_site_legacy_relocated(&mm) && mm > 1100000);
        /* L3 #2: moved right after the survey ended. Every window the
         * runtime ever sees is at the new place; only the 1005 knows the
         * old one, and that is enough. */
        gnss_site_configured(GNSS_SITE_SURVEY, 0, 0, 0, true);
        broadcast(ARP_LAT + 0.01, ARP_LON, 64 + 310);
        CHECK(gnss_site_legacy_relocated(&mm));
        /* L4: the run is judged at each window; a 1005 that arrives in
         * between and puts the reference where the unit is must stop a drop
         * at once. Second frame: the golden one with Z + 1000 m. */
        {
            uint8_t moved_arp[sizeof(ARP_FRAME)];
            memcpy(moved_arp, ARP_FRAME, sizeof(moved_arp));
            uint64_t z = UINT64_C(48227136169) + UINT64_C(10000000);
            for (unsigned i = 0; i < 38; ++i) {
                unsigned bit = 3 * 8 + 114 + i;
                uint8_t m = (uint8_t)(1u << (7 - (bit & 7)));
                if ((z >> (37 - i)) & 1) moved_arp[bit >> 3] |= m;
                else moved_arp[bit >> 3] &= (uint8_t)~m;
            }
            double lat2 = 0, lon2 = 0;
            CHECK(gnss_site_parse_arp(moved_arp, sizeof(moved_arp), &lat2, &lon2));
            CHECK(gnss_site_horizontal_mm(lat2, lon2, ARP_LAT, ARP_LON) > 500000);
            gnss_site_configured(GNSS_SITE_SURVEY, 0, 0, 0, true);
            broadcast(lat2, lon2, 64 + 310);              /* 1005 says ARP, unit at 2 */
            CHECK(gnss_site_legacy_relocated(&mm));
            gnss_site_runtime_note_frame(moved_arp, sizeof(moved_arp));
            CHECK(!gnss_site_legacy_relocated(&mm));      /* no new window needed */
        }
        gnss_site_configured(GNSS_SITE_SURVEY, 0, 0, 0, true);
        broadcast(ARP_LAT + 0.01, ARP_LON, 64 + 310);
        CHECK(gnss_site_legacy_relocated(&mm));
        /* A 1005 older than 30 s is no reference: the run ends. */
        for (unsigned i = 0; i < 40; ++i) sample(ARP_LAT + 0.01, ARP_LON, true);
        CHECK(!gnss_site_legacy_relocated(&mm));
        /* A new MODE forgets the old 1005. */
        arp();
        gnss_site_configured(GNSS_SITE_SURVEY, 0, 0, 0, true);
        for (unsigned i = 0; i < 400; ++i) sample(ARP_LAT + 0.01, ARP_LON, true);
        CHECK(!gnss_site_legacy_relocated(&mm));
        /* So does a receiver reset. */
        broadcast(ARP_LAT + 0.01, ARP_LON, 64 + 310);
        CHECK(gnss_site_legacy_relocated(&mm));
        gnss_site_receiver_reset();
        gnss_site_configured(GNSS_SITE_SURVEY, 0, 0, 0, true);
        for (unsigned i = 0; i < 400; ++i) sample(ARP_LAT + 0.01, ARP_LON, true);
        CHECK(!gnss_site_legacy_relocated(&mm));

        /* L4 #1: the OTA hold only covers an acknowledged MODE, and only for
         * GNSS_SITE_SETTLE_MAX_MS. */
        CHECK(gnss_site_mode_settling());                 /* survey 400 s old */
        now_ms += GNSS_SITE_SETTLE_MAX_MS;
        CHECK(!gnss_site_mode_settling());
        gnss_site_configured(GNSS_SITE_FIXED, 52, 13, 0, false);   /* NACK: UNKNOWN */
        CHECK(!gnss_site_mode_settling());
        gnss_site_configured(GNSS_SITE_FIXED, 52, 13, 0, true);
        CHECK(gnss_site_mode_settling());
        gnss_site_receiver_reset();                       /* mode UNKNOWN */
        CHECK(!gnss_site_mode_settling());
        /* L5: a survey whose first fix comes after the 15 min still gets its
         * 330 s from that fix, once. */
        gnss_site_configured(GNSS_SITE_SURVEY, 0, 0, 0, true);
        now_ms += GNSS_SITE_SETTLE_MAX_MS + 60000;        /* no fix for 16 min */
        CHECK(!gnss_site_mode_settling());                /* nothing to wait for */
        stable(52, 13);                                   /* first valid window */
        CHECK(gnss_site_mode_settling());
        for (unsigned i = 0; i < 200; ++i) sample(52, 13, true);
        CHECK(gnss_site_mode_settling());                 /* 200 s into the survey */
        for (unsigned i = 0; i < 140; ++i) sample(52, 13, true);
        CHECK(!gnss_site_mode_settling());                /* 340 s: output is due */
        CHECK(gnss_site_output_expected());

        /* Fixed in place, then moved ~1.1 km: no drop before 5 min. */
        gnss_site_configured(GNSS_SITE_FIXED, 52, 13, 0, true);
        stable(52, 13);
        CHECK(!gnss_site_legacy_relocated(&mm) && mm == 0);
        for (unsigned i = 0; i < 200; ++i) sample(52.01, 13, true);
        CHECK(!gnss_site_legacy_relocated(&mm));
        CHECK(mm > 1100000 && mm < 1125000);            /* distance known early */
        /* A window mixing both places has a km-sized spread and is invalid:
         * the run starts once all 64 samples are at the new place. */
        for (unsigned i = 0; i < 150; ++i) sample(52.01, 13, true);
        CHECK(!gnss_site_legacy_relocated(&mm));         /* 286 s of run */
        for (unsigned i = 0; i < 20; ++i) sample(52.01, 13, true);
        CHECK(gnss_site_legacy_relocated(&mm));          /* > 300 s of evidence */
        now_ms += 20000;                                 /* evidence went stale */
        CHECK(!gnss_site_legacy_relocated(&mm));

        /* One sample without a single-point fix invalidates the window and
         * restarts the run: five minutes of UNINTERRUPTED evidence. */
        gnss_site_configured(GNSS_SITE_FIXED, 52, 13, 0, true);
        stable(52, 13);
        for (unsigned i = 0; i < 250; ++i) sample(52.01, 13, true);
        sample(52.01, 13, false);
        /* The no-fix record empties the window (gnss_site.c accept_sample):
         * valid again after 60 s, and only then does a new run start. */
        for (unsigned i = 0; i < 300; ++i) sample(52.01, 13, true);
        CHECK(!gnss_site_legacy_relocated(&mm));         /* run about 240 s */
        for (unsigned i = 0; i < 70; ++i) sample(52.01, 13, true);
        CHECK(gnss_site_legacy_relocated(&mm));

        /* The drop: a survey has no reference, so the new place is in place
         * (observation check and OTA evidence back on), and no run remains. */
        gnss_site_configured(GNSS_SITE_SURVEY, 0, 0, 0, true);
        CHECK(!gnss_site_legacy_relocated(&mm));
        gnss_site_status_t s;
        gnss_site_status(&s);
        CHECK(s.valid && s.upload_allowed);              /* evidence, not "displaced" */
        CHECK(gnss_site_legacy_agreement(52, 13, &mm) == GNSS_SITE_AGREEMENT_DISAGREE);
        CHECK(gnss_site_legacy_agreement(52.01, 13, &mm) == GNSS_SITE_AGREEMENT_AGREE);

        /* An unacknowledged MODE (mode UNKNOWN) and a receiver reset end a run. */
        gnss_site_configured(GNSS_SITE_FIXED, 52, 13, 0, true);
        for (unsigned i = 0; i < 400; ++i) sample(52.01, 13, true);
        CHECK(gnss_site_legacy_relocated(&mm));
        gnss_site_configured(GNSS_SITE_FIXED, 52, 13, 0, false);
        CHECK(!gnss_site_legacy_relocated(&mm));
        gnss_site_configured(GNSS_SITE_FIXED, 52, 13, 0, true);
        for (unsigned i = 0; i < 400; ++i) sample(52.01, 13, true);
        CHECK(gnss_site_legacy_relocated(&mm));
        gnss_site_receiver_reset();
        CHECK(!gnss_site_legacy_relocated(&mm));
        CHECK(critical == 0 && mutex_depth == 0);
#else
        CHECK(!"relocate scenario needs -DCONFIG_RTK_SITE_LEGACY");
#endif
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
    } else if (!strcmp(scenario, "frozen")) {
        gnss_site_runtime_init();
        now_ms = 60000;                       /* past the receiver-start grace */
        stable(52,13);
        CHECK(!gnss_site_needs_recovery());
        unsigned alive_before = alive_calls;
        for (unsigned i = 0; i < 58; ++i) repeat_epoch(52,13);
        CHECK(alive_calls == alive_before + 58);  /* liveness keeps flowing */
        CHECK(!gnss_site_upload_allowed());   /* sample age closed the gate */
        CHECK(!gnss_site_needs_recovery());   /* under a minute: no reset yet */
        repeat_epoch(52,13); repeat_epoch(52,13);
        CHECK(gnss_site_needs_recovery());    /* frozen navigation -> recover */
        /* one reset per freeze: after it there is no sample to be stale */
        gnss_site_receiver_reset();
        now_ms += 61000;
        sample(52,13,true);
        CHECK(!gnss_site_needs_recovery());
        sample(52,13,true);
        CHECK(!gnss_site_needs_recovery());   /* advancing again: healthy */
        /* R6: a receiver without any progress (one record, then the same
         * epoch again and again) must never be reset by this rule */
        gnss_site_receiver_reset();
        now_ms += 61000;
        sample(52,13,false);
        for (unsigned i = 0; i < 130; ++i) repeat_epoch(52,13);
        CHECK(!gnss_site_needs_recovery());
        /* R7: two advancing records arriving in ONE UART chunk arm the rule
         * too; a frozen receiver after that is recovered */
        gnss_site_receiver_reset();
        now_ms += 61000;
        {
            char two[2 * GNSS_SITE_LINE_MAX];
            size_t n2 = 0;
            for (int k = 0; k < 2; ++k) {
                char line[GNSS_SITE_LINE_MAX];
                epoch_ms += 1000;
                int n = snprintf(line, sizeof(line),
                    "#BASEPOSA,96,GPS,FINE,2440,%u,0,0,18,20;"
                    "SOL_COMPUTED,SINGLE,%.11f,%.11f,40,2,WGS84,1,1,2,"
                    "\"0\",0,0,30,12,28,0,16,12,01,51,"
                    "SOL_COMPUTED,DOPPLER_VELOCITY,0,0,0,0,0,0,0",
                    epoch_ms, 52.0, 13.0);
                uint32_t crc = gnss_site_crc32((const uint8_t *)line+1, (size_t)n-1);
                n += snprintf(line+n, sizeof(line)-(size_t)n, "*%08x\r\n", crc);
                memcpy(two + n2, line, (size_t)n); n2 += (size_t)n;
            }
            now_ms += 1000;
            gnss_site_runtime_ingest((const uint8_t *)two, n2);
        }
        for (unsigned i = 0; i < 61; ++i) repeat_epoch(52,13);
        CHECK(gnss_site_needs_recovery());
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
        for scenario in ("basic", "corrupt", "reboot", "observations", "frozen"):
            subprocess.run([str(binary), scenario], check=True)
        # The same runtime built as the legacy-IE firmware (Kconfig
        # RTK_SITE_LEGACY, default on until the IE serves site-v1).
        legacy = Path(temp) / ("legacy.exe" if os.name == "nt" else "legacy")
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-DCONFIG_RTK_SITE_LEGACY=1",
                        f"-I{ROOT / 'main/include'}", f"-I{ROOT / 'tests/host'}", str(source),
                        str(ROOT / "main/gnss_site.c"),
                        str(ROOT / "main/interface/ntrip_tx_core.c"), "-lm", "-o", str(legacy)], check=True)
        for scenario in ("legacy", "relocate"):
            subprocess.run([str(legacy), scenario], check=True)

if __name__ == "__main__":
    main()
