/*
 * RTKdata reference firmware - UM980 GNSS configuration (SOTA, ACK-gated).
 *
 * The UM980 echoes every command: "$command,<cmd>,response: OK*<crc>" on success,
 * "...response: PARSING FAILD NO MATCHING FUNC <cmd>*<crc>" on failure. We capture
 * the receiver's UART output via a registered read handler and gate each command on
 * its ACK with retries - replacing the stock blind vTaskDelay(500ms) open loop.
 *
 * Reference-base message set (see docs/UM980-config-research.md):
 *   MSM7 1077/1087/1097/1117/1127/1137 @1Hz  (full-resolution observables)
 *   1005 @10s  - ARP identity anchor (Lighthouse rewrites this to the catalog ECEF)
 *   1033 @10s  - receiver/antenna descriptor
 *   1230 @10s  - GLONASS code-phase biases (missing in stock OnoLink; needed for
 *                cross-brand GLONASS RTK)
 * Position is provisional survey-in here; the precise fixed coordinate is pushed by
 * the IE via gnss_set_fixed_base().
 *
 * License: GPLv3 (see LICENSE).
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <esp_event.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include "nvs.h"

#include "board.h"
#include "gnss.h"
#include "gnss_ack.h"
#include "gnss_capture.h"
#include "uart.h"
#include "supervisor.h"
#include "interface/ntrip.h"

#define TAG "GNSS"

#define GPIO_GNSS_RESET    BOARD_GNSS_RESET_GPIO
#define GNSS_RESET_MS      500
#define ACK_TIMEOUT_MS     900   // UM980 usually replies within ~100-300 ms
#define ACK_POLL_MS        20
#define ACK_RETRIES        3
#define MODE_ACK_TIMEOUT_MS 3000  // MODE is disruptive: wait long, resend once
#define MODE_RETRIES       1

/* ---- UM980 command response capture -----------------------------------
 * uart_task feeds every UART chunk in while we are configuring. The capture
 * (gnss_capture.c) keeps only complete '$'/'#' ASCII lines: the old buffer
 * took the raw stream and cleared itself whenever a chunk did not fit, so with
 * satellites tracked an MSM7 burst wiped the ACK before the 20 ms poll read
 * it. Every command then failed its first attempt and "mode base" was re-sent,
 * which stalls RTCM for seconds (bench, 2026-11-06).
 */
#define CAP_SZ GNSS_CAP_SZ
static gnss_cap_t s_cap_state;
static volatile bool s_capturing = false;
static portMUX_TYPE s_cap_mux = portMUX_INITIALIZER_UNLOCKED;

/* Called SYNCHRONOUSLY from uart_task for every chunk. The command/ACK path is
 * authoritative control traffic and must never be lossy: it used to ride the
 * esp_event bus, and when that fanout was made best-effort (timeout 0) to stop
 * a slow secondary socket from blocking the sole UART reader, ACKs became
 * droppable too - a successful "mode base" could read as four failed attempts
 * (review 2026-08-03, self-inflicted regression). Bounded work under a
 * spinlock, no allocation, no event queue. */
void gnss_ingest_uart(const uint8_t *data, size_t len) {
    gnss_site_runtime_ingest(data, len);
    if (!s_capturing || data == NULL || len == 0) return;
    portENTER_CRITICAL(&s_cap_mux);
    gnss_cap_feed(&s_cap_state, data, len);
    portEXIT_CRITICAL(&s_cap_mux);
}

static void cap_reset(void) {
    portENTER_CRITICAL(&s_cap_mux);
    gnss_cap_reset(&s_cap_state);
    portEXIT_CRITICAL(&s_cap_mux);
}

/* Binary-safe search: RTCM bytes share this buffer with the ASCII response,
 * and a legal 0x00 BEFORE the response must not truncate the haystack the way
 * strstr did (review 2026-08-03: an applied command could read as a timeout).
 * Returns the offset just past the match, or -1. */
static int cap_find_from(const char *hay, size_t n, size_t from, const char *needle) {
    size_t nlen = strlen(needle);
    if (nlen == 0 || n < nlen) return -1;
    for (size_t i = from; i + nlen <= n; i++) {
        if (hay[i] == needle[0] && memcmp(hay + i, needle, nlen) == 0) {
            return (int)(i + nlen);
        }
    }
    return -1;
}

static size_t cap_snapshot(char *dst) {
    portENTER_CRITICAL(&s_cap_mux);
    size_t n = s_cap_state.len;
    memcpy(dst, s_cap_state.buf, n);
    portEXIT_CRITICAL(&s_cap_mux);
    return n;
}

static bool cap_contains(const char *needle) {
    static char tmp[CAP_SZ];
    size_t n = cap_snapshot(tmp);
    return cap_find_from(tmp, n, 0, needle) >= 0;
}

/* Correlated verdict lookup, implemented and host-tested in gnss_ack.c. The
 * receiver uses two grammars that put the command on OPPOSITE sides of the
 * verdict, so this cannot be done with a single "find the command, then look
 * for OK/FAIL" scan - that version missed every real reject
 * (review 2026-08-03, tests/host/test_gnss_ack.c). */
static gnss_ack_verdict_t cap_verdict_for(const char *cmd_trimmed) {
    static char tmp[CAP_SZ];
    size_t n = cap_snapshot(tmp);
    return gnss_ack_scan(tmp, n, cmd_trimmed);
}

/* One transaction owner at a time: the supervisor (gnss_recover) and the
 * provisioning heartbeat (gnss_set_fixed_base) run on different tasks but
 * share the capture buffer and the generic-OK detector. Without this lock,
 * one task's "response: OK" can confirm the OTHER task's command, or a reset
 * can land between a command and its ACK (review 2026-08-03, P0). Recursive:
 * gnss_recover -> config_gnss_base nests. The full command-echo correlation
 * and a dedicated GNSS control task remain tracked architectural work; this
 * lock removes the cross-task interleaving today. */
/* Statically allocated, so creation cannot fail and leave every transaction
 * unserialized (review 2026-11-06, G5). Created first thing in gnss_init();
 * a caller that runs before that (the K_EVK button task starts earlier)
 * must check s_txn and defer, see gnss_after_external_reset(). */
static StaticSemaphore_t s_txn_buf;
static SemaphoreHandle_t s_txn = NULL;
static void txn_take(void) { if (s_txn) xSemaphoreTakeRecursive(s_txn, portMAX_DELAY); }
static void txn_give(void) { if (s_txn) xSemaphoreGiveRecursive(s_txn); }

/* ---- ACK-gated command send ------------------------------------------- */

static bool send_cmd_acked(const char *cmd, int retries) {
    int show = (int)strlen(cmd);
    while (show > 0 && (cmd[show - 1] == '\r' || cmd[show - 1] == '\n')) show--;  // trim CRLF for logs

    // The command text without CRLF, which is exactly what the receiver echoes
    // back inside "$command,<text>,response: ...".
    char echo[96];
    int  elen = show < (int)sizeof(echo) - 1 ? show : (int)sizeof(echo) - 1;
    memcpy(echo, cmd, (size_t)elen);
    echo[elen] = '\0';

    // "mode base ..." restarts the receiver's base engine and pauses RTCM
    // output each time it is APPLIED. A retry after a late or lost ACK applies
    // it again, so MODE gets a long window and one retry, never four sends
    // (review 2026-11-06, G1). Bench 2026-11-06: every ACK, MODE included,
    // arrived within ~100 ms, so the normal window is ample for the rest.
    bool disruptive = strncmp(cmd, "mode ", 5) == 0;
    int timeout_ms = disruptive ? MODE_ACK_TIMEOUT_MS : ACK_TIMEOUT_MS;
    if (disruptive && retries > MODE_RETRIES) retries = MODE_RETRIES;

    for (int attempt = 0; attempt <= retries; attempt++) {
        cap_reset();
        drv_uart_gnss_send((uint8_t *)cmd, strlen(cmd));

        for (int waited = 0; waited < timeout_ms; waited += ACK_POLL_MS) {
            vTaskDelay(pdMS_TO_TICKS(ACK_POLL_MS));

            gnss_ack_verdict_t verdict = cap_verdict_for(echo);
            if (verdict == GNSS_ACK_NONE) continue;   // no verdict for THIS command yet

            // A command response IS receiver liveness: during (re)config the
            // RTCM output is intentionally stopped, and the frame-based
            // watchdog must not count silence WE caused (2026-08-01: it
            // hardware-reset a healthy receiver mid-configuration).
            supervisor_note_gnss_rx();

            if (verdict == GNSS_ACK_OK) {
                ESP_LOGI(TAG, "ack: %.*s", show, cmd);
                return true;
            }
            ESP_LOGW(TAG, "rejected: %.*s", show, cmd);
            return false;                          // won't pass on retry
        }
        // "No ack" without the actual reply is unactionable: it hides whether
        // the receiver said nothing, said something we fail to parse, or said
        // it in a grammar we do not know. Dump the printable capture once per
        // command so the response format is evidence, not an assumption
        // (that assumption was wrong twice; review 2026-08-03).
        if (attempt == retries) {
            static char raw[CAP_SZ];
            size_t rn = cap_snapshot(raw);
            char esc[192];
            size_t e = 0;
            for (size_t i = 0; i < rn && e < sizeof(esc) - 5; i++) {
                unsigned char c = (unsigned char)raw[i];
                if (c >= 0x20 && c < 0x7F) {
                    esc[e++] = (char)c;
                } else {
                    e += (size_t)snprintf(esc + e, sizeof(esc) - e, "<%02X>", c);
                }
            }
            esc[e] = '\0';
            ESP_LOGW(TAG, "no ack for '%.*s'; capture (%u B): %s",
                     show, cmd, (unsigned)rn, rn ? esc : "(empty)");
        } else {
            ESP_LOGW(TAG, "no ack (attempt %d/%d): %.*s", attempt + 1, retries + 1, show, cmd);
        }
    }
    return false;
}

/* ---- hardware reset --------------------------------------------------- */

static void gnss_receiver_epoch_invalidate(void);   // defined with the latch below

static void gnss_reset_pulse(void) {
    gpio_set_direction(GPIO_GNSS_RESET, GPIO_MODE_OUTPUT);
    gpio_pullup_en(GPIO_GNSS_RESET);
    gpio_set_level(GPIO_GNSS_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(GNSS_RESET_MS));
    gpio_set_level(GPIO_GNSS_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(GNSS_RESET_MS));

    // A hardware reset erases whatever the receiver had applied, so every
    // piece of evidence about the OLD epoch must die with it. Invalidate
    // AFTER the pulse, not before: bytes that were already in flight when the
    // reset started would otherwise be stamped into the NEW epoch and make it
    // look alive before it produced anything (review 2026-08-03).
    drv_uart_gnss_flush();                  // drop pre-reset bytes still queued
    gnss_receiver_epoch_invalidate();
}

/* ---- persisted fixed base (anti reboot-re-survey) ----------------------
 * Once the IE pushes the PPP-converged precise coordinate (gnss_set_fixed_base),
 * we persist it in NVS. On every later boot/recovery config_gnss_base reads it back
 * and re-applies "mode base <lat> <lon> <h>" instead of a fresh survey-in, so a
 * power-cycle / reboot / GNSS-recover does NOT re-measure (which would broadcast a
 * meter-level 1005 until the next IE heartbeat re-pushed it). A fresh, never-fixed
 * device falls back to the provisional survey-in. */
#define GNSS_NVS_NS    "gnss"
#define GNSS_KEY_BASE  "base"
typedef struct { double lat, lon, h; } gnss_base_fix_t;

/* Receiver truth for the idempotence gate: the coordinate the RUNNING receiver
 * last ACKed a "mode base" for in THIS boot. An NVS match alone proves what we
 * WANTED, not what the receiver IS (a failed boot restore leaves NVS populated,
 * and an ACKed apply whose saveconfig failed leaves NVS stale while the
 * receiver is already correct). Keying the skip on NVS made both cases wrong:
 * the first skipped the repairing IE re-push, the second re-sent "mode base"
 * forever, stalling RTCM ~18 s each time (v1.1.2 audit 2026-08-01). */
static bool           s_base_acked_valid = false;
static bool           s_base_saved = false;
static gnss_base_fix_t s_base_acked_coord;

static void gnss_base_ack_invalidate(void) {
    s_base_acked_valid = false;
    s_base_saved = false;
}

/* The same truth latch for a survey (review 2026-11-06, G2): the site
 * revision whose "mode base time" THIS receiver epoch acknowledged. A survey
 * whose saveconfig failed counted as not applied, and the next identical
 * request re-sent MODE and restarted the 300 s survey, every time the save
 * failed. Now that request retries saveconfig alone. Cleared by a receiver
 * reset and by any fixed MODE. */
static bool     s_survey_acked = false;
static uint32_t s_survey_rev = 0;
static bool     s_survey_mode_acked_now = false;   // out-param of gnss_start_site_survey

/* Bounded retry: a receiver that keeps failing the apply must not be re-poked
 * on every heartbeat reply (a SUCCESSFUL redundant "mode base" stalls RTCM for
 * ~18 s, and even NACK storms are UART noise during configuration). One
 * attempt per cooldown for the SAME target coordinate; a NEW coordinate is
 * always applied immediately. */
#define GNSS_BASE_RETRY_COOLDOWN_US (60LL * 1000 * 1000)
static int64_t s_base_fail_us = 0;
static gnss_base_fix_t s_base_fail_coord;

#ifdef CONFIG_RTK_SITE_LEGACY
/* Relocation guard (atomic): the distance that last refused an IE
 * coordinate or dropped the fixed one, 0 while the unit is where its
 * coordinate says. Reported in the heartbeat health as moved_m. */
static uint32_t s_legacy_moved_mm = 0;
/* Relocation drop still to finish (under txn): the NVS erase and the survey
 * MODE are retried separately, and a correct coordinate accepted in between
 * cancels both (review 13.11. L2 #3). */
static bool s_drop_erase_pending = false, s_drop_mode_pending = false;
#endif

/* Everything we believe about the RUNNING receiver, dropped at once: the
 * ACKed-coordinate latch (else the idempotence gate skips a needed re-apply),
 * the failure cooldown (a fresh epoch deserves an immediate attempt), and the
 * required-RTCM freshness evidence (the new epoch must re-prove that it emits
 * the production set before anything calls it healthy). */
static void gnss_receiver_epoch_invalidate(void) {
    gnss_base_ack_invalidate();
    s_survey_acked = false;
    s_base_fail_us = 0;
    ntrip_server_msg_epoch_reset();
    gnss_site_receiver_reset();
}

// Reject Null-Island and out-of-range coords: never apply garbage as a fixed base
// (it would broadcast a wrong 1005 to every rover). Mirrors the LH-side guards.
static bool gnss_coord_valid(double lat, double lon) {
    if (lat == 0.0 && lon == 0.0) return false;
    return lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0;
}

// Height gets its own guard: a backend schema drift that maps a missing
// height to 0.0 must not program a base at exactly sea level (review
// 2026-08-03). Dead Sea to high mountain, ellipsoidal.
static bool gnss_height_valid(double h) {
    return isfinite(h) && h >= -500.0 && h <= 9000.0;
}

static bool gnss_load_fixed_base(double *lat, double *lon, double *h) {
    nvs_handle_t nh;
    if (nvs_open(GNSS_NVS_NS, NVS_READONLY, &nh) != ESP_OK) return false;
    gnss_base_fix_t b;
    size_t sz = sizeof(b);
    esp_err_t err = nvs_get_blob(nh, GNSS_KEY_BASE, &b, &sz);
    nvs_close(nh);
    if (err != ESP_OK || sz != sizeof(b) || !gnss_coord_valid(b.lat, b.lon) ||
        !gnss_height_valid(b.h)) return false;
    *lat = b.lat; *lon = b.lon; *h = b.h;
    return true;
}

// Persist failures must be VISIBLE: a silently failed save leaves NVS stale
// while the receiver runs the new coordinate, and the next boot restores the
// old one (review 2026-08-03).
static bool gnss_save_fixed_base(double lat, double lon, double h) {
    nvs_handle_t nh;
    if (nvs_open(GNSS_NVS_NS, NVS_READWRITE, &nh) != ESP_OK) {
        ESP_LOGE(TAG, "fixed-base NVS open failed; coordinate NOT persisted");
        return false;
    }
    gnss_base_fix_t b = { lat, lon, h };
    esp_err_t err = nvs_set_blob(nh, GNSS_KEY_BASE, &b, sizeof(b));
    if (err == ESP_OK) err = nvs_commit(nh);
    nvs_close(nh);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "fixed-base NVS save failed (%s); coordinate NOT persisted",
                 esp_err_to_name(err));
        uart_nmea("$PESP,RTK,GNSS,PERSISTFAIL");
        return false;
    }
    return true;
}

/* ---- reference-base configuration ------------------------------------- */

void config_gnss_base(void) {
    static const char *seq[] = {
        // NOTE: RTCM enables use the comma delimiter (rtcm<type>,com1,<sec>) - the
        // exact form the stock OnoLink firmware used on these UM980 devices, proven
        // to stream MSM7. Do NOT switch to the space form without hardware re-test.
        "unlog com1\r\n",
        "CONFIG SIGNALGROUP 2\r\n",   // enable all bands incl. Galileo E6
        // MUST precede any "mode base": the UM980 reads the mode-base height as
        // ALTITUDE (above MSL) by default and adds its internal undulation
        // before deriving the ECEF it broadcasts in the 1005. We feed the
        // ELLIPSOIDAL height from CSRS-PPP, so the default made the receiver
        // place itself N metres too high (measured 2026-08-06 on
        // RTK14335C5CA838: catalog 34.1504 m ellipsoidal -> broadcast 54.654 m,
        // delta 20.5036 m = the local undulation). The base then computed its
        // corrections for a position 20.5 m above its antenna while Lighthouse
        // served the correct ARP, so every rover inherited that offset and DJI
        // receivers rejected the fix as implausible. Zeroing the undulation
        // makes "mode base <lat> <lon> <h>" take h as ellipsoidal, which is
        // what the IE always sends.
        "CONFIG UNDULATION 0.0\r\n",
        "MASK 10.0\r\n",              // 10 deg elevation cutoff
        "rtcm1077,com1,1\r\n",        // GPS    MSM7 @1Hz
        "rtcm1087,com1,1\r\n",        // GLONASS
        "rtcm1097,com1,1\r\n",        // Galileo
        "rtcm1117,com1,1\r\n",        // QZSS   (regional)
        "rtcm1127,com1,1\r\n",        // BeiDou
        "rtcm1137,com1,1\r\n",        // NavIC  (regional)
        "rtcm1005,com1,10\r\n",       // ARP identity anchor (LH rewrites this)
        "rtcm1033,com1,10\r\n",       // receiver/antenna descriptor
        "rtcm1230,com1,10\r\n",       // GLONASS code-phase biases (cross-brand RTK)
        "BASEPOSA 1\r\n",             // actual independent SPP, even in fixed mode
        // The base MODE + saveconfig are issued AFTER this list, conditional on a
        // persisted precise coordinate (fixed) vs a fresh device (survey-in).
    };
    ESP_LOGI(TAG, "configuring UM980 reference base (ACK-gated)");
    txn_take();
    s_capturing = true;
    int seq_n = (int)(sizeof(seq) / sizeof(seq[0]));
    int ok = 0, total = seq_n + 2;   // + base-mode + saveconfig
    bool undulation_acked = false;
    for (int i = 0; i < seq_n; i++) {
        bool acked = send_cmd_acked(seq[i], ACK_RETRIES);
        if (acked) ok++;
        if (strcmp(seq[i], "CONFIG UNDULATION 0.0\r\n") == 0) undulation_acked = acked;
    }

    // Base mode: re-apply the persisted PPP-precise FIXED coordinate if we have one
    // (so a reboot/power-cycle does NOT re-survey), else provisional survey-in.
    double lat = 0, lon = 0, h = 0;
    gnss_site_desired_t desired;
    bool managed = gnss_site_get_desired(&desired);
    uint32_t revision = managed ? desired.revision : 0;
    bool fixed = managed ? desired.mode == GNSS_SITE_FIXED :
        (gnss_site_legacy_allowed() && gnss_load_fixed_base(&lat, &lon, &h));
#ifdef CONFIG_RTK_SITE_LEGACY
    // A relocation drop whose NVS erase has not gone through yet: that
    // coordinate is the place the unit left. A receiver recovery or the reset
    // button must not put it back (review 13.11. L3 #1); survey instead.
    if (!managed && s_drop_erase_pending) fixed = false;
#endif
    if (managed && fixed) {
        lat = desired.lat_e9 / 1e9;
        lon = desired.lon_e9 / 1e9;
        h = desired.h_mm / 1000.0;
    }
    bool mode_acked = false;
    gnss_site_mode_t mode = GNSS_SITE_UNKNOWN;
    if (fixed) {
        mode = GNSS_SITE_FIXED;
        char cmd[96];
        snprintf(cmd, sizeof(cmd), "mode base %.9f %.9f %.4f\r\n", lat, lon, h);
        // A MODE ACK alone cannot prove ellipsoidal height: the receiver may
        // still add its geoid separation if UNDULATION was rejected or lost.
        s_survey_acked = false;   // a fixed MODE ends any survey, acked or not
        if (undulation_acked && send_cmd_acked(cmd, ACK_RETRIES)) {
            mode_acked = true;
            ok++;
            s_base_acked_valid = true;
            s_base_acked_coord = (gnss_base_fix_t){ lat, lon, h };
            ESP_LOGI(TAG, "restored FIXED base from NVS: %.9f %.9f %.4f", lat, lon, h);
            uart_nmea("$PESP,RTK,GNSS,BASEMODE,FIXED");
        } else {
            // Receiver base mode is now UNKNOWN (it may still be surveying or
            // hold a stale mode). The acked-coordinate record stays invalid so
            // the next IE coordinate push re-applies instead of being
            // idempotence-skipped on the NVS match (v1.1.2 audit 2026-08-01).
            // The old code logged "restored" and BASEMODE,FIXED even on a NACK.
            s_base_acked_valid = false;
            ESP_LOGE(TAG, "FIXED base restore NOT acked; receiver base mode unknown");
            uart_nmea("$PESP,RTK,GNSS,BASEMODE,RESTOREFAIL");
        }
    } else if (managed || gnss_site_legacy_allowed()) {
        mode = GNSS_SITE_SURVEY;
        // Same honesty as the FIXED branch above: only report SURVEYIN when the
        // receiver actually acked it (v1.1.2 audit 2026-08-01). Either way the
        // receiver is NOT on a fixed coordinate anymore: the truth latch must
        // fall, or a later identical IE push is wrongly skipped (review
        // 2026-08-03, P0).
        gnss_base_ack_invalidate();
        // The second argument is SAVED-POSITION REUSE DISTANCE, not sigma.
        // Zero forces a fresh survey rather than resurrecting factory position.
        s_survey_acked = false;
        if (undulation_acked && send_cmd_acked("mode base time 300 0\r\n", ACK_RETRIES)) {
            mode_acked = true;
            ok++;
            s_survey_acked = true;
            s_survey_rev = revision;
            ESP_LOGI(TAG, "no persisted fixed base -> provisional survey-in");
            uart_nmea("$PESP,RTK,GNSS,BASEMODE,SURVEYIN");
        } else {
            ESP_LOGE(TAG, "survey-in NOT acked; receiver base mode unknown");
            uart_nmea("$PESP,RTK,GNSS,BASEMODE,SURVEYINFAIL");
        }
    }
    bool saved = send_cmd_acked("saveconfig\r\n", ACK_RETRIES);
    if (saved) ok++;
    s_base_saved = mode_acked && fixed && saved;
#ifdef CONFIG_RTK_SITE_LEGACY
    // Legacy: no revision to acknowledge, only the running receiver mode. A
    // failed SAVE must not leave it UNKNOWN: the later SAVE-only retry in
    // gnss_set_fixed_base never reports again, and observation supervision
    // stayed off for the whole boot (review 13.11., P2).
    gnss_site_configured(mode, lat, lon, revision, mode_acked);
#else
    gnss_site_configured(mode, lat, lon, revision, mode_acked && saved);
#endif

    s_capturing = false;
    ESP_LOGI(TAG, "UM980 base config: %d/%d commands acked", ok, total);
    uart_nmea("$PESP,RTK,GNSS,CONFIG,%d,%d", ok, total);
    txn_give();
}

bool gnss_set_fixed_base(double lat_deg, double lon_deg, double height_m) {
    // Never apply a garbage coordinate as the fixed base (it would be broadcast as
    // the 1005 to every rover). Reject Null-Island / out-of-range outright.
    if (!gnss_coord_valid(lat_deg, lon_deg) || !gnss_height_valid(height_m)) {
        ESP_LOGE(TAG, "rejecting invalid fixed base %.9f %.9f h=%.4f",
                 lat_deg, lon_deg, height_m);
        uart_nmea("$PESP,RTK,GNSS,FIXEDBASE,0");
        return false;
    }

    txn_take();

    // Idempotence gate: the IE re-pushes the coordinate on heartbeat replies,
    // and boot already applied the NVS-persisted one. Re-sending "mode base"
    // for an UNCHANGED coordinate makes the UM980 stop its RTCM output for
    // many seconds for zero gain (measured 2026-08-01: an 18+ s stall right
    // after boot that tripped the liveness watchdog into a needless receiver
    // reset). Epsilons: ~0.1 mm in position, 0.5 mm in height.
    //
    // The gate keys on RECEIVER truth (the coordinate this boot got an ACK for),
    // NOT on NVS (v1.1.2 audit 2026-08-01). NVS only records what we intended:
    // a NACKed boot restore leaves it populated while the receiver mode is
    // unknown, and an ACKed apply whose saveconfig failed leaves it stale while
    // the receiver is already correct. Keying on NVS made the first case skip
    // the repairing re-push and the second re-send "mode base" every cooldown
    // window forever, stalling RTCM ~18 s each time.
    if (s_base_acked_valid &&
        fabs(s_base_acked_coord.lat - lat_deg) < 1e-9 &&
        fabs(s_base_acked_coord.lon - lon_deg) < 1e-9 &&
        fabs(s_base_acked_coord.h - height_m) < 5e-4) {
        // Debug level: with the once-per-boot latch in provisioning removed,
        // this is the steady-state path on every heartbeat reply.
        ESP_LOGD(TAG, "fixed base unchanged and receiver-acked, skipping re-apply");
        // NVS may still be stale if a previous saveconfig failed; refresh it so
        // the next boot restores this coordinate instead of re-surveying. Cheap
        // (NVS only rewrites on a real change) and touches no UART. Done BEFORE
        // the SAVE retry: the receiver runs on this ACKed coordinate whatever
        // its own SAVE does, and an early return on a failed retry left the
        // MCU restoring the old one at the next boot (review 13.11. R3, P1;
        // 1.1.9 refreshed NVS here unconditionally).
        double nv_lat, nv_lon, nv_h;
        if (!gnss_load_fixed_base(&nv_lat, &nv_lon, &nv_h) ||
            fabs(nv_lat - lat_deg) >= 1e-9 ||
            fabs(nv_lon - lon_deg) >= 1e-9 ||
            fabs(nv_h - height_m) >= 5e-4) {
            gnss_save_fixed_base(lat_deg, lon_deg, height_m);
        }
        // A previous MODE ACK with failed SAVE proves only volatile state.
        // Retry SAVE alone; do not falsely acknowledge durable application or
        // interrupt RTCM with another unchanged MODE command.
        if (!s_base_saved) {
            s_capturing = true;
            s_base_saved = send_cmd_acked("saveconfig\r\n", ACK_RETRIES);
            s_capturing = false;
            if (!s_base_saved) { txn_give(); return false; }
        }
        txn_give();
        return true;
    }

    // Bounded retry: one attempt per cooldown for the SAME coordinate that
    // just failed; a genuinely new coordinate always goes through at once.
    if (s_base_fail_us != 0 &&
        fabs(s_base_fail_coord.lat - lat_deg) < 1e-9 &&
        fabs(s_base_fail_coord.lon - lon_deg) < 1e-9 &&
        fabs(s_base_fail_coord.h - height_m) < 5e-4 &&
        (esp_timer_get_time() - s_base_fail_us) < GNSS_BASE_RETRY_COOLDOWN_US) {
        txn_give();
        return false;
    }

#ifdef CONFIG_RTK_SITE_LEGACY
    // Relocation guard (review 13.11. L1, R1). The legacy IE pushes the
    // station's coordinate by identity, wherever the unit is now: a unit
    // that got its coordinate at our office, or in the customer's garden
    // before it went on the roof, would broadcast that place. Beyond 300 m
    // the UM980 then suppresses RTCM, closer it sends wrong corrections. A NEW
    // coordinate is applied only when the receiver's own position (median of
    // a valid BASEPOS window, typically 1 to 3 m) agrees with it within
    // max(10 m, 5 sigma). No window yet: wait, the next reply asks again.
    {
        static gnss_site_agreement_t s_last_verdict = GNSS_SITE_AGREEMENT_AGREE;
        uint32_t away_mm = 0;
        gnss_site_agreement_t verdict = gnss_site_legacy_agreement(lat_deg, lon_deg, &away_mm);
        if (verdict != s_last_verdict) {
            if (verdict == GNSS_SITE_AGREEMENT_DISAGREE) {
                ESP_LOGE(TAG, "IE coordinate %.9f %.9f is %u m from the receiver's own position: "
                         "not applied (unit moved?)", lat_deg, lon_deg, (unsigned)(away_mm / 1000));
                uart_nmea("$PESP,RTK,GNSS,FIXEDBASE,REFUSED,%u", (unsigned)(away_mm / 1000));
            } else if (verdict == GNSS_SITE_AGREEMENT_UNKNOWN) {
                ESP_LOGI(TAG, "IE coordinate waits for a valid own position");
            }
            s_last_verdict = verdict;
        }
        if (verdict == GNSS_SITE_AGREEMENT_DISAGREE)
            __atomic_store_n(&s_legacy_moved_mm, away_mm ? away_mm : 1, __ATOMIC_RELAXED);
        if (verdict != GNSS_SITE_AGREEMENT_AGREE) {
            txn_give();
            return false;
        }
        __atomic_store_n(&s_legacy_moved_mm, 0, __ATOMIC_RELAXED);
    }
#endif

    char cmd[96];
    // UM980: "mode base <lat> <lon> <height>" - degrees / metres, ITRF2020 from the IE
    snprintf(cmd, sizeof(cmd), "mode base %.9f %.9f %.4f\r\n", lat_deg, lon_deg, height_m);

    s_capturing = true;
    // Zero the undulation IMMEDIATELY before every mode base, not only in
    // config_gnss_base: this path runs from the IE heartbeat and must be
    // correct on a receiver that never saw the boot sequence (mid-life OTA, or
    // a saveconfig that failed). Idempotent, one command, and the idempotence
    // gate above keeps it off the UART in steady state. Without it the height
    // is taken as MSL and the base sits N metres too high; see config_gnss_base.
    bool undulation_acked = send_cmd_acked("CONFIG UNDULATION 0.0\r\n", ACK_RETRIES);
    if (undulation_acked) s_survey_acked = false;   // MODE goes out: no survey any more
    bool acked = undulation_acked && send_cmd_acked(cmd, ACK_RETRIES);
    // Persist the moment the receiver runs it, BEFORE its SAVECONFIG (review
    // 13.11. L1, R5): saved only after a successful SAVECONFIG, a power cut in
    // between, or a failed SAVECONFIG plus a cut before the next identical
    // reply, restored the previous coordinate (or a survey) at the next boot.
    // Not before the MODE: a rejected MODE must never become the coordinate
    // the next boot restores. A failed write does not stop the apply; the
    // unchanged-coordinate branch above rewrites NVS on the next reply.
    if (acked) gnss_save_fixed_base(lat_deg, lon_deg, height_m);
    bool ok = acked;
    if (ok) ok = send_cmd_acked("saveconfig\r\n", ACK_RETRIES);
    s_capturing = false;

    // The mode-base ACK alone settles receiver state, so record it even when
    // saveconfig failed: the receiver IS on this coordinate and must not be
    // re-poked. saveconfig only governs persistence across a receiver power
    // cycle; the skip path above refreshes NVS without touching the UART.
    if (acked) {
        s_base_acked_valid = true;
        s_base_acked_coord = (gnss_base_fix_t){ lat_deg, lon_deg, height_m };
#ifdef CONFIG_RTK_SITE_LEGACY
        // Legacy IE: this push is the only fixed-mode command, so it must also
        // tell the site runtime, or the observation check keeps the survey
        // clock and expects RTCM through the ~18 s stall of the new MODE.
        gnss_site_configured(GNSS_SITE_FIXED, lat_deg, lon_deg, 0, true);
        // A coordinate that passed the agreement gate is where the unit is
        // now: an unfinished drop of the old one must not erase it.
        s_drop_erase_pending = s_drop_mode_pending = false;
#endif
    } else {
        s_base_acked_valid = false;
    }
    s_base_saved = ok;
    if (!ok) {
        s_base_fail_us = esp_timer_get_time();
        s_base_fail_coord = (gnss_base_fix_t){ lat_deg, lon_deg, height_m };
    } else {
        s_base_fail_us = 0;
    }

    ESP_LOGI(TAG, "fixed base %.9f %.9f %.4f -> %s", lat_deg, lon_deg, height_m, ok ? "OK" : "FAILED");
    uart_nmea("$PESP,RTK,GNSS,FIXEDBASE,%d", ok ? 1 : 0);
    txn_give();
    return ok;
}

bool gnss_start_site_survey(void) {
    txn_take();
    s_capturing = true;
    gnss_base_ack_invalidate();
    s_base_fail_us = 0;
    ntrip_server_msg_epoch_reset();
    s_survey_acked = false;
    bool mode_acked = send_cmd_acked("CONFIG UNDULATION 0.0\r\n", ACK_RETRIES) &&
                      send_cmd_acked("mode base time 300 0\r\n", ACK_RETRIES);
    s_survey_mode_acked_now = mode_acked;
    bool ok = mode_acked && send_cmd_acked("saveconfig\r\n", ACK_RETRIES);
    s_capturing = false;
    txn_give();
    return ok;
}

#ifdef CONFIG_RTK_SITE_LEGACY
uint32_t gnss_legacy_moved_m(void) {
    return __atomic_load_n(&s_legacy_moved_mm, __ATOMIC_RELAXED) / 1000;
}

static bool gnss_forget_fixed_base(void) {
    nvs_handle_t nh;
    if (nvs_open(GNSS_NVS_NS, NVS_READWRITE, &nh) != ESP_OK) return false;
    esp_err_t err = nvs_erase_key(nh, GNSS_KEY_BASE);
    if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;
    if (err == ESP_OK) err = nvs_commit(nh);
    nvs_close(nh);
    return err == ESP_OK;
}

/* Called by the provisioning task every poll (about 30 s), IE reachable or
 * not. Two jobs, both without the IE:
 * 1. Relocation (review 13.11. L1, R1): fixed on a coordinate the receiver
 *    has left for five minutes -> forget it, survey-in here. Forgotten in
 *    NVS first, so a power cut after this point cannot bring it back. The
 *    IE keeps pushing it; gnss_set_fixed_base refuses it until the IE's own
 *    re-survey (its move detection compares our survey 1005) catches up.
 * 2. Unknown mode (review 13.11. L1, R4): a boot or recovery whose MODE
 *    went unacknowledged leaves the mode UNKNOWN, which switches observation
 *    supervision and log repair off while BASEPOS keeps the receiver
 *    "alive". Reconfigure after 5 min, backing off to an hour. */
void gnss_legacy_tick(void) {
    static int64_t s_drop_try_us = 0;
    static int64_t s_unknown_since_us = 0, s_unknown_wait_us = 0;
    int64_t now = esp_timer_get_time();
    uint32_t mm = 0;
    gnss_site_status_t site;

    bool retry_due = (s_drop_erase_pending || s_drop_mode_pending) &&
        now - s_drop_try_us >= GNSS_BASE_RETRY_COOLDOWN_US;
    if (gnss_site_legacy_relocated(&mm) || retry_due) {
        txn_take();
        if (gnss_site_legacy_relocated(&mm)) {   // recheck under the receiver lock
            ESP_LOGE(TAG, "receiver %u m away from its reference for %u s: unit moved, "
                     "coordinate forgotten, survey-in here", (unsigned)(mm / 1000),
                     (unsigned)(GNSS_SITE_RELOCATE_MS / 1000));
            uart_nmea("$PESP,RTK,GNSS,RELOCATED,%u", (unsigned)(mm / 1000));
            __atomic_store_n(&s_legacy_moved_mm, mm ? mm : 1, __ATOMIC_RELAXED);
            s_drop_erase_pending = s_drop_mode_pending = true;
        }
        if (s_drop_erase_pending || s_drop_mode_pending) {   // a correct coordinate cancels
            s_drop_try_us = now;
            // Forgotten first: a power cut after this cannot restore the old
            // place. Retried alone: no UART, no survey restart.
            if (s_drop_erase_pending) {
                s_drop_erase_pending = !gnss_forget_fixed_base();
                if (s_drop_erase_pending) uart_nmea("$PESP,RTK,GNSS,PERSISTFAIL");
            }
            // A survey that is running (acked now, or restored by a receiver
            // recovery in between) is the goal; MODE again would restart it.
            gnss_site_status(&site);
            if (s_drop_mode_pending && site.mode == GNSS_SITE_SURVEY &&
                !gnss_site_legacy_relocated(&mm)) {
                s_drop_mode_pending = false;
            }
            if (s_drop_mode_pending) {
                s_survey_mode_acked_now = false;
                gnss_start_site_survey();
                bool acked = s_survey_mode_acked_now;
                if (acked) {
                    s_survey_acked = true;
                    s_survey_rev = 0;
                }
                s_base_saved = false;
                gnss_site_configured(GNSS_SITE_SURVEY, 0, 0, 0, acked);
                // A NACK leaves the mode UNKNOWN, which also ends the run that
                // triggered this: the retry is driven from here, every 60 s.
                s_drop_mode_pending = !acked;
            }
        }
        txn_give();
    }

    gnss_site_status(&site);
    supervisor_health_t health;
    supervisor_health(&health);
    if (site.mode != GNSS_SITE_UNKNOWN) s_unknown_wait_us = 0;   // recovered: next episode starts at 5 min
    if (site.mode != GNSS_SITE_UNKNOWN || !health.gnss_ok || s_drop_mode_pending) {
        s_unknown_since_us = 0;     // a known mode, or a receiver the supervisor owns
        return;
    }
    if (s_unknown_since_us == 0) {
        s_unknown_since_us = now;
        if (s_unknown_wait_us == 0) s_unknown_wait_us = 5LL * 60 * 1000 * 1000;
        return;
    }
    if (now - s_unknown_since_us < s_unknown_wait_us) return;
    txn_take();
    gnss_site_status(&site);
    if (site.mode == GNSS_SITE_UNKNOWN) {
        ESP_LOGW(TAG, "receiver mode unknown for %u s: reset + reconfigure",
                 (unsigned)((now - s_unknown_since_us) / 1000000));
        uart_nmea("$PESP,RTK,GNSS,MODEUNKNOWN");
        gnss_reset_pulse();
        config_gnss_base();
        s_unknown_wait_us *= 2;
        if (s_unknown_wait_us > 60LL * 60 * 1000 * 1000) s_unknown_wait_us = 60LL * 60 * 1000 * 1000;
    }
    s_unknown_since_us = 0;
    txn_give();
}
#endif

bool gnss_apply_site_revision(const gnss_site_desired_t *desired) {
    if (!gnss_site_desired_valid(desired)) return false;
    txn_take();
    bool applied;
    if (desired->mode == GNSS_SITE_FIXED) {
        applied = gnss_set_fixed_base(desired->lat_e9 / 1e9, desired->lon_e9 / 1e9,
                                      desired->h_mm / 1000.0);
    } else if (s_survey_acked && s_survey_rev == desired->revision) {
        /* This survey is already running on this receiver epoch; only its
         * saveconfig failed. Retry that alone: MODE again would restart it. */
        s_capturing = true;
        applied = send_cmd_acked("saveconfig\r\n", ACK_RETRIES);
        s_capturing = false;
    } else {
        s_survey_mode_acked_now = false;
        applied = gnss_start_site_survey();
        if (s_survey_mode_acked_now) {
            s_survey_acked = true;
            s_survey_rev = desired->revision;
        }
    }
    /* Keep the receiver transaction lock through the revision ACK, so a
     * watchdog reset cannot intervene between MODE completion and this ACK. */
    gnss_site_configured((gnss_site_mode_t)desired->mode, desired->lat_e9 / 1e9,
                         desired->lon_e9 / 1e9, desired->revision, applied);
    txn_give();
    return applied;
}

void gnss_recover(void) {
    txn_take();
    /* Supervisor may have decided just before an IE mode transaction or a
     * fresh sample. Recheck after taking the receiver lock so an old fault
     * cannot reset a newly ACKed 300-second survey/intentional held state. */
    supervisor_health_t health;
    supervisor_health(&health);
    if (health.gnss_ok) { txn_give(); return; }
    ESP_LOGW(TAG, "GNSS recover: hardware reset + reconfigure");
    uart_nmea("$PESP,RTK,GNSS,RECOVER");
    gnss_reset_pulse();
    config_gnss_base();
    txn_give();
}

/* Re-enable required RTCM messages the receiver is not producing, without
 * MODE and without a reset (review 2026-11-06, G4). A log enable that failed
 * during configuration (a transient UART loss) was never retried: advancing
 * MSM plus BASEPOS kept the station "healthy" without, say, its 1005. Called
 * by the supervisor once the stream is expected and a required type stayed
 * stale. Each command is a plain log enable: it does not pause the stream. */
void gnss_repair_required_logs(void) {
    if (s_txn == NULL) return;
    txn_take();
    ntrip_msg_freshness_t fresh;
    ntrip_server_msg_freshness(&fresh);   // re-read under the lock
    if (!fresh.all_fresh && gnss_site_log_repair_expected()) {
        s_capturing = true;
        for (int i = 0; i < NTRIP_REQUIRED_MSG_COUNT; i++) {
            if (fresh.fresh[i]) continue;
            unsigned type = fresh.type[i];
            // same rates as config_gnss_base: MSM 1 Hz, 1005/1033 every 10 s
            unsigned period = (type == 1005 || type == 1033) ? 10 : 1;
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "rtcm%u,com1,%u\r\n", type, period);
            bool ok = send_cmd_acked(cmd, ACK_RETRIES);
            ESP_LOGW(TAG, "required RTCM %u not produced: re-enabled -> %s", type,
                     ok ? "ack" : "FAILED");
            uart_nmea("$PESP,RTK,GNSS,LOGREPAIR,%u,%d", type, ok ? 1 : 0);
        }
        s_capturing = false;
    }
    txn_give();
}

void gnss_after_external_reset(void) {
    /* A press before gnss_init() needs no action: gnss_init() resets and
     * configures the receiver anyway. Running config here without the lock
     * raced the boot configuration on the shared capture (G5). A press
     * during gnss_init() waits on the lock and reconfigures after it. */
    if (s_txn == NULL) {
        ESP_LOGI(TAG, "GNSS reset by the button before GNSS init: boot config covers it");
        return;
    }
    txn_take();
    ESP_LOGW(TAG, "GNSS reset by the button: reconfigure");
    uart_nmea("$PESP,RTK,GNSS,BUTTONRESET");
    /* The button pulled RESET_N low in hardware (K_EVK_V4 D22) and C58 lets it
     * go a moment after release. Give the UM980 the same settle time our own
     * pulse gives it, then treat it exactly like our reset: old evidence dies,
     * config goes back on. No health gate here, unlike gnss_recover(): the
     * receiver HAS lost its config even though bytes may still look fresh. */
    vTaskDelay(pdMS_TO_TICKS(2 * GNSS_RESET_MS));
    drv_uart_gnss_flush();
    gnss_receiver_epoch_invalidate();
    config_gnss_base();
    txn_give();
}

/* Query + log the UM980 firmware version (VERSIONA). The #VERSIONA response
 * carries the model + firmware build (e.g. ...,"UM980",...,"R4.10Build11833",...)
 * which we need for inventory + partner config questions (which UM980 fw is on
 * the station). VERSIONA is a one-shot query, so it still answers after the
 * "unlog com1" in config_gnss_base. */
/* body starts just past "#VERSIONA,". The record is printable ASCII up to
 * '*', then exactly 8 hex digits (CRC32) and the line end, all on one line.
 * Returns the offset past the CRC, 0 while incomplete or malformed. */
static int version_record_end(const char *hay, size_t n, size_t body) {
    size_t i = body;
    while (i < n && hay[i] != '*') {
        if (hay[i] < 0x20 || hay[i] > 0x7e) return 0;   // CR/LF or binary: not one record
        i++;
    }
    if (i + 10 > n) return 0;                            // CRC or line end not here yet
    for (size_t k = i + 1; k < i + 9; k++) {
        char c = hay[k];
        bool hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
        if (!hex) return 0;
    }
    if (hay[i + 9] != '\r' && hay[i + 9] != '\n') return 0;
    return (int)(i + 9);
}

/* Only a complete "#VERSIONA,...*<8 hex>" record counts: the command echo
 * ("$command,VERSIONA,response: OK") and the RTCM stream share the capture
 * buffer, and neither is a version answer. */
size_t gnss_query_version(char *out, size_t out_sz) {
    static char tmp[CAP_SZ];
    int start = -1, end = -1;
    txn_take();
    s_capturing = true;
    cap_reset();
    drv_uart_gnss_send((uint8_t *)"VERSIONA\r\n", 10);
    for (int waited = 0; waited < 2000; waited += ACK_POLL_MS) {
        vTaskDelay(pdMS_TO_TICKS(ACK_POLL_MS));
        size_t n = cap_snapshot(tmp);
        for (int p = cap_find_from(tmp, n, 0, "#VERSIONA,"); p >= 0;
             p = cap_find_from(tmp, n, (size_t)p, "#VERSIONA,")) {
            int rec_end = version_record_end(tmp, n, (size_t)p);
            if (rec_end > 0) {
                start = p - 10;     // back to the '#'
                end = rec_end;      // past the CRC32
                break;
            }
        }
        if (start >= 0) break;
    }
    s_capturing = false;
    txn_give();
    if (out && out_sz) out[0] = '\0';
    if (start < 0) return 0;
    size_t len = (size_t)(end - start);
    if (out && out_sz) {
        size_t c = len < out_sz - 1 ? len : out_sz - 1;
        for (size_t i = 0; i < c; i++) {
            char ch = tmp[start + i];
            out[i] = (ch >= 0x20 && ch < 0x7f) ? ch : ' ';
        }
        out[c] = '\0';
    }
    return len;
}

void gnss_log_version(void) {
    static char ver[CAP_SZ];
    size_t n = gnss_query_version(ver, sizeof(ver));
    ESP_LOGI(TAG, "UM980 VERSIONA: %s", n ? ver : "(no response)");
    uart_nmea("$PESP,RTK,GNSS,VER,%s", n ? ver : "none");
}

void gnss_test_reset(void) {
    txn_take();
    ESP_LOGW(TAG, "GNSS reset requested by the board test page");
    uart_nmea("$PESP,RTK,GNSS,TESTRESET");
    gnss_reset_pulse();
    config_gnss_base();
    txn_give();
}

void gnss_init(void) {
    // The transaction lock exists before anything can race for the receiver
    // (supervisor and provisioning start later, but order must not matter).
    s_txn = xSemaphoreCreateRecursiveMutexStatic(&s_txn_buf);
    gnss_site_runtime_init();

    // No event-bus registration: uart_task calls gnss_ingest_uart() directly,
    // so ACK capture cannot be dropped by a saturated event queue.

    txn_take();
    gnss_reset_pulse();
    config_gnss_base();
    gnss_log_version();     // log the UM980 firmware (inventory + partner config)
    txn_give();
}
