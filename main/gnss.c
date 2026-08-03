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
#include <esp_log.h>
#include <esp_event.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include "nvs.h"

#include "gnss.h"
#include "gnss_ack.h"
#include "uart.h"
#include "supervisor.h"
#include "interface/ntrip.h"

#define TAG "GNSS"

#define GPIO_GNSS_RESET    GPIO_NUM_22
#define GNSS_RESET_MS      500
#define ACK_TIMEOUT_MS     900   // UM980 usually replies within ~100-300 ms
#define ACK_POLL_MS        20
#define ACK_RETRIES        3

/* ---- UM980 command response capture -----------------------------------
 * A registered UART_EVENT_READ handler appends the receiver's bytes to a small
 * buffer while we are configuring. Coexists with the NTRIP forward handler (the
 * ESP event loop supports multiple handlers on the same event).
 */
#define CAP_SZ 1024
static char s_cap[CAP_SZ];
static volatile size_t s_cap_len = 0;
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
    if (!s_capturing || data == NULL || len == 0) return;
    portENTER_CRITICAL(&s_cap_mux);
    if (s_cap_len + len >= CAP_SZ) s_cap_len = 0;   // keep the newest
    for (size_t i = 0; i < len && s_cap_len < CAP_SZ - 1; i++) s_cap[s_cap_len++] = (char)data[i];
    s_cap[s_cap_len] = '\0';
    portEXIT_CRITICAL(&s_cap_mux);
}

static void cap_reset(void) {
    portENTER_CRITICAL(&s_cap_mux);
    s_cap_len = 0; s_cap[0] = '\0';
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
    size_t n = s_cap_len;
    memcpy(dst, s_cap, n);
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

    for (int attempt = 0; attempt <= retries; attempt++) {
        cap_reset();
        drv_uart_gnss_send((uint8_t *)cmd, strlen(cmd));

        for (int waited = 0; waited < ACK_TIMEOUT_MS; waited += ACK_POLL_MS) {
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
    // A hardware reset erases whatever the receiver had applied. Every piece
    // of evidence about the OLD epoch must die with it (review 2026-08-03).
    gnss_receiver_epoch_invalidate();
    gpio_set_direction(GPIO_GNSS_RESET, GPIO_MODE_OUTPUT);
    gpio_pullup_en(GPIO_GNSS_RESET);
    gpio_set_level(GPIO_GNSS_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(GNSS_RESET_MS));
    gpio_set_level(GPIO_GNSS_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(GNSS_RESET_MS));
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
static gnss_base_fix_t s_base_acked_coord;

static void gnss_base_ack_invalidate(void) { s_base_acked_valid = false; }

/* Bounded retry: a receiver that keeps failing the apply must not be re-poked
 * on every heartbeat reply (a SUCCESSFUL redundant "mode base" stalls RTCM for
 * ~18 s, and even NACK storms are UART noise during configuration). One
 * attempt per cooldown for the SAME target coordinate; a NEW coordinate is
 * always applied immediately. */
#define GNSS_BASE_RETRY_COOLDOWN_US (60LL * 1000 * 1000)
static int64_t s_base_fail_us = 0;
static gnss_base_fix_t s_base_fail_coord;

/* Everything we believe about the RUNNING receiver, dropped at once: the
 * ACKed-coordinate latch (else the idempotence gate skips a needed re-apply),
 * the failure cooldown (a fresh epoch deserves an immediate attempt), and the
 * required-RTCM freshness evidence (the new epoch must re-prove that it emits
 * the production set before anything calls it healthy). */
static void gnss_receiver_epoch_invalidate(void) {
    gnss_base_ack_invalidate();
    s_base_fail_us = 0;
    ntrip_server_msg_epoch_reset();
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
        // The base MODE + saveconfig are issued AFTER this list, conditional on a
        // persisted precise coordinate (fixed) vs a fresh device (survey-in).
    };
    ESP_LOGI(TAG, "configuring UM980 reference base (ACK-gated)");
    txn_take();
    s_capturing = true;
    int seq_n = (int)(sizeof(seq) / sizeof(seq[0]));
    int ok = 0, total = seq_n + 2;   // + base-mode + saveconfig
    for (int i = 0; i < seq_n; i++) {
        if (send_cmd_acked(seq[i], ACK_RETRIES)) ok++;
    }

    // Base mode: re-apply the persisted PPP-precise FIXED coordinate if we have one
    // (so a reboot/power-cycle does NOT re-survey), else provisional survey-in.
    double lat, lon, h;
    if (gnss_load_fixed_base(&lat, &lon, &h)) {
        char cmd[96];
        snprintf(cmd, sizeof(cmd), "mode base %.9f %.9f %.4f\r\n", lat, lon, h);
        if (send_cmd_acked(cmd, ACK_RETRIES)) {
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
    } else {
        // Same honesty as the FIXED branch above: only report SURVEYIN when the
        // receiver actually acked it (v1.1.2 audit 2026-08-01). Either way the
        // receiver is NOT on a fixed coordinate anymore: the truth latch must
        // fall, or a later identical IE push is wrongly skipped (review
        // 2026-08-03, P0).
        gnss_base_ack_invalidate();
        if (send_cmd_acked("mode base time 300 1.5\r\n", ACK_RETRIES)) {  // VERIFY syntax on first hw
            ok++;
            ESP_LOGI(TAG, "no persisted fixed base -> provisional survey-in");
            uart_nmea("$PESP,RTK,GNSS,BASEMODE,SURVEYIN");
        } else {
            ESP_LOGE(TAG, "survey-in NOT acked; receiver base mode unknown");
            uart_nmea("$PESP,RTK,GNSS,BASEMODE,SURVEYINFAIL");
        }
    }
    if (send_cmd_acked("saveconfig\r\n", ACK_RETRIES)) ok++;

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
        // (NVS only rewrites on a real change) and touches no UART.
        double nv_lat, nv_lon, nv_h;
        if (!gnss_load_fixed_base(&nv_lat, &nv_lon, &nv_h) ||
            fabs(nv_lat - lat_deg) >= 1e-9 ||
            fabs(nv_lon - lon_deg) >= 1e-9 ||
            fabs(nv_h - height_m) >= 5e-4) {
            gnss_save_fixed_base(lat_deg, lon_deg, height_m);
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

    char cmd[96];
    // UM980: "mode base <lat> <lon> <height>" - degrees / metres, ITRF2020 from the IE
    snprintf(cmd, sizeof(cmd), "mode base %.9f %.9f %.4f\r\n", lat_deg, lon_deg, height_m);

    s_capturing = true;
    bool acked = send_cmd_acked(cmd, ACK_RETRIES);
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
    } else {
        s_base_acked_valid = false;
    }
    if (!ok) {
        s_base_fail_us = esp_timer_get_time();
        s_base_fail_coord = (gnss_base_fix_t){ lat_deg, lon_deg, height_m };
    } else {
        s_base_fail_us = 0;
    }

    // Persist so a reboot/recovery re-applies this fixed coord instead of re-surveying.
    if (ok) gnss_save_fixed_base(lat_deg, lon_deg, height_m);

    ESP_LOGI(TAG, "fixed base %.9f %.9f %.4f -> %s", lat_deg, lon_deg, height_m, ok ? "OK" : "FAILED");
    uart_nmea("$PESP,RTK,GNSS,FIXEDBASE,%d", ok ? 1 : 0);
    txn_give();
    return ok;
}

void gnss_recover(void) {
    ESP_LOGW(TAG, "GNSS recover: hardware reset + reconfigure");
    uart_nmea("$PESP,RTK,GNSS,RECOVER");
    txn_take();
    gnss_reset_pulse();
    config_gnss_base();
    txn_give();
}

/* Query + log the UM980 firmware version (VERSIONA). The #VERSIONA response
 * carries the model + firmware build (e.g. ...,"UM980",...,"R4.10Build11833",...)
 * which we need for inventory + partner config questions (which UM980 fw is on
 * the station). VERSIONA is a one-shot query, so it still answers after the
 * "unlog com1" in config_gnss_base. */
void gnss_log_version(void) {
    txn_take();
    s_capturing = true;
    cap_reset();
    drv_uart_gnss_send((uint8_t *)"VERSIONA\r\n", 10);
    for (int waited = 0; waited < 2000; waited += ACK_POLL_MS) {
        vTaskDelay(pdMS_TO_TICKS(ACK_POLL_MS));
        if (cap_contains("VERSION")) break;     // got the #VERSIONA line
    }
    static char tmp[CAP_SZ];
    portENTER_CRITICAL(&s_cap_mux);
    size_t n = s_cap_len;
    memcpy(tmp, s_cap, n + 1);
    portEXIT_CRITICAL(&s_cap_mux);
    s_capturing = false;
    for (size_t i = 0; i < n; i++) if (tmp[i] == '\r' || tmp[i] == '\n') tmp[i] = ' ';
    ESP_LOGI(TAG, "UM980 VERSIONA: %s", n ? tmp : "(no response)");
    uart_nmea("$PESP,RTK,GNSS,VER,%s", n ? tmp : "none");
    txn_give();
}

void gnss_init(void) {
    // The transaction lock exists before anything can race for the receiver
    // (supervisor and provisioning start later, but order must not matter).
    s_txn = xSemaphoreCreateRecursiveMutex();
    if (!s_txn) ESP_LOGE(TAG, "gnss txn mutex creation failed; transactions unserialized");

    // No event-bus registration: uart_task calls gnss_ingest_uart() directly,
    // so ACK capture cannot be dropped by a saturated event queue.

    txn_take();
    gnss_reset_pulse();
    config_gnss_base();
    gnss_log_version();     // log the UM980 firmware (inventory + partner config)
    txn_give();
}
