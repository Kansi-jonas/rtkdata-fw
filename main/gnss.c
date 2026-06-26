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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_event.h>
#include <driver/gpio.h>
#include "nvs.h"

#include "gnss.h"
#include "uart.h"
#include "supervisor.h"

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

static void gnss_uart_capture(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg; (void)base;
    supervisor_note_gnss_rx();              // any GNSS byte batch = the receiver is alive
    if (!s_capturing || data == NULL) return;
    int len = (int)id;                          // uart_task posts len as the event id
    const uint8_t *d = (const uint8_t *)data;
    portENTER_CRITICAL(&s_cap_mux);
    if (s_cap_len + (size_t)len >= CAP_SZ) s_cap_len = 0;   // keep the newest
    for (int i = 0; i < len && s_cap_len < CAP_SZ - 1; i++) s_cap[s_cap_len++] = (char)d[i];
    s_cap[s_cap_len] = '\0';
    portEXIT_CRITICAL(&s_cap_mux);
}

static void cap_reset(void) {
    portENTER_CRITICAL(&s_cap_mux);
    s_cap_len = 0; s_cap[0] = '\0';
    portEXIT_CRITICAL(&s_cap_mux);
}

static bool cap_contains(const char *needle) {
    static char tmp[CAP_SZ];
    portENTER_CRITICAL(&s_cap_mux);
    size_t n = s_cap_len;
    memcpy(tmp, s_cap, n + 1);
    portEXIT_CRITICAL(&s_cap_mux);
    return strstr(tmp, needle) != NULL;
}

/* ---- ACK-gated command send ------------------------------------------- */

static bool send_cmd_acked(const char *cmd, int retries) {
    int show = (int)strlen(cmd);
    while (show > 0 && (cmd[show - 1] == '\r' || cmd[show - 1] == '\n')) show--;  // trim CRLF for logs

    for (int attempt = 0; attempt <= retries; attempt++) {
        cap_reset();
        drv_uart_gnss_send((uint8_t *)cmd, strlen(cmd));

        for (int waited = 0; waited < ACK_TIMEOUT_MS; waited += ACK_POLL_MS) {
            vTaskDelay(pdMS_TO_TICKS(ACK_POLL_MS));
            if (cap_contains("response: OK")) {
                ESP_LOGI(TAG, "ack: %.*s", show, cmd);
                return true;
            }
            if (cap_contains("PARSING FAIL")) {            // matches the "FAILD" typo too
                ESP_LOGW(TAG, "rejected: %.*s", show, cmd);
                return false;                              // won't pass on retry
            }
        }
        ESP_LOGW(TAG, "no ack (attempt %d/%d): %.*s", attempt + 1, retries + 1, show, cmd);
    }
    return false;
}

/* ---- hardware reset --------------------------------------------------- */

static void gnss_reset_pulse(void) {
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

// Reject Null-Island and out-of-range coords: never apply garbage as a fixed base
// (it would broadcast a wrong 1005 to every rover). Mirrors the LH-side guards.
static bool gnss_coord_valid(double lat, double lon) {
    if (lat == 0.0 && lon == 0.0) return false;
    return lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0;
}

static bool gnss_load_fixed_base(double *lat, double *lon, double *h) {
    nvs_handle_t nh;
    if (nvs_open(GNSS_NVS_NS, NVS_READONLY, &nh) != ESP_OK) return false;
    gnss_base_fix_t b;
    size_t sz = sizeof(b);
    esp_err_t err = nvs_get_blob(nh, GNSS_KEY_BASE, &b, &sz);
    nvs_close(nh);
    if (err != ESP_OK || sz != sizeof(b) || !gnss_coord_valid(b.lat, b.lon)) return false;
    *lat = b.lat; *lon = b.lon; *h = b.h;
    return true;
}

static void gnss_save_fixed_base(double lat, double lon, double h) {
    nvs_handle_t nh;
    if (nvs_open(GNSS_NVS_NS, NVS_READWRITE, &nh) != ESP_OK) return;
    gnss_base_fix_t b = { lat, lon, h };
    if (nvs_set_blob(nh, GNSS_KEY_BASE, &b, sizeof(b)) == ESP_OK) nvs_commit(nh);
    nvs_close(nh);
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
        if (send_cmd_acked(cmd, ACK_RETRIES)) ok++;
        ESP_LOGI(TAG, "restored FIXED base from NVS: %.9f %.9f %.4f", lat, lon, h);
        uart_nmea("$PESP,RTK,GNSS,BASEMODE,FIXED");
    } else {
        if (send_cmd_acked("mode base time 300 1.5\r\n", ACK_RETRIES)) ok++;  // VERIFY syntax on first hw
        ESP_LOGI(TAG, "no persisted fixed base -> provisional survey-in");
        uart_nmea("$PESP,RTK,GNSS,BASEMODE,SURVEYIN");
    }
    if (send_cmd_acked("saveconfig\r\n", ACK_RETRIES)) ok++;

    s_capturing = false;
    ESP_LOGI(TAG, "UM980 base config: %d/%d commands acked", ok, total);
    uart_nmea("$PESP,RTK,GNSS,CONFIG,%d,%d", ok, total);
}

bool gnss_set_fixed_base(double lat_deg, double lon_deg, double height_m) {
    // Never apply a garbage coordinate as the fixed base (it would be broadcast as
    // the 1005 to every rover). Reject Null-Island / out-of-range outright.
    if (!gnss_coord_valid(lat_deg, lon_deg)) {
        ESP_LOGE(TAG, "rejecting invalid fixed base %.9f %.9f", lat_deg, lon_deg);
        uart_nmea("$PESP,RTK,GNSS,FIXEDBASE,0");
        return false;
    }

    char cmd[96];
    // UM980: "mode base <lat> <lon> <height>" - degrees / metres, ITRF2020 from the IE
    snprintf(cmd, sizeof(cmd), "mode base %.9f %.9f %.4f\r\n", lat_deg, lon_deg, height_m);

    s_capturing = true;
    bool ok = send_cmd_acked(cmd, ACK_RETRIES);
    if (ok) ok = send_cmd_acked("saveconfig\r\n", ACK_RETRIES);
    s_capturing = false;

    // Persist so a reboot/recovery re-applies this fixed coord instead of re-surveying.
    if (ok) gnss_save_fixed_base(lat_deg, lon_deg, height_m);

    ESP_LOGI(TAG, "fixed base %.9f %.9f %.4f -> %s", lat_deg, lon_deg, height_m, ok ? "OK" : "FAILED");
    uart_nmea("$PESP,RTK,GNSS,FIXEDBASE,%d", ok ? 1 : 0);
    return ok;
}

void gnss_recover(void) {
    ESP_LOGW(TAG, "GNSS recover: hardware reset + reconfigure");
    uart_nmea("$PESP,RTK,GNSS,RECOVER");
    gnss_reset_pulse();
    config_gnss_base();
}

/* Query + log the UM980 firmware version (VERSIONA). The #VERSIONA response
 * carries the model + firmware build (e.g. ...,"UM980",...,"R4.10Build11833",...)
 * which we need for inventory + partner config questions (which UM980 fw is on
 * the station). VERSIONA is a one-shot query, so it still answers after the
 * "unlog com1" in config_gnss_base. */
void gnss_log_version(void) {
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
}

void gnss_init(void) {
    // Register the response-capture handler once (coexists with the NTRIP forwarder).
    uart_register_read_handler(gnss_uart_capture);

    gnss_reset_pulse();
    config_gnss_base();
    gnss_log_version();     // log the UM980 firmware (inventory + partner config)
}
