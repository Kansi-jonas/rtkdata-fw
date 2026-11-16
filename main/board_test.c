/*
 * Board test page backend for the Ethernet boards (see board_test.h).
 *
 * GET /test/status   one JSON snapshot: board, resets, heap, Ethernet, Wi-Fi,
 *                    GNSS, PPS, button, PoE, revision ID, last test results
 * GET /test/do?a=... actions: gnss_version, gnss_reset, atecc, led (with
 *                    led=0|1 and m=-1..3), clear_boots, restart
 *
 * License: GPLv3 (see LICENSE).
 */
#include "sdkconfig.h"
#include "board_test.h"

/* A test build dir rebuilt with -DRTK_TEST_PAGE=0 keeps CONFIG_RTK_TEST_PAGE=y
 * in its sdkconfig, and the "customer" image would still carry the test page.
 * Fail the compile instead (same guard as the board choice in board.h). Build
 * customer images in their own dir anyway. */
#if defined(RTK_CMAKE_TEST_PAGE) != defined(CONFIG_RTK_TEST_PAGE)
#error "RTK_TEST_PAGE (CMake) and CONFIG_RTK_TEST_PAGE (sdkconfig) disagree: use a fresh build dir"
#endif

#if CONFIG_RTK_TEST_PAGE

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <esp_random.h>
#include <bootloader_random.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_mac.h>
#include <esp_app_desc.h>
#include <esp_heap_caps.h>
#include <esp_rom_sys.h>
#include <nvs.h>
#include <esp_wifi.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

#include "board.h"
#include "eth.h"
#include "wifi.h"
#include "gnss.h"
#include "status_led.h"
#include "supervisor.h"
#include "update.h"

static const char *TAG = "BOARD_TEST";

#define BT_NVS_NS "btest"

enum { JOB_NONE, JOB_GNSS_VERSION, JOB_GNSS_RESET, JOB_ATECC };
static const char *s_job_name[] = { "idle", "gnss_version", "gnss_reset", "atecc" };
static volatile int s_job = JOB_NONE;
static TaskHandle_t s_worker;

/* ---- boot counter -------------------------------------------------------- */

static const char *s_reason_keys[] = { "r_pwr", "r_sw", "r_panic", "r_wdt", "r_brown", "r_other" };
#define BT_REASONS (sizeof(s_reason_keys) / sizeof(s_reason_keys[0]))
static uint32_t s_boots, s_reason_count[BT_REASONS];
static esp_reset_reason_t s_reason;

static int reason_slot(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON: case ESP_RST_EXT: return 0;   // power-on or EN pin
        case ESP_RST_SW:                       return 1;
        case ESP_RST_PANIC:                    return 2;
        case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return 3;
        case ESP_RST_BROWNOUT:                 return 4;
        default:                               return 5;
    }
}

static const char *reason_name(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:  return "power-on / EN";
        case ESP_RST_EXT:      return "external pin";
        case ESP_RST_SW:       return "software";
        case ESP_RST_PANIC:    return "panic";
        case ESP_RST_INT_WDT:  return "interrupt watchdog";
        case ESP_RST_TASK_WDT: return "task watchdog";
        case ESP_RST_WDT:      return "watchdog";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_DEEPSLEEP:return "deep sleep";
        default:               return "other";
    }
}

/* One ID per boot for the page's running report, which must tell boots apart
 * without guessing from uptime and counters: a sequence number that "Clear
 * counters" never resets, an epoch that is new whenever the NVS was erased
 * (flash erase, or config_init's own recovery) and so restarts the sequence,
 * and a random part in case the NVS cannot be written at all. */
static uint32_t s_boot_seq, s_boot_epoch;
static char s_boot_id[40] = "unknown";
static char s_epoch_str[12] = "none";

static void boots_load_and_count(void) {
    nvs_handle_t h;
    // Before Wi-Fi starts esp_random() is only pseudo-random, so an erased
    // board could draw the same epoch again. The bootloader's SAR ADC entropy
    // source is safe here: ADC and Wi-Fi are not running yet.
    bootloader_random_enable();
    uint32_t rnd = esp_random(), new_epoch = esp_random() | 1;
    bootloader_random_disable();
    if (nvs_open(BT_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        if (nvs_get_u32(h, "epoch", &s_boot_epoch) != ESP_OK || s_boot_epoch == 0) {
            s_boot_epoch = new_epoch;
            nvs_set_u32(h, "epoch", s_boot_epoch);
        }
        nvs_get_u32(h, "seq", &s_boot_seq);
        nvs_get_u32(h, "boots", &s_boots);
        for (size_t i = 0; i < BT_REASONS; i++) nvs_get_u32(h, s_reason_keys[i], &s_reason_count[i]);
        s_boot_seq++;
        s_boots++;
        s_reason_count[reason_slot(s_reason)]++;
        nvs_set_u32(h, "seq", s_boot_seq);
        nvs_set_u32(h, "boots", s_boots);
        nvs_set_u32(h, s_reason_keys[reason_slot(s_reason)], s_reason_count[reason_slot(s_reason)]);
        nvs_commit(h);
        nvs_close(h);
    }
    snprintf(s_epoch_str, sizeof(s_epoch_str), "%08x", (unsigned)s_boot_epoch);
    snprintf(s_boot_id, sizeof(s_boot_id), "%s-%u-%08x", s_epoch_str, (unsigned)s_boot_seq, (unsigned)rnd);
}

static void boots_clear(void) {
    nvs_handle_t h;
    s_boots = 0;
    memset(s_reason_count, 0, sizeof(s_reason_count));
    if (nvs_open(BT_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, "boots");   // "seq" and "epoch" stay: they name the boots
    for (size_t i = 0; i < BT_REASONS; i++) nvs_erase_key(h, s_reason_keys[i]);
    nvs_commit(h);
    nvs_close(h);
}

/* ---- PPS and button counters (interrupts) -------------------------------- */

// 64-bit timestamps are not atomic on the ESP32: ISR and tasks share them
// only under this lock.
static portMUX_TYPE s_pps_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_pps_count;
static int64_t  s_pps_last_us;
static int64_t  s_pps_max_gap_us;   // longest PPS pause since the last clear
static volatile uint32_t s_btn_count;
static int64_t  s_btn_change_us;    // button ISR only: last state change
static bool     s_btn_down;         // button ISR only: debounced state

typedef struct { uint32_t count; int64_t last_us, max_gap_us; } pps_snap_t;

static pps_snap_t pps_snapshot(void) {
    pps_snap_t p;
    portENTER_CRITICAL(&s_pps_mux);
    p.count = s_pps_count;
    p.last_us = s_pps_last_us;
    p.max_gap_us = s_pps_max_gap_us;
    portEXIT_CRITICAL(&s_pps_mux);
    return p;
}

static void pps_clear_gap(void) {
    portENTER_CRITICAL(&s_pps_mux);
    s_pps_max_gap_us = 0;
    portEXIT_CRITICAL(&s_pps_mux);
}

/* ESP32 errata 3.11: while SARADC1 or the Wi-Fi modem sleep powers up, GPIO36
 * and GPIO39 are pulled low for about 80 ns, which can fake an edge. The test
 * build therefore reads the ADC only once before this ISR is attached and turns
 * Wi-Fi power save off; an edge closer than 900 ms to the last one is ignored
 * on top (the UM980 PPS is 1 Hz). */
static void IRAM_ATTR pps_isr(void *arg) {
    (void)arg;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&s_pps_mux);
    if (s_pps_last_us == 0 || now - s_pps_last_us >= 900000) {
        if (s_pps_last_us && now - s_pps_last_us > s_pps_max_gap_us) s_pps_max_gap_us = now - s_pps_last_us;
        s_pps_count++;
        s_pps_last_us = now;
    }
    portEXIT_CRITICAL_ISR(&s_pps_mux);
}

/* A press counts only when the button goes down after being up for at least
 * 50 ms. Contacts bounce on press AND on release; the old falling-edge counter
 * took the release bounce for a new press (13 counts for 3 presses on the
 * bench, 2026-11-06). Both edges are watched and the level read here. */
static void IRAM_ATTR button_isr(void *arg) {
    (void)arg;
    int64_t now = esp_timer_get_time();
    bool down = gpio_get_level(BOARD_BUTTON_GPIO) == BOARD_BUTTON_PRESSED_LEVEL;
    if (down && !s_btn_down && now - s_btn_change_us >= 50000) s_btn_count++;
    if (down != s_btn_down) {
        s_btn_down = down;
        s_btn_change_us = now;
    }
}

/* ---- revision ID (ADC1, GPIO35) ------------------------------------------ */

// 10k/10k from 3.3 V: about 1650 mV, wide window for divider and ADC tolerance.
#define REV_V41_MIN_MV 1400
#define REV_V41_MAX_MV 1900

#ifdef BOARD_REV_ID_GPIO
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static bool s_adc_ok, s_cali_ok;
static int s_rev_mv = -1;

static int rev_read_mv(void);

// Read once at boot, then release ADC1 (see the GPIO39 note at pps_isr).
static void rev_measure_once(void) {
    adc_oneshot_unit_init_cfg_t unit = { .unit_id = ADC_UNIT_1 };
    if (adc_oneshot_new_unit(&unit, &s_adc) != ESP_OK) return;
    adc_oneshot_chan_cfg_t ch = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    s_adc_ok = adc_oneshot_config_channel(s_adc, ADC_CHANNEL_7, &ch) == ESP_OK;   // GPIO35
    adc_cali_line_fitting_config_t cal = {
        .unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT,
        .default_vref = 1100,   // only used if the eFuse holds no Vref
    };
    s_cali_ok = adc_cali_create_scheme_line_fitting(&cal, &s_cali) == ESP_OK;
    s_rev_mv = rev_read_mv();
    if (s_cali_ok) adc_cali_delete_scheme_line_fitting(s_cali);
    adc_oneshot_del_unit(s_adc);
    s_adc_ok = s_cali_ok = false;
    ESP_LOGI(TAG, "revision ID divider: %d mV", s_rev_mv);
}

static int rev_read_mv(void) {
    if (!s_adc_ok) return -1;
    int sum = 0, n = 0;
    for (int i = 0; i < 16; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, ADC_CHANNEL_7, &raw) == ESP_OK) { sum += raw; n++; }
    }
    if (n == 0) return -1;
    int raw = sum / n, mv = 0;
    if (s_cali_ok && adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) return mv;
    return raw * 3300 / 4095;   // rough, uncalibrated
}

static const char *rev_name(int mv) {
    if (mv < 0)                  return "ADC error";
    if (mv >= REV_V41_MIN_MV && mv <= REV_V41_MAX_MV) return "ID 1 = K_EVK_V4.1";
    if (mv < 150 || mv > 3000)   return "no divider (old board?)";
    return "unknown";
}
#endif

/* ---- ATECC608B presence (wake + read the 4-byte wake answer) -------------- */

static char s_atecc_result[96] = "not run";

static void atecc_check(void) {
#if defined(BOARD_I2C_SDA_GPIO) && defined(BOARD_REV_ID_GPIO)
    // A reworked V4.0 runs this image too, and there GPIO15 is still the UM980
    // reset net: wake pulse and I2C traffic would reset the receiver. Only a
    // board whose divider says V4.1 gets the probe.
    if (!(s_rev_mv >= REV_V41_MIN_MV && s_rev_mv <= REV_V41_MAX_MV)) {
        snprintf(s_atecc_result, sizeof(s_atecc_result),
                 "SKIPPED: revision ID %d mV is not V4.1, GPIO15 may be the GNSS reset", s_rev_mv);
        ESP_LOGW(TAG, "ATECC: %s", s_atecc_result);
        return;
    }
    // Wake: SDA low for >= 60 us, then >= 1.5 ms before the first transfer.
    gpio_config_t sda = { .pin_bit_mask = 1ULL << BOARD_I2C_SDA_GPIO, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&sda);
    gpio_set_level(BOARD_I2C_SDA_GPIO, 0);
    esp_rom_delay_us(100);
    gpio_set_level(BOARD_I2C_SDA_GPIO, 1);
    sda.mode = GPIO_MODE_INPUT;
    gpio_config(&sda);
    esp_rom_delay_us(3000);   // not vTaskDelay: 3 ms is 0 ticks at 100 Hz

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,                          // any free port
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,   // R78 / R65 2.2k on the board
    };
    i2c_master_bus_handle_t bus = NULL;
    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) {
        snprintf(s_atecc_result, sizeof(s_atecc_result), "FAIL: I2C bus init");
        return;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x60,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev = NULL;
    uint8_t buf[4] = {0};
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (err == ESP_OK) err = i2c_master_receive(dev, buf, sizeof(buf), 50);
    if (err == ESP_OK && buf[0] == 0x04 && buf[1] == 0x11 && buf[2] == 0x33 && buf[3] == 0x43) {
        snprintf(s_atecc_result, sizeof(s_atecc_result), "PASS: ATECC608B answered (04 11 33 43)");
    } else if (err == ESP_OK) {
        snprintf(s_atecc_result, sizeof(s_atecc_result), "FAIL: unexpected answer %02X %02X %02X %02X",
                 buf[0], buf[1], buf[2], buf[3]);
    } else {
        snprintf(s_atecc_result, sizeof(s_atecc_result), "FAIL: no answer at 0x60 (%s)", esp_err_to_name(err));
    }
    if (dev) {
        uint8_t sleep_cmd = 0x01;
        i2c_master_transmit(dev, &sleep_cmd, 1, 50);
        i2c_master_bus_rm_device(dev);
    }
    i2c_del_master_bus(bus);
    gpio_reset_pin(BOARD_I2C_SDA_GPIO);
    gpio_reset_pin(BOARD_I2C_SCL_GPIO);
#else
    snprintf(s_atecc_result, sizeof(s_atecc_result), "SKIPPED: not on this board");
#endif
    ESP_LOGI(TAG, "ATECC: %s", s_atecc_result);
}

/* ---- GNSS ------------------------------------------------------------------ */

static char s_gnss_version[256] = "not run";
static char s_gnss_reset_result[128] = "not run";

static void gnss_version_check(void) {
    size_t n = gnss_query_version(s_gnss_version, sizeof(s_gnss_version));
    if (n == 0) snprintf(s_gnss_version, sizeof(s_gnss_version), "FAIL: no answer from UM980");
    ESP_LOGI(TAG, "GNSS version: %s", s_gnss_version);
}

/* A VERSIONA answer alone does not prove the reset: the old receiver answers
 * too if the reset line does nothing (reworked V4.0: reset still on GPIO15).
 * The proof is the PPS: a real reset stops it for well over a second. */
static void gnss_reset_check(void) {
    int64_t t0 = esp_timer_get_time();
    pps_snap_t before = pps_snapshot();
    bool pps_before = before.last_us && (t0 - before.last_us) < 1500000;
    pps_clear_gap();
    gnss_test_reset();
    char ver[64];
    size_t n = gnss_query_version(ver, sizeof(ver));
    int64_t now = esp_timer_get_time();
    pps_snap_t after = pps_snapshot();
    int64_t gap = after.max_gap_us;
    if (after.last_us && now - after.last_us > gap) gap = now - after.last_us;
    int ms = (int)((now - t0) / 1000);
    if (n == 0) {
        snprintf(s_gnss_reset_result, sizeof(s_gnss_reset_result),
                 "FAIL: no answer after the reset (%d ms)", ms);
    } else if (!pps_before) {
        snprintf(s_gnss_reset_result, sizeof(s_gnss_reset_result),
                 "CHECK: answers after %d ms, but no PPS before the reset, so not proven (antenna?)", ms);
    } else if (gap > 1500000) {
        snprintf(s_gnss_reset_result, sizeof(s_gnss_reset_result),
                 "PASS: PPS stopped %d ms, UM980 answers again after %d ms", (int)(gap / 1000), ms);
    } else {
        snprintf(s_gnss_reset_result, sizeof(s_gnss_reset_result),
                 "FAIL: PPS never stopped, the reset line did not reset the UM980");
    }
    ESP_LOGI(TAG, "GNSS reset: %s", s_gnss_reset_result);
}

/* ---- HTTP ------------------------------------------------------------------- */

static void add_ip(cJSON *o, const char *k, const esp_ip4_addr_t *ip) {
    char s[16];
    snprintf(s, sizeof(s), IPSTR, IP2STR(ip));
    cJSON_AddStringToObject(o, k, s);
}

static esp_err_t send_json(httpd_req_t *req, cJSON *root) {
    char *txt = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!txt) return httpd_resp_send_500(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t r = httpd_resp_sendstr(req, txt);
    free(txt);
    return r;
}

esp_err_t board_test_status_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();

    const esp_app_desc_t *app = esp_app_get_description();
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    uint32_t flash = 0;
    esp_flash_get_size(NULL, &flash);
    uint8_t mac[6];
    char macs[18];

    cJSON *b = cJSON_AddObjectToObject(root, "board");
    cJSON_AddStringToObject(b, "name", BOARD_NAME);
    cJSON_AddStringToObject(b, "project", app->project_name);
    cJSON_AddStringToObject(b, "version", app->version);
    cJSON_AddStringToObject(b, "idf", app->idf_ver);
    cJSON_AddNumberToObject(b, "chip_rev", chip.revision);
    cJSON_AddNumberToObject(b, "cores", chip.cores);
    cJSON_AddNumberToObject(b, "flash_mb", flash / (1024 * 1024));
    cJSON_AddNumberToObject(b, "psram_kb", heap_caps_get_total_size(MALLOC_CAP_SPIRAM) / 1024);
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(macs, sizeof(macs), MACSTR, MAC2STR(mac));
    cJSON_AddStringToObject(b, "mac_wifi", macs);

    cJSON *r = cJSON_AddObjectToObject(root, "reset");
    cJSON_AddStringToObject(r, "last", reason_name(s_reason));
    cJSON_AddNumberToObject(r, "uptime_s", (double)(esp_timer_get_time() / 1000000));
    cJSON_AddNumberToObject(r, "boots", s_boots);
    cJSON_AddStringToObject(r, "boot_id", s_boot_id);
    cJSON_AddNumberToObject(r, "boot_seq", s_boot_seq);
    cJSON_AddStringToObject(r, "boot_epoch", s_epoch_str);
    for (size_t i = 0; i < BT_REASONS; i++) cJSON_AddNumberToObject(r, s_reason_keys[i], s_reason_count[i]);

    cJSON *hp = cJSON_AddObjectToObject(root, "heap");
    cJSON_AddNumberToObject(hp, "internal_free", heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    cJSON_AddNumberToObject(hp, "internal_min", heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    cJSON_AddNumberToObject(hp, "internal_largest", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    cJSON_AddNumberToObject(hp, "psram_free", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    eth_status_t es;
    eth_status(&es);
    cJSON *e = cJSON_AddObjectToObject(root, "eth");
    cJSON_AddBoolToObject(e, "present", es.present);
    cJSON_AddBoolToObject(e, "link", es.link);
    cJSON_AddNumberToObject(e, "speed", es.speed_mbps);
    cJSON_AddBoolToObject(e, "full_duplex", es.full_duplex);
    cJSON_AddBoolToObject(e, "has_ip", es.has_ip);
    add_ip(e, "ip", &es.ip4_addr);
    add_ip(e, "gw", &es.gw4_addr);
    snprintf(macs, sizeof(macs), MACSTR, MAC2STR(es.mac));
    cJSON_AddStringToObject(e, "mac", macs);
    cJSON_AddNumberToObject(e, "link_ups", es.link_ups);
    cJSON_AddNumberToObject(e, "symbol_errors", es.symbol_errors);

    wifi_ap_status_t ap;
    memset(&ap, 0, sizeof(ap));
    wifi_ap_status(&ap);
    wifi_sta_status_t sta;
    memset(&sta, 0, sizeof(sta));
    wifi_sta_status(&sta);
    cJSON *w = cJSON_AddObjectToObject(root, "wifi");
    cJSON_AddBoolToObject(w, "ap_active", ap.active);
    cJSON_AddStringToObject(w, "ap_ssid", ap.ssid);
    cJSON_AddNumberToObject(w, "ap_clients", ap.devices);
    cJSON_AddBoolToObject(w, "sta_connected", sta.connected);
    cJSON_AddBoolToObject(w, "sta_parked", sta.suspended);
    cJSON_AddStringToObject(w, "sta_ssid", sta.ssid);
    cJSON_AddNumberToObject(w, "sta_rssi", sta.rssi);
    add_ip(w, "sta_ip", &sta.ip4_addr);

    supervisor_health_t h;
    supervisor_health(&h);
    cJSON *g = cJSON_AddObjectToObject(root, "gnss");
    cJSON_AddBoolToObject(g, "data_ok", h.gnss_ok);
    cJSON_AddNumberToObject(g, "silent_s", h.gnss_silent_s);
    cJSON_AddStringToObject(g, "version", s_gnss_version);
    cJSON_AddStringToObject(g, "reset_test", s_gnss_reset_result);
    pps_snap_t pps = pps_snapshot();
    int64_t pps_age = pps.last_us ? (esp_timer_get_time() - pps.last_us) / 1000 : -1;
    cJSON_AddNumberToObject(g, "pps_count", pps.count);
    cJSON_AddNumberToObject(g, "pps_age_ms", (double)pps_age);
    cJSON_AddNumberToObject(g, "pps_max_gap_ms", (double)(pps.max_gap_us / 1000));

    cJSON *io = cJSON_AddObjectToObject(root, "io");
    cJSON_AddBoolToObject(io, "button_pressed", gpio_get_level(BOARD_BUTTON_GPIO) == BOARD_BUTTON_PRESSED_LEVEL);
    cJSON_AddNumberToObject(io, "button_presses", s_btn_count);
#if defined(BOARD_POE_SENSE_GPIO) && defined(BOARD_REV_ID_GPIO)
    // Only a V4.1 has the sense circuit (T12, R166 pull-up). On a V4.0 GPIO36
    // floats (input-only pin, no internal pull) and read "PoE present" on a
    // USB-powered board without a PoE module (bench 13.11.). Without the
    // revision divider the field is left out and the page shows "n/a".
    if (s_rev_mv >= REV_V41_MIN_MV && s_rev_mv <= REV_V41_MAX_MV) {
        cJSON_AddBoolToObject(io, "poe_voltage", gpio_get_level(BOARD_POE_SENSE_GPIO) == 0);
    }
#endif
#ifdef BOARD_REV_ID_GPIO
    cJSON_AddNumberToObject(io, "rev_id_mv", s_rev_mv);
    cJSON_AddStringToObject(io, "rev_id", rev_name(s_rev_mv));
#endif
    cJSON_AddNumberToObject(io, "led_ntrip", status_led_test_get(0));
    cJSON_AddNumberToObject(io, "led_second", status_led_test_get(1));
    cJSON_AddStringToObject(io, "atecc", s_atecc_result);
    cJSON_AddStringToObject(root, "job", s_job_name[s_job]);

    return send_json(req, root);
}

esp_err_t board_test_do_handler(httpd_req_t *req) {
    char q[96] = {0}, a[24] = {0}, v[8] = {0};
    httpd_req_get_url_query_str(req, q, sizeof(q));
    httpd_query_key_value(q, "a", a, sizeof(a));

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "action", a);

    int job = JOB_NONE;
    if (strcmp(a, "gnss_version") == 0)    job = JOB_GNSS_VERSION;
    else if (strcmp(a, "gnss_reset") == 0) job = JOB_GNSS_RESET;
    else if (strcmp(a, "atecc") == 0)      job = JOB_ATECC;

    // A floating GPIO35 on a reworked V4.0 can read inside the V4.1 window, so
    // the divider alone is not proof: the tester confirms V4.1 on the page too.
    if (job == JOB_ATECC &&
        !(httpd_query_key_value(q, "v41", v, sizeof(v)) == ESP_OK && strcmp(v, "1") == 0)) {
        snprintf(s_atecc_result, sizeof(s_atecc_result), "SKIPPED: not confirmed as V4.1 on the page");
        cJSON_AddStringToObject(root, "result", s_atecc_result);
    } else if (job != JOB_NONE) {
        // The GNSS jobs hold the receiver lock for seconds (the reset runs the
        // whole ACK-gated config): never on the httpd task.
        if (s_job != JOB_NONE || s_worker == NULL) {
            cJSON_AddStringToObject(root, "result", "busy, try again");
        } else {
            s_job = job;
            xTaskNotifyGive(s_worker);
            cJSON_AddStringToObject(root, "result", "started");
        }
    } else if (strcmp(a, "led") == 0) {
        int led = 0, mode = -1;
        if (httpd_query_key_value(q, "led", v, sizeof(v)) == ESP_OK) led = atoi(v);
        if (httpd_query_key_value(q, "m", v, sizeof(v)) == ESP_OK) mode = atoi(v);
        status_led_test_set(led, mode);
        cJSON_AddStringToObject(root, "result", "ok");
    } else if (strcmp(a, "clear_boots") == 0 && s_job == JOB_GNSS_RESET) {
        // The running reset test measures the PPS pause; a clear would erase it.
        cJSON_AddStringToObject(root, "result", "busy, try again");
    } else if (strcmp(a, "clear_boots") == 0) {
        boots_clear();
        pps_clear_gap();
        cJSON_AddStringToObject(root, "result", "counters cleared");
    } else if (strcmp(a, "restart") == 0) {
        cJSON_AddStringToObject(root, "result", "restarting");
        send_json(req, root);
        vTaskDelay(pdMS_TO_TICKS(500));
        ota_note_planned_reboot();   // a test restart is not a crash (boot-loop guard)
        esp_restart();
        return ESP_OK;
    } else {
        cJSON_AddStringToObject(root, "result", "unknown action");
    }
    return send_json(req, root);
}

static void worker_task(void *arg) {
    (void)arg;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        switch (s_job) {
            case JOB_GNSS_VERSION: gnss_version_check(); break;
            case JOB_GNSS_RESET:   gnss_reset_check();   break;
            case JOB_ATECC:        atecc_check();        break;
            default: break;
        }
        s_job = JOB_NONE;
    }
}

void board_test_count_boot(void) {
    s_reason = esp_reset_reason();
    boots_load_and_count();
    ESP_LOGW(TAG, "board test build, boot #%u since clear, reason: %s",
             (unsigned)s_boots, reason_name(s_reason));
}

void board_test_init(void) {
    ESP_LOGW(TAG, "board test page enabled: http://<device ip>/test.html");

    // Same stack as the supervisor, which runs the same GNSS recovery path.
    if (xTaskCreate(worker_task, "board_test", 4096, NULL, 4, &s_worker) != pdPASS) {
        s_worker = NULL;
        ESP_LOGE(TAG, "board test worker NOT created: GNSS and ATECC buttons answer busy");
    }

#ifdef BOARD_REV_ID_GPIO
    rev_measure_once();
#endif
    // Test build only: no modem sleep, so no GPIO39 glitches (see pps_isr).
    esp_wifi_set_ps(WIFI_PS_NONE);

    gpio_install_isr_service(0);   // ESP_ERR_INVALID_STATE if already installed: fine

#ifdef BOARD_PPS_GPIO
    gpio_config_t pps = { .pin_bit_mask = 1ULL << BOARD_PPS_GPIO, .mode = GPIO_MODE_INPUT,
                          .intr_type = GPIO_INTR_POSEDGE };
    gpio_config(&pps);
    gpio_isr_handler_add(BOARD_PPS_GPIO, pps_isr, NULL);
#endif
    // The button task already made GPIO34 an input; only add the edge counter.
    s_btn_down = gpio_get_level(BOARD_BUTTON_GPIO) == BOARD_BUTTON_PRESSED_LEVEL;
    gpio_set_intr_type(BOARD_BUTTON_GPIO, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(BOARD_BUTTON_GPIO, button_isr, NULL);
    gpio_intr_enable(BOARD_BUTTON_GPIO);

#ifdef BOARD_POE_SENSE_GPIO
    gpio_config_t poe = { .pin_bit_mask = 1ULL << BOARD_POE_SENSE_GPIO, .mode = GPIO_MODE_INPUT };
    gpio_config(&poe);
#endif
}

#else /* !CONFIG_RTK_TEST_PAGE */

void board_test_count_boot(void) {}
void board_test_init(void) {}

#endif
