/*
 * RTKdata Ethernet uplink (K_EVK_V4.0: ESP32 EMAC + LAN8720AI over RMII).
 * See eth.h for the contract and board.h for the pin rationale.
 *
 * License: GPLv3 (see LICENSE).
 */
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_mac.h>
#include <esp_rom_sys.h>
#include <driver/gpio.h>

#include "board.h"
#include "eth.h"
#include "wifi.h"
#include "uart.h"

#if BOARD_HAS_ETH

#include <esp_eth.h>

static const char *TAG = "ETH";

#define ETH_LINK_BIT            BIT0

// Above the STA (100): if both ever hold an IP, the cable carries the traffic.
#define ETH_ROUTE_PRIO          128

// LAN8720 vendor register: Symbol Error Counter (16 bit, rolls over).
#define LAN8720_REG_SECR        26

// Report the PHY's view of the wire once if the link has not come up by then.
#define ETH_NOLINK_REPORT_MS    15000

static esp_eth_handle_t   s_eth      = NULL;
static esp_netif_t       *s_netif    = NULL;
static EventGroupHandle_t s_events   = NULL;

static volatile bool      s_link     = false;
static volatile bool      s_has_ip   = false;
static volatile uint8_t   s_speed    = 0;
static volatile bool      s_full_dup = false;
static volatile uint32_t  s_link_ups = 0;

static portMUX_TYPE       s_secr_mux  = portMUX_INITIALIZER_UNLOCKED;
static uint16_t           s_secr_last = 0;
static uint32_t           s_secr_sum  = 0;
static bool               s_secr_init = false;

/* ---- Diagnostics for a failed bring-up ----------------------------------
 * Run only when the EMAC driver does not install, after it released its
 * pins. They separate "no PHY at all" (not populated, unpowered, held in
 * reset) from "PHY alive but no RMII clock", which is what a support case
 * needs from a unit in the field. Measured on the 2026-09 samples: the
 * non-PoE sample showed no PHY on MDIO and undriven RMII outputs. */

#define RMII_CLK_GPIO           0
#define REFCLK_SAMPLES          2000

// Toggling reads mean transitions on GPIO0 (not a frequency measurement);
// a constant level means no clock.
static int refclk_sample_ones(void) {
    int ones = 0;
    for (int i = 0; i < REFCLK_SAMPLES; i++) ones += gpio_get_level(RMII_CLK_GPIO);
    return ones;
}

static inline void mdio_half(void) { esp_rom_delay_us(2); }

static void mdio_write_bit(int b) {
    gpio_set_level(BOARD_ETH_MDIO_GPIO, b);
    mdio_half();
    gpio_set_level(BOARD_ETH_MDC_GPIO, 1);
    mdio_half();
    gpio_set_level(BOARD_ETH_MDC_GPIO, 0);
}

static int mdio_read_bit(void) {
    mdio_half();
    int b = gpio_get_level(BOARD_ETH_MDIO_GPIO);
    gpio_set_level(BOARD_ETH_MDC_GPIO, 1);
    mdio_half();
    gpio_set_level(BOARD_ETH_MDC_GPIO, 0);
    return b;
}

// Clause-22 read without the EMAC. Returns -1 when no PHY drove the
// turnaround low.
static int mdio_read(int phy, int reg) {
    for (int i = 0; i < 32; i++) mdio_write_bit(1);          // preamble
    int hdr[] = {0, 1, 1, 0};                                 // start 01, read 10
    for (int i = 0; i < 4; i++) mdio_write_bit(hdr[i]);
    for (int i = 4; i >= 0; i--) mdio_write_bit((phy >> i) & 1);
    for (int i = 4; i >= 0; i--) mdio_write_bit((reg >> i) & 1);
    gpio_set_level(BOARD_ETH_MDIO_GPIO, 1);                   // release (open drain)
    mdio_read_bit();                                          // TA, Z
    int ta = mdio_read_bit();                                 // TA, PHY drives 0
    int v = 0;
    for (int i = 0; i < 16; i++) v = (v << 1) | mdio_read_bit();
    mdio_write_bit(1);                                        // idle
    return ta == 0 ? v : -1;
}

// A live LAN8720 out of reset drives RXD0/RXD1/CRS_DV low while idle. A line
// that follows our pull-up and pull-down is not driven by anybody.
static const char *rmii_output_state(int pin) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io);
    esp_rom_delay_us(200);
    int with_up = gpio_get_level(pin);
    gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY);
    esp_rom_delay_us(200);
    int with_down = gpio_get_level(pin);
    gpio_set_pull_mode(pin, GPIO_FLOATING);
    return (with_up && !with_down) ? "float" : (with_up ? "high" : "low");
}

static void eth_diagnose_bringup(void) {

    int ones = refclk_sample_ones();
    bool clock = ones > REFCLK_SAMPLES / 20 && ones < REFCLK_SAMPLES - REFCLK_SAMPLES / 20;

    const char *rxd0 = rmii_output_state(25);
    const char *rxd1 = rmii_output_state(26);
    const char *crs  = rmii_output_state(27);

    gpio_config_t mdc = { .pin_bit_mask = 1ULL << BOARD_ETH_MDC_GPIO, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&mdc);
    gpio_config_t mdio = {
        .pin_bit_mask = 1ULL << BOARD_ETH_MDIO_GPIO,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,       // the board has no external MDIO pull-up
    };
    gpio_config(&mdio);
    gpio_set_level(BOARD_ETH_MDC_GPIO, 0);
    gpio_set_level(BOARD_ETH_MDIO_GPIO, 1);

    int addr = -1, id1 = -1, id2 = -1;
    for (int a = 0; a < 32 && addr < 0; a++) {
        int v = mdio_read(a, 2);
        if (v >= 0 && v != 0xFFFF) { addr = a; id1 = v; id2 = mdio_read(a, 3); }
    }

    if (addr >= 0) {
        ESP_LOGE(TAG, "diagnosis: PHY %04x:%04x at addr %d, REF_CLK %s (%d/%d high), "
                      "RMII outputs %s/%s/%s", id1, id2, addr, clock ? "toggling" : "ABSENT",
                 ones, REFCLK_SAMPLES, rxd0, rxd1, crs);
    } else {
        ESP_LOGE(TAG, "diagnosis: NO PHY on MDIO (not populated, unpowered or held in reset), "
                      "REF_CLK %s (%d/%d high), RMII outputs %s/%s/%s",
                 clock ? "toggling" : "absent", ones, REFCLK_SAMPLES, rxd0, rxd1, crs);
    }
    uart_nmea("$PESP,ETH,DIAG,%d,%04x,%s,%s,%s,%s", addr, id1 < 0 ? 0 : id1,
              clock ? "CLK" : "NOCLK", rxd0, rxd1, crs);
}

/* ---- Link events -------------------------------------------------------- */

static void uplink_lost(const char *why) {

    if (s_has_ip) {
        ESP_LOGW(TAG, "Ethernet uplink lost (%s)", why);
    }
    s_has_ip = false;
    wifi_note_eth_ip(false);
    wifi_sta_set_suspended(false);
}

static void on_eth_event(void *arg, esp_event_base_t base, int32_t id, void *data) {

    switch (id) {

    case ETHERNET_EVENT_CONNECTED: {
        eth_speed_t  speed  = ETH_SPEED_10M;
        eth_duplex_t duplex = ETH_DUPLEX_HALF;
        esp_eth_ioctl(s_eth, ETH_CMD_G_SPEED, &speed);
        esp_eth_ioctl(s_eth, ETH_CMD_G_DUPLEX_MODE, &duplex);
        s_speed    = (speed == ETH_SPEED_100M) ? 100 : 10;
        s_full_dup = (duplex == ETH_DUPLEX_FULL);
        s_link     = true;
        s_link_ups++;
        xEventGroupSetBits(s_events, ETH_LINK_BIT);
        ESP_LOGI(TAG, "link up %u Mbps %s duplex", (unsigned)s_speed, s_full_dup ? "full" : "half");
        uart_nmea("$PESP,ETH,LINK,UP,%u,%c", (unsigned)s_speed, s_full_dup ? 'F' : 'H');
        break;
    }

    case ETHERNET_EVENT_DISCONNECTED:
        s_link  = false;
        s_speed = 0;
        xEventGroupClearBits(s_events, ETH_LINK_BIT);
        ESP_LOGW(TAG, "link down");
        uart_nmea("$PESP,ETH,LINK,DOWN");
        // Do not wait for IP_EVENT_ETH_LOST_IP: esp_netif only raises it after
        // its lost-IP timer, and until then the STA would stay parked with
        // no uplink at all. No link means no uplink, now.
        uplink_lost("link down");
        break;

    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "driver started");
        break;

    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "driver stopped");
        s_link = false;
        xEventGroupClearBits(s_events, ETH_LINK_BIT);
        uplink_lost("driver stopped");
        break;

    default:
        break;
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data) {

    if (id == IP_EVENT_ETH_GOT_IP) {

        const ip_event_got_ip_t *ev = (const ip_event_got_ip_t *)data;

        ESP_LOGI(TAG, "IP " IPSTR "/%d gw " IPSTR,
                 IP2STR(&ev->ip_info.ip), ffs(~ev->ip_info.netmask.addr) - 1,
                 IP2STR(&ev->ip_info.gw));
        uart_nmea("$PESP,ETH,IP," IPSTR "/%d," IPSTR,
                  IP2STR(&ev->ip_info.ip), ffs(~ev->ip_info.netmask.addr) - 1,
                  IP2STR(&ev->ip_info.gw));

        s_has_ip = true;
        wifi_note_eth_ip(true);
        wifi_sta_set_suspended(true);

    } else if (id == IP_EVENT_ETH_LOST_IP) {

        uart_nmea("$PESP,ETH,IP_LOST");
        uplink_lost("DHCP lease lost");
    }
}

/* ---- One-shot "no link" report ------------------------------------------
 * If no link has come up ETH_NOLINK_REPORT_MS after start, log what the PHY
 * sees once. ENERGYON (reg 17 bit 1) = signal from a partner on the wire;
 * zero with a known-good cable means the analog path is broken (the 2026-09
 * samples: MDI bias network missing). A plain unplugged cable looks the same,
 * so this is a hint for support, not a verdict. */

static int phy_reg(uint32_t reg) {
    uint32_t v = 0;
    esp_eth_phy_reg_rw_data_t d = { .reg_addr = reg, .reg_value_p = &v };
    return esp_eth_ioctl(s_eth, ETH_CMD_READ_PHY_REG, &d) == ESP_OK ? (int)v : -1;
}

static void nolink_report_task(void *arg) {

    vTaskDelay(pdMS_TO_TICKS(ETH_NOLINK_REPORT_MS));

    if (!s_link) {
        phy_reg(1);                                  // BMSR link bit is latch-low:
        int bmsr = phy_reg(1);                       // the second read is current
        int mcsr = phy_reg(17), anlpar = phy_reg(5), r27 = phy_reg(27);

        if (bmsr < 0 || mcsr < 0) {
            ESP_LOGW(TAG, "no link after %d s, PHY registers unreadable", ETH_NOLINK_REPORT_MS / 1000);
            uart_nmea("$PESP,ETH,NOLINK,READFAIL");
        } else {
            int energy = (mcsr >> 1) & 1;
            ESP_LOGW(TAG, "no link after %d s: energy on the wire %s, bmsr %04x anlpar %04x r27 %04x%s",
                     ETH_NOLINK_REPORT_MS / 1000, energy ? "yes" : "NO", bmsr,
                     anlpar < 0 ? 0 : anlpar, r27 < 0 ? 0 : r27,
                     energy ? "" : " (cable unplugged, dead port, or board analog path)");
            uart_nmea("$PESP,ETH,NOLINK,%d,%04x", energy, bmsr);
        }
    }
    vTaskDelete(NULL);
}

/* ---- Bring-up ----------------------------------------------------------- */

esp_err_t eth_init(void) {

    s_events = xEventGroupCreate();
    if (s_events == NULL) return ESP_ERR_NO_MEM;

    esp_netif_inherent_config_t inherent = ESP_NETIF_INHERENT_DEFAULT_ETH();
    inherent.route_prio = ETH_ROUTE_PRIO;
    esp_netif_config_t netif_cfg = {
        .base   = &inherent,
        .driver = NULL,
        .stack  = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };
    s_netif = esp_netif_new(&netif_cfg);
    if (s_netif == NULL) {
        ESP_LOGE(TAG, "netif create failed");
        return ESP_FAIL;
    }

    // Same suffix as the setup AP SSID (RTKdata_XXXXXX), so a customer finds
    // the unit in the router's DHCP table under a name they have seen.
    uint8_t sta_mac[6];
    char hostname[24];
    esp_read_mac(sta_mac, ESP_MAC_WIFI_STA);
    snprintf(hostname, sizeof(hostname), "rtkdata-%02x%02x%02x", sta_mac[3], sta_mac[4], sta_mac[5]);
    esp_netif_set_hostname(s_netif, hostname);

    eth_mac_config_t        mac_cfg   = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_cfg  = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_cfg.smi_gpio.mdc_num  = BOARD_ETH_MDC_GPIO;
    emac_cfg.smi_gpio.mdio_num = BOARD_ETH_MDIO_GPIO;
    // RMII clock: LAN8720 REFCLKO -> GPIO0 as INPUT. This is the Kconfig
    // default (CONFIG_ETH_RMII_CLK_INPUT, GPIO0); stated here so a changed
    // sdkconfig cannot silently flip the ESP32 into driving GPIO0.
    emac_cfg.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_cfg.clock_config.rmii.clock_gpio = EMAC_CLK_IN_GPIO;

    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr       = BOARD_ETH_PHY_ADDR;
    // esp_eth_driver_install() pulses nRST and releases it BEFORE the EMAC
    // init, so REFCLKO is running when the EMAC reset needs a clock.
    phy_cfg.reset_gpio_num = BOARD_ETH_PHY_RST_GPIO;
    // 0: esp_eth_start() must not block boot waiting for autonegotiation
    // (4 s by default with no cable). The driver's link poll reports the
    // link whenever it comes up.
    phy_cfg.autonego_timeout_ms = 0;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_cfg);
    if (mac == NULL || phy == NULL) {
        ESP_LOGE(TAG, "EMAC/PHY object create failed (mac=%p phy=%p)", mac, phy);
        uart_nmea("$PESP,ETH,INIT,FAIL,objects");
        if (mac) mac->del(mac);
        if (phy) phy->del(phy);
        return ESP_FAIL;
    }

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_err_t err = esp_eth_driver_install(&eth_cfg, &s_eth);
    if (err != ESP_OK) {
        // Typical: "emac reset timeout" = no 50 MHz on GPIO0. Wi-Fi keeps
        // the unit usable; the diagnosis says whether a PHY is there at all.
        ESP_LOGE(TAG, "driver install failed: %s", esp_err_to_name(err));
        uart_nmea("$PESP,ETH,INIT,FAIL,%s", esp_err_to_name(err));
        mac->del(mac);
        phy->del(phy);
        s_eth = NULL;
        eth_diagnose_bringup();
        return err;
    }

    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &on_eth_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &on_ip_event, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_LOST_IP, &on_ip_event, NULL);

    err = esp_netif_attach(s_netif, esp_eth_new_netif_glue(s_eth));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "netif attach failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t eth_mac[6] = {0};
    esp_eth_ioctl(s_eth, ETH_CMD_G_MAC_ADDR, eth_mac);

    err = esp_eth_start(s_eth);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "up: LAN8720 addr %d, mac " MACSTR ", hostname %s",
             BOARD_ETH_PHY_ADDR, MAC2STR(eth_mac), hostname);
    uart_nmea("$PESP,ETH,INIT,OK," MACSTR, MAC2STR(eth_mac));
    xTaskCreate(nolink_report_task, "eth_nolink", 3072, NULL, 3, NULL);
    return ESP_OK;
}

bool eth_wait_link(uint32_t timeout_ms) {

    if (s_events == NULL || s_eth == NULL) return false;
    EventBits_t bits = xEventGroupWaitBits(s_events, ETH_LINK_BIT, pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(timeout_ms));
    return (bits & ETH_LINK_BIT) != 0;
}

bool eth_link_up(void) { return s_link; }

bool eth_has_ip(void) { return s_has_ip; }

static uint32_t symbol_errors(void) {

    if (s_eth == NULL) return 0;

    uint32_t val = 0;
    esp_eth_phy_reg_rw_data_t reg = { .reg_addr = LAN8720_REG_SECR, .reg_value_p = &val };
    if (esp_eth_ioctl(s_eth, ETH_CMD_READ_PHY_REG, &reg) != ESP_OK) {
        uint32_t sum;
        portENTER_CRITICAL(&s_secr_mux);
        sum = s_secr_sum;
        portEXIT_CRITICAL(&s_secr_mux);
        return sum;
    }

    uint16_t now = (uint16_t)val;
    uint32_t sum;
    portENTER_CRITICAL(&s_secr_mux);
    if (s_secr_init) {
        s_secr_sum += (uint16_t)(now - s_secr_last);   // modulo 2^16: survives the rollover
    } else {
        s_secr_sum  = now;
        s_secr_init = true;
    }
    s_secr_last = now;
    sum = s_secr_sum;
    portEXIT_CRITICAL(&s_secr_mux);
    return sum;
}

void eth_status(eth_status_t *out) {

    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (s_eth == NULL) return;

    out->present     = true;
    out->link        = s_link;
    out->has_ip      = s_has_ip;
    out->speed_mbps  = s_speed;
    out->full_duplex = s_full_dup;
    out->link_ups    = s_link_ups;
    esp_eth_ioctl(s_eth, ETH_CMD_G_MAC_ADDR, out->mac);

    if (s_has_ip) {
        esp_netif_ip_info_t info;
        if (esp_netif_get_ip_info(s_netif, &info) == ESP_OK) {
            out->ip4_addr = info.ip;
            out->gw4_addr = info.gw;
        }
    }

    out->symbol_errors = symbol_errors();
}

void eth_link_recover(void) {

    if (s_eth != NULL && s_link && !s_has_ip) {
        ESP_LOGW(TAG, "link up but no lease -> DHCP client restart");
        uart_nmea("$PESP,RTK,SUPERVISOR,ETHDHCP");
        esp_netif_dhcpc_stop(s_netif);
        esp_netif_dhcpc_start(s_netif);
        return;
    }

    // No cable (or no Ethernet at all): the STA is the uplink to recover.
    wifi_driver_restart();
}

#else /* !BOARD_HAS_ETH */

esp_err_t eth_init(void) { return ESP_ERR_NOT_SUPPORTED; }

bool eth_wait_link(uint32_t timeout_ms) { (void)timeout_ms; return false; }

bool eth_link_up(void) { return false; }

bool eth_has_ip(void) { return false; }

void eth_status(eth_status_t *out) { if (out) memset(out, 0, sizeof(*out)); }

void eth_link_recover(void) { wifi_driver_restart(); }

#endif
