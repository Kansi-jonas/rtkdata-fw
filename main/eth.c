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

/* ---- RMII reference clock bring-up --------------------------------------
 *
 * The LAN8720 latches its nINTSEL strap on the rising edge of nRST, from the
 * same pin that later outputs REFCLKO (nINT/REFCLKO). Only nINTSEL = LOW
 * selects "REF_CLK Out", i.e. the 50 MHz we need on GPIO0. On K_EVK_V4 that
 * pin is tied through R102 (0R) to GPIO0, which carries R71 10k to 3V3 (boot
 * strap) against R100 10k to GND, plus the ESP32 and LAN8720 internal
 * pull-ups: the strap floats near 2 V and reads HIGH, the PHY comes up in
 * nINT mode and never clocks the EMAC ("emac reset timeout", measured on the
 * first sample 2026-09-29).
 *
 * Fix without touching the board: hold GPIO0 actively LOW while nRST rises,
 * then hand GPIO0 straight back as an input for the clock. The PHY is in
 * reset (outputs tri-stated) while we drive, so there is no contention before
 * the edge; after it we release within about a microsecond. ESP-IDF must NOT
 * pulse nRST again afterwards (reset_gpio_num = -1): a second rising edge with
 * GPIO0 as an input would latch nINT mode again. */

static void rmii_outputs_probe(const char *when);

#define RMII_CLK_GPIO           0
#define REFCLK_SAMPLES          2000

// A running 50 MHz clock read back through the GPIO matrix gives a mix of
// levels; a dead line gives a constant one.
static int refclk_sample_ones(void) {
    int ones = 0;
    for (int i = 0; i < REFCLK_SAMPLES; i++) ones += gpio_get_level(RMII_CLK_GPIO);
    return ones;
}

static bool refclk_running(int ones) {
    return ones > REFCLK_SAMPLES / 20 && ones < REFCLK_SAMPLES - REFCLK_SAMPLES / 20;
}

static bool phy_release_with_refclk_strap(void) {

    gpio_config_t rst = {
        .pin_bit_mask = 1ULL << BOARD_ETH_PHY_RST_GPIO,
        .mode = GPIO_MODE_INPUT_OUTPUT,   // input too: read back the actual pin level
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst);
    gpio_set_level(BOARD_ETH_PHY_RST_GPIO, 0);          // PHY in reset

    gpio_config_t clk = {
        .pin_bit_mask = 1ULL << RMII_CLK_GPIO,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&clk);
    gpio_set_level(RMII_CLK_GPIO, 0);                   // nINTSEL = 0 -> REF_CLK Out

    esp_rom_delay_us(1000);                             // reset asserted >> 100 us
    rmii_outputs_probe("in_reset");

    // No preemption between the edge and the release: every microsecond after
    // the edge is the PHY's REFCLKO driving against our low.
    static portMUX_TYPE edge_mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&edge_mux);
    gpio_set_level(BOARD_ETH_PHY_RST_GPIO, 1);          // strap latched on this edge
    gpio_set_direction(RMII_CLK_GPIO, GPIO_MODE_INPUT); // hand the line to REFCLKO
    portEXIT_CRITICAL(&edge_mux);

    vTaskDelay(pdMS_TO_TICKS(20));                      // oscillator + PLL settle

    int ones = refclk_sample_ones();
    bool running = refclk_running(ones);
    ESP_LOGI(TAG, "RMII REF_CLK on GPIO%d after strap-forced release: %s (%d/%d high)",
             RMII_CLK_GPIO, running ? "running" : "NOT running", ones, REFCLK_SAMPLES);
    uart_nmea("$PESP,ETH,REFCLK,%s,%d", running ? "OK" : "NONE", ones);
    return running;
}

/* ---- MDIO bit-bang probe (bring-up diagnostics) -------------------------
 * Clause-22 reads on MDC/MDIO without the EMAC, so a dead RMII clock does
 * not hide whether the PHY itself is alive (powered, out of reset, strapped
 * address). Runs before the EMAC claims the pins. */

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

// Returns the register value, or -1 when no PHY drove the turnaround low.
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

/* Is anybody driving the PHY's RMII outputs? A live LAN8720 out of reset
 * drives RXD0/RXD1/CRS_DV low while idle. Read each line once with the ESP32
 * pull-up and once with the pull-down: a line that follows our pull is
 * floating (PHY not driving = dead/unpowered/unclocked); a line that stays low
 * against the pull-up is driven. */
static const int s_rmii_outs[] = {25, 26, 27};   // RXD0, RXD1, CRS_DV

static void rmii_outputs_probe(const char *when) {
    char res[3][8];
    for (int i = 0; i < 3; i++) {
        int pin = s_rmii_outs[i];
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
        snprintf(res[i], sizeof(res[i]), "%s",
                 (with_up && !with_down) ? "float" : (with_up ? "high" : "low"));
    }
    ESP_LOGI(TAG, "RMII outputs %s: RXD0=%s RXD1=%s CRS_DV=%s", when, res[0], res[1], res[2]);
    uart_nmea("$PESP,ETH,RMIIOUT,%s,%s,%s,%s", when, res[0], res[1], res[2]);
}

static void phy_mdio_probe(void) {
    gpio_config_t mdc = {
        .pin_bit_mask = 1ULL << BOARD_ETH_MDC_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&mdc);
    gpio_config_t mdio = {
        .pin_bit_mask = 1ULL << BOARD_ETH_MDIO_GPIO,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&mdio);
    gpio_set_level(BOARD_ETH_MDC_GPIO, 0);
    gpio_set_level(BOARD_ETH_MDIO_GPIO, 1);

    int found = 0;
    for (int a = 0; a < 32; a++) {
        int id1 = mdio_read(a, 2);
        if (id1 < 0 || id1 == 0xFFFF) continue;
        int id2 = mdio_read(a, 3), bmcr = mdio_read(a, 0), bmsr = mdio_read(a, 1);
        int smr = mdio_read(a, 18), mcsr = mdio_read(a, 17), scsr = mdio_read(a, 31);
        found++;
        ESP_LOGI(TAG, "MDIO: PHY at addr %d id %04x:%04x bmcr %04x bmsr %04x (link %s) "
                      "modes(18) %04x mcsr(17) %04x scsr(31) %04x",
                 a, id1, id2, bmcr, bmsr, (bmsr >= 0 && (bmsr & 0x4)) ? "up" : "down",
                 smr, mcsr, scsr);
        uart_nmea("$PESP,ETH,MDIO,%d,%04x,%04x,%04x,%04x", a, id1, id2, bmsr, smr);
    }
    if (!found) {
        ESP_LOGE(TAG, "MDIO: no PHY answers on any address (unpowered, held in reset, or MDC/MDIO open)");
        uart_nmea("$PESP,ETH,MDIO,NONE");
    }
}

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

/* Bring-up diagnostics: while there is no link, dump the PHY's view of the
 * wire every 3 s for the first minute. ENERGYON (reg 17 bit 1) = the PHY
 * hears the partner; reg 5 = the partner's advertised abilities it decoded;
 * reg 1 bit 5 = autonegotiation complete (needs the partner to hear US). */
static int phy_reg(uint32_t reg) {
    uint32_t v = 0;
    esp_eth_phy_reg_rw_data_t d = { .reg_addr = reg, .reg_value_p = &v };
    return esp_eth_ioctl(s_eth, ETH_CMD_READ_PHY_REG, &d) == ESP_OK ? (int)v : -1;
}

static void phy_diag_task(void *arg) {
    for (int i = 0; i < 20 && !s_link; i++) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        int bmsr = phy_reg(1), anlpar = phy_reg(5), aner = phy_reg(6), mcsr = phy_reg(17);
        int scsr = phy_reg(31), isr = phy_reg(29), secr = phy_reg(26);
        ESP_LOGI(TAG, "PHY diag: bmsr %04x (link %d, an_done %d) anlpar(5) %04x aner(6) %04x "
                      "mcsr(17) %04x (energyon %d) scsr(31) %04x isr(29) %04x secr(26) %u",
                 bmsr, (bmsr >> 2) & 1, (bmsr >> 5) & 1, anlpar, aner,
                 mcsr, (mcsr >> 1) & 1, scsr, isr, (unsigned)secr);
        uart_nmea("$PESP,ETH,PHYDIAG,%04x,%04x,%04x,%04x,%04x", bmsr, anlpar, aner, mcsr, scsr);
    }
    vTaskDelete(NULL);
}

/* ---- Bench test matrix (TEMPORARY, bring-up only) ------------------------
 * Tries every documented speed / negotiation / MDI-MDIX combination on the
 * LAN8720A and records what the PHY sees on the wire. Answers "can ANY
 * firmware setting get a link on this board". Remove after bring-up. */
#define ETH_BENCH_MATRIX 1

static int phy_wr(uint32_t reg, uint32_t val) {
    esp_eth_phy_reg_rw_data_t d = { .reg_addr = reg, .reg_value_p = &val };
    return esp_eth_ioctl(s_eth, ETH_CMD_WRITE_PHY_REG, &d) == ESP_OK ? 0 : -1;
}

typedef struct {
    const char *name;
    bool        autoneg;
    uint16_t    anar;        // advertisement (autoneg only)
    uint16_t    bmcr_forced; // speed/duplex bits (forced only)
    int         mdix;        // -1 auto, 0 forced MDI, 1 forced MDIX
} phy_trial_t;

static const phy_trial_t s_trials[] = {
    { "AN full, auto-MDIX",     true,  0x01E1, 0,      -1 },
    { "AN 10-only, auto-MDIX",  true,  0x0061, 0,      -1 },
    { "AN full, forced MDI",    true,  0x01E1, 0,       0 },
    { "AN full, forced MDIX",   true,  0x01E1, 0,       1 },
    { "forced 10 half, MDI",    false, 0,      0x0000,  0 },
    { "forced 10 half, MDIX",   false, 0,      0x0000,  1 },
    { "forced 100 half, MDI",   false, 0,      0x2000,  0 },
    { "forced 100 half, MDIX",  false, 0,      0x2000,  1 },
};

static void phy_matrix_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "MATRIX start: %d trials", (int)(sizeof(s_trials) / sizeof(s_trials[0])));

    for (size_t t = 0; t < sizeof(s_trials) / sizeof(s_trials[0]); t++) {
        const phy_trial_t *tr = &s_trials[t];

        // Normal operation: no power-down, isolate or loopback; no EDPD / far loopback.
        int mcsr = phy_reg(17);
        if (mcsr >= 0) phy_wr(17, (uint32_t)mcsr & ~((1u << 13) | (1u << 9)));

        int sc = phy_reg(27);
        if (sc >= 0) {
            uint32_t v = (uint32_t)sc;
            if (tr->mdix < 0) {
                v &= ~(1u << 15);                          // AMDIXCTRL=0: Auto-MDIX
            } else {
                v |= (1u << 15);                           // manual
                v = tr->mdix ? (v | (1u << 13)) : (v & ~(1u << 13));  // CH_SELECT
            }
            phy_wr(27, v);
        }

        if (tr->autoneg) {
            phy_wr(4, tr->anar);
            phy_wr(0, (1u << 12) | (1u << 9));              // AN enable + restart
        } else {
            phy_wr(0, tr->bmcr_forced);                    // AN off, speed, half duplex
        }

        int energy_max = 0, link_any = 0, an_any = 0, anlpar_seen = 0x0001, aner_seen = 0;
        for (int s = 0; s < 9; s++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            phy_reg(1);                                    // BMSR link is latch-low:
            int bmsr = phy_reg(1);                         // second read = current state
            int m = phy_reg(17), lp = phy_reg(5), ae = phy_reg(6);
            if (bmsr < 0 || m < 0) continue;
            energy_max |= (m >> 1) & 1;
            link_any   |= (bmsr >> 2) & 1;
            an_any     |= (bmsr >> 5) & 1;
            if (lp > 0 && lp != 0x0001) anlpar_seen = lp;
            if (ae > 0) aner_seen |= ae;
        }
        int bmcr = phy_reg(0), r27 = phy_reg(27);
        ESP_LOGI(TAG, "MATRIX %d/%d %-22s energyon %d link %d an_done %d anlpar %04x aner %04x "
                      "(bmcr %04x r27 %04x)",
                 (int)t + 1, (int)(sizeof(s_trials) / sizeof(s_trials[0])), tr->name,
                 energy_max, link_any, an_any, anlpar_seen, aner_seen, bmcr, r27);
        uart_nmea("$PESP,ETH,MATRIX,%d,%d,%d,%d,%04x", (int)t + 1, energy_max, link_any,
                  an_any, anlpar_seen);
    }

    // Back to the default: autonegotiation, full advertisement, Auto-MDIX.
    int sc = phy_reg(27);
    if (sc >= 0) phy_wr(27, (uint32_t)sc & ~(1u << 15));
    phy_wr(4, 0x01E1);
    phy_wr(0, (1u << 12) | (1u << 9));
    ESP_LOGI(TAG, "MATRIX done, PHY back to autonegotiation");
    vTaskDelete(NULL);
}

esp_err_t eth_init(void) {

    s_events = xEventGroupCreate();
    if (s_events == NULL) return ESP_ERR_NO_MEM;

    // Before the EMAC exists: its init needs the PHY clock already running.
    phy_release_with_refclk_strap();
    rmii_outputs_probe("released");
    phy_mdio_probe();
    {
        int ones = refclk_sample_ones();
        ESP_LOGI(TAG, "REF_CLK after MDIO probe (+~%d ms): %d/%d high, LAN_RST reads %d",
                 40, ones, REFCLK_SAMPLES, gpio_get_level(BOARD_ETH_PHY_RST_GPIO));
    }

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
    // -1: we released nRST ourselves with the strap forced. A driver pulse
    // would re-latch nINTSEL from the pulled-up GPIO0 and kill the clock.
    phy_cfg.reset_gpio_num = -1;

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
        // Typical: "emac reset timeout" = no 50 MHz on GPIO0 (PHY held in
        // reset, missing crystal, R102 open). Wi-Fi keeps the unit usable.
        ESP_LOGE(TAG, "driver install failed: %s (no RMII clock on GPIO0?)", esp_err_to_name(err));
        uart_nmea("$PESP,ETH,INIT,FAIL,%s", esp_err_to_name(err));
        // Bring-up diagnostics: rule out a slow PHY start (seconds, not ms).
        vTaskDelay(pdMS_TO_TICKS(3000));
        {
            int ones = refclk_sample_ones();
            ESP_LOGI(TAG, "3 s later: REF_CLK %d/%d high, LAN_RST %d", ones, REFCLK_SAMPLES,
                     gpio_get_level(BOARD_ETH_PHY_RST_GPIO));
        }
        rmii_outputs_probe("after_3s");
        phy_mdio_probe();
        mac->del(mac);
        phy->del(phy);
        s_eth = NULL;
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
#if ETH_BENCH_MATRIX
    xTaskCreate(phy_matrix_task, "eth_matrix", 4096, NULL, 3, NULL);
#else
    xTaskCreate(phy_diag_task, "eth_diag", 3072, NULL, 3, NULL);
#endif
    uart_nmea("$PESP,ETH,INIT,OK," MACSTR, MAC2STR(eth_mac));
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
