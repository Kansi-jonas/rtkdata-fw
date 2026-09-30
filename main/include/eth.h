/*
 * RTKdata Ethernet uplink (K_EVK_V4.0: ESP32 EMAC + LAN8720AI over RMII).
 *
 * The Ethernet is the primary uplink on this board. While it holds an IPv4
 * the Wi-Fi STA is parked (wifi_sta_set_suspended), and the netif carries a
 * higher route priority than the STA, so every socket leaves over the cable.
 * Losing the IP (or the link) hands the uplink back to the STA.
 *
 * On a board without Ethernet every function is a cheap stub, so callers need
 * no #if.
 *
 * License: GPLv3 (see LICENSE).
 */
#ifndef RTKDATA_ETH_H
#define RTKDATA_ETH_H

#include <stdbool.h>
#include <stdint.h>
#include <esp_err.h>
#include <esp_netif_ip_addr.h>

typedef struct {
    bool     present;          // board has Ethernet and the driver came up
    bool     link;             // PHY reports a link
    bool     has_ip;           // DHCP lease (IPv4) on the Ethernet netif
    uint8_t  speed_mbps;       // 10 / 100, 0 without link
    bool     full_duplex;
    uint8_t  mac[6];
    esp_ip4_addr_t ip4_addr;
    esp_ip4_addr_t gw4_addr;
    uint32_t link_ups;         // link-up events since boot (flap counter)
    uint32_t symbol_errors;    // LAN8720 reg 26, running sum since boot
} eth_status_t;

// Bring up EMAC + PHY + netif and start the driver. Never aborts: a missing
// RMII clock or PHY returns an error and the device carries on over Wi-Fi.
esp_err_t eth_init(void);

// Wait up to timeout_ms for a PHY link (cable plugged and negotiated).
bool eth_wait_link(uint32_t timeout_ms);

bool eth_link_up(void);
bool eth_has_ip(void);

void eth_status(eth_status_t *out);

// Supervisor link-recovery rung for the Ethernet board: link up without an
// IP -> restart the DHCP client; no link -> restart the Wi-Fi driver (the STA
// is the fallback uplink). On a board without Ethernet: the Wi-Fi restart.
void eth_link_recover(void);

#endif // RTKDATA_ETH_H
