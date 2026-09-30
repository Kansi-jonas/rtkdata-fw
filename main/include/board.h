/*
 * RTKdata board pin map. One header, two boards; the Kconfig choice
 * RTK_BOARD picks the block. Every GPIO the firmware drives must come from
 * here: on the Ethernet board the old OnoLink pins land on RMII, PSRAM and
 * LED lines, so a stray hardcoded pin is a hardware fault, not a style issue.
 *
 * K_EVK_V4.0 source: schematic K_EVK_V4.0_SCH0919 (2026-09-19). The vendor's
 * IO handoff sheet is wrong on the RMII clock direction (it says GPIO0 OUT);
 * the schematic and the vendor's own correction say the LAN8720 drives
 * REFCLKO into GPIO0, which is the ESP-IDF default (CONFIG_ETH_RMII_CLK_INPUT).
 *
 * License: GPLv3 (see LICENSE).
 */
#ifndef RTKDATA_BOARD_H
#define RTKDATA_BOARD_H

#include "sdkconfig.h"
#include "driver/gpio.h"

/* The CMake -DRTK_BOARD switch (project name, OTA channel, sdkconfig
 * defaults) and the Kconfig choice (pins) must agree. A build dir reused
 * across boards keeps its old sdkconfig; fail the compile instead of shipping
 * an image with one board's name and the other board's pins. */
#if defined(RTK_CMAKE_BOARD_K_EVK_V4) != defined(CONFIG_RTK_BOARD_K_EVK_V4)
#error "RTK_BOARD (CMake) and CONFIG_RTK_BOARD_* (sdkconfig) disagree: use a fresh build dir"
#endif

#if CONFIG_RTK_BOARD_K_EVK_V4

#define BOARD_NAME                  "k_evk_v4"
#define BOARD_HAS_ETH               1
#define BOARD_HAS_RGB_STATUS_LED    0

/* SW1 on GPIO34: input-only, R77 4.7k external pull-up, active low. The
 * button ALSO resets the UM980 in hardware (through D22) on every press, the
 * ESP32 is not reset (R79 not fitted). */
#define BOARD_BUTTON_GPIO           GPIO_NUM_34
#define BOARD_BUTTON_PRESSED_LEVEL  0

/* UM980 reset through T6/D21/C58/T7, low = reset, R78 keeps it high at boot.
 * Shares the net with I2C SDA; the crypto chip U6 is not fitted. */
#define BOARD_GNSS_RESET_GPIO       GPIO_NUM_15

/* Two firmware LEDs, NPN low side, high = on. Red power and green PPS are
 * hardware only. GPIO12 is the MTDI strap: R64 47k pulls it low, and we only
 * drive it after boot. */
#define BOARD_LED_NTRIP_GPIO        GPIO_NUM_12
#define BOARD_LED_STATUS_GPIO       GPIO_NUM_13

/* LAN8720AI, RMII. MDC/MDIO and the data lines are the ESP32 IO_MUX pins
 * (TXEN 21, TXD0 19, TXD1 22, RXD0 25, RXD1 26, CRS_DV 27), fixed in
 * silicon. PHY address strapped to 0. nRST has an external pull-down, so the
 * PHY (and with it the 50 MHz REFCLKO into GPIO0) stays off until GPIO5
 * releases it; esp_eth_driver_install() does that before the EMAC init. */
#define BOARD_ETH_MDC_GPIO          23
#define BOARD_ETH_MDIO_GPIO         18
#define BOARD_ETH_PHY_ADDR          0
#define BOARD_ETH_PHY_RST_GPIO      5

/* GPIO2 goes to UM980 EVENT/STDBY_IN and is unused ("Not used", vendor).
 * It stays in its reset default; never drive it high (download-mode strap). */

#else /* OnoLink, the Wi-Fi fleet */

#define BOARD_NAME                  "onolink"
#define BOARD_HAS_ETH               0
#define BOARD_HAS_RGB_STATUS_LED    1

#define BOARD_BUTTON_GPIO           GPIO_NUM_13
#define BOARD_BUTTON_PRESSED_LEVEL  1

#define BOARD_GNSS_RESET_GPIO       GPIO_NUM_22

#endif

#endif // RTKDATA_BOARD_H
