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
#if defined(RTK_CMAKE_BOARD_K_EVK_V41) != defined(CONFIG_RTK_BOARD_K_EVK_V41)
#error "RTK_BOARD (CMake) and CONFIG_RTK_BOARD_* (sdkconfig) disagree: use a fresh build dir"
#endif

/* The Ethernet boards run out of internal RAM without Wi-Fi/lwIP buffers in
 * PSRAM (2.1.0 on the first V4.1 hardware: tasks not created, supervisor
 * reboot loop). sdkconfig.defaults.k_evk_v4* set both values, but a build dir
 * from before 2.1.1 keeps its old sdkconfig, which defaults never override. */
#if CONFIG_RTK_BOARD_K_EVK && \
    (!CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP || CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL > 4096)
#error "Ethernet board without the PSRAM memory settings: use a fresh build dir"
#endif

#if CONFIG_RTK_BOARD_K_EVK_V41

/*
 * K_EVK_V4.1, schematic K_EVK_V4.0_SCH_1013 (2026-10-13) plus the fixes asked
 * for on 13.10. (FB5 fitted, 1k pull-up on GPIO13, U6 fitted). Differences to
 * V4.0: the LAN8720 runs in REF_CLK In mode from oscillator Y7, the GNSS reset
 * moved from GPIO15 to GPIO13 (GPIO15 is now I2C SDA only), the blue status
 * LED is gone and a Wi-Fi LED sits on GPIO32 (the former COM2 TX line).
 *
 * The same image is safe on a reworked V4.0 sample: GPIO2 high only enables
 * the oscillator (or drives the unused UM980 EVENT pin), GPIO13 high keeps the
 * receiver running or lights the old blue LED, GPIO32 toggles the unused COM2.
 */
#define BOARD_NAME                  "k_evk_v41"
#define BOARD_HAS_ETH               1
#define BOARD_HAS_RGB_STATUS_LED    0

/* SW1 on GPIO34, unchanged: R77 4.7k pull-up, active low, also resets the
 * UM980 in hardware (T8/D22); the ESP32 is not reset (R79 not fitted). */
#define BOARD_BUTTON_GPIO           GPIO_NUM_34
#define BOARD_BUTTON_PRESSED_LEVEL  0

/* UM980 reset through T6/D21/C58/T7 as before, now driven by GPIO13 (MTCK,
 * not a strap). Same polarity: low = reset, high = run. GPIO13 has an internal
 * pull-down after reset, so the board needs its own 1k pull-up to keep the
 * receiver running through ESP32 resets (asked for on 13.10.). */
#define BOARD_GNSS_RESET_GPIO       GPIO_NUM_13

/* Two firmware LEDs, digital NPN low side, high = on. GPIO12 (MTDI strap,
 * R64 47k pull-down) as before; GPIO32 is not a strap. Power, PoE and PPS
 * are hardware only. */
#define BOARD_LED_NTRIP_GPIO        GPIO_NUM_12
#define BOARD_LED_WIFI_GPIO         GPIO_NUM_32

/* LAN8720AI in REF_CLK In mode (nINTSEL high via R100): oscillator Y7
 * (YXC YSO110TR 50 MHz) feeds U7 CLKIN through R113 and GPIO0 through R160.
 * Its tri-state pin is GPIO2 with R107 4.7k to GND: the output is high
 * impedance at every chip reset, so GPIO0 sees only its R71 pull-up while the
 * straps are latched. eth_init() holds nRST low, enables the clock, waits for
 * the start-up time (3 ms max per YXC) and only then lets the driver pulse
 * nRST, because the LAN8720 needs CLKIN running during reset (DS 3.8.5.1). */
#define BOARD_ETH_MDC_GPIO          23
#define BOARD_ETH_MDIO_GPIO         18
#define BOARD_ETH_PHY_ADDR          0
#define BOARD_ETH_PHY_RST_GPIO      5
#define BOARD_ETH_CLK_EN_GPIO       GPIO_NUM_2
#define BOARD_ETH_CLK_START_MS      5

/* Inputs only, not used by the firmware yet: GPIO36 is low while the PoE
 * module delivers voltage (T12 open collector, R166 pull-up); GPIO35 reads
 * the revision divider (10k/10k = about 1.65 V on this revision). */
#define BOARD_POE_SENSE_GPIO        GPIO_NUM_36
#define BOARD_REV_ID_GPIO           GPIO_NUM_35

/* UM980 1PPS (input only), I2C to the crypto chip U6 (R78/R65 2.2k pull-ups). */
#define BOARD_PPS_GPIO              GPIO_NUM_39
#define BOARD_I2C_SDA_GPIO          GPIO_NUM_15
#define BOARD_I2C_SCL_GPIO          GPIO_NUM_4

#elif CONFIG_RTK_BOARD_K_EVK_V4

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
