# K_EVK_V4.0 Ethernet board (firmware 2.x)

The K_EVK_V4.0 board (ESP32-WROOM-32UE-N16R2, LAN8720AI over RMII, optional
external PoE module on J9/J10) is the hardware generation after the OnoLink
Wi-Fi units. PoE and non-PoE are the same PCB with a different fitting, so
there is ONE image and ONE OTA channel for both.

The firmware is the same tree as the 1.x Wi-Fi fleet, built as a second
target:

| | OnoLink Wi-Fi fleet | K_EVK_V4 Ethernet |
|---|---|---|
| Build | `idf.py build` | `idf.py -B build_eth -DRTK_BOARD=k_evk_v4 build` |
| Project name / app | `rtkdata-fw` / `rtkdata-fw.bin` | `rtkdata-fw-eth` / `rtkdata-fw-eth.bin` |
| Version | `FW_VERSION` (1.x) | `FW_VERSION_ETH` (2.x) |
| OTA channel | `main/ota/` | `main/ota/eth/` |
| sdkconfig | repo root (gitignored) | `build_eth/sdkconfig`, from `sdkconfig.defaults` + `sdkconfig.defaults.k_evk_v4` |

The major version is the hardware generation. `board.h` fails the compile
if the CMake switch and the Kconfig choice disagree (a reused build dir).
`update.c` refuses any OTA image whose project name differs from the running
one, so a manifest that points at the wrong `.bin` cannot brick a unit.

Docker build (no local ESP-IDF needed):

```
docker run --rm -v "<repo>:/project" -w /project espressif/idf:v5.3.1 \
  idf.py -B build_eth -DRTK_BOARD=k_evk_v4 build
```

## Pin map

Source: schematic `K_EVK_V4.0_SCH0919` (2026-09-19). The vendor's IO handoff
sheet is wrong on the RMII clock (it says GPIO0 OUT). The LAN8720 runs in
REFCLKO mode and drives 50 MHz INTO GPIO0, which is the ESP-IDF default and
Espressif's recommended topology when Wi-Fi runs too.

| Function | GPIO | Notes |
|---|---|---|
| RMII REF_CLK in | 0 | from LAN8720 REFCLKO via R102 0R; boot strap |
| PHY reset (nRST) | 5 | external pull-down, PHY off until firmware releases it |
| MDC / MDIO | 23 / 18 | PHY address 0 |
| TXEN, TXD0, TXD1 | 21, 19, 22 | IO_MUX, fixed |
| RXD0, RXD1, CRS_DV | 25, 26, 27 | IO_MUX, fixed |
| GNSS UART (UM980 COM1) | TX 17 / RX 14 | unchanged from OnoLink |
| UM980 COM2 | TX 32 / RX 33 | not used yet |
| GNSS reset | 15 | low = reset (T6/D21/C58/T7); shared with I2C SDA, crypto chip not fitted |
| Button SW1 | 34 | input only, external pull-up, active low; also resets the UM980 in hardware |
| LED green (NTRIP) | 12 | high = on; MTDI strap, 47k pull-down |
| LED blue | 13 | high = on |
| 1PPS | 39 | not used yet |
| GPIO2 | 2 | UM980 EVENT, unused, never driven |
| PSRAM CS | 16 | in-package PSRAM (D0WDR2-V3), never touch |

Every OnoLink LED pin (21, 22, 19, 32, 27, 16), its GNSS reset (22) and its
button (13) collide with RMII, PSRAM or the LEDs here. That is why the two
boards can never share an image.

## Behaviour

Uplink:
- The Ethernet is the primary uplink (DHCP, hostname `rtkdata-xxxxxx` from
  the Wi-Fi MAC, same suffix as the setup AP `RTKdata_XXXXXX`).
- While the cable holds an IPv4 the Wi-Fi STA is parked: no connect, no
  reconnect loop. Link loss or lease loss hands the uplink back to the STA
  at once, a new lease parks it again.
- The Ethernet netif has route priority 128 (STA 100), so the cable wins
  whenever both have an IP.
- The device ID stays on the Wi-Fi STA MAC, as on the fleet.
- The setup AP keeps its 15 minute boot window. A unit with a cable and no
  Wi-Fi configured takes the provisioned boot path: OTA check first, then
  the web server, reachable on the LAN IP.
- A missing RMII clock or PHY is logged (`$PESP,ETH,INIT,FAIL,...`) and the
  unit carries on over Wi-Fi. It never aborts. The failure path then logs a
  one-line diagnosis (`$PESP,ETH,DIAG,...`): PHY ID over a bit-banged MDIO
  scan or "no PHY", REF_CLK toggling or absent, RMII outputs driven or
  floating.
- If no link has come up 15 s after start, one line reports whether the PHY
  sees energy on the wire (`$PESP,ETH,NOLINK,<energy>,<bmsr>`). No energy with
  a known-good cable points at the board's analog path.
- `esp_eth_start()` does not wait for autonegotiation; the link poll reports
  the link when it comes up.

Supervisor link rung: link up but no lease restarts the DHCP client, no link
restarts the Wi-Fi driver (the STA is the fallback). With PSRAM the heap
floor guards the internal RAM, not the total.

LEDs (red power and green PPS are hardware):

| LED | on | 1 Hz blink | 4 Hz blink | off |
|---|---|---|---|---|
| green (NTRIP) | caster streaming | uplink, not streaming yet | | no uplink |
| blue | Ethernet uplink | Wi-Fi uplink | setup AP open, no uplink | nothing |

During boot blue is on and green is off.

Button: every press resets the UM980 in hardware. On release the firmware
reconfigures the receiver (`$PESP,RTK,GNSS,BUTTONRESET`). Holding it 6 s
wipes the configuration and reboots.

Telemetry: the heartbeat's `health` carries `board`, `uplink`, `eth_link`,
`eth_speed`, `eth_link_ups` and `eth_sym_err` (outside the HMAC canon, stored
as-is by the IE). `/status` has an `eth` block, and `wifi.sta.parked`.

## Factory flash

`tools/flash_gui.py` lists `dist-ship-eth-*` and `build_eth` next to the
Wi-Fi folders, sorted by version. It checks the chip before erasing: the
Ethernet image needs a chip with embedded PSRAM (D0WDR2), the Wi-Fi image
refuses one.

## Bench acceptance (per sample)

Numbers fixed here, not negotiable per run:

1. Flash: 5 consecutive USB flashes through the auto-program circuit, 5/5 OK.
2. Strap soak: `python tools/eth_bench.py reset --port COMx --cycles 100`,
   100/100 `SPI_FAST_FLASH_BOOT`, then 20 power cycles by hand, 20/20 up with
   an Ethernet link.
3. Boot with cable: link up within 5 s of the banner, lease within 15 s,
   caster streaming (green steady) within 120 s.
4. LAN soak inside the AP window: `python tools/eth_bench.py net --ip <ip>
   --minutes 10`: 0 failed fetches, 0 link flaps, 0 symbol errors.
5. Failover: pull the cable while streaming, the STA must reconnect and the
   caster resume within 90 s. Plug it back in: the STA parks, the traffic
   moves back to the cable, no reboot.
6. Button: a short press while streaming brings the stream back within 60 s
   without a reboot. A 6 s hold does a factory reset.
7. 24 h soak on the cable: `reboots` 0, `rec_caster` at most 2, `eth_link_ups` 1.

## Sample findings (2026-09-29/30)

Two samples, both with blank flash from the factory, ESP32-D0WDR2-V3 rev 3.1,
16 MB flash, 2 MB PSRAM. The vendor says one has PoE, one does not.

Firmware verified on the samples: PSRAM, board banner, UM980 configuration
through the GPIO15 reset (15/16 commands acked, 1230 is the known receiver
reject), LEDs on GPIO12/13, and the fallback to Wi-Fi when the Ethernet does
not come up.

Sample without PoE (MAC ...a0:44): no PHY at all. No REFCLKO on GPIO0, no
MDIO answer on any address, RXD0/RXD1/CRS_DV undriven after nRST release.
Probably not populated; not confirmed.

Sample with PoE (MAC ...a0:30): PHY alive (ID 0007:C0F1 at address 0,
REFCLKO present, EMAC driver starts), but **no link**. On a cable and router
port that gave a PC a 1 Gbit link seconds before, ENERGYON (reg 17) stays 0
and autonegotiation never completes. A firmware matrix of autonegotiation
(full, 10-only) and forced 10/100 half duplex, each with Auto-MDIX, forced MDI
and forced MDIX, showed no energy in any mode. Cause in the schematic: the
PHY-side center taps of T10 (pins 2 and 7) only have capacitors to GND, and
the four 49.9R pull-ups on TXP/TXN/RXP/RXN are missing. Microchip's LAN8720A
schematic checklist (Rev D, Fig. 1/2) and datasheet (Fig. 3-18) require both
center taps on 3.3V and 4x 49.9R 1% to 3.3V. No firmware setting replaces
them. An independent review (Codex, 2026-09-30) confirmed the schematic
noncompliance and found three more deviations: no 1.5k MDIO pull-up, no 500R
series resistor on XTAL2 for the 100 uW crystal, and H1102NL magnetics that
Pulse lists as non-PoE.

Strap: the PoE sample entered DOWNLOAD_BOOT in 7 of 12 EN resets (boot:0x6);
a normal boot reads boot:0x16 on this board, not 0x13. The PHY is not held in
reset reliably with R101 = 10k (ESP32 GPIO5 pull-up plus the LAN8720's own
50 uA nRST pull-up), so REFCLKO toggles GPIO0 while the straps are sampled.
Firmware cannot help: power-on, EN, RTC watchdog and brownout resets sample
the straps before any code runs. R101 = 4.7k is agreed with the vendor but
not validated; measure nRST and GPIO0 on a scope during reset.

## Open

- OTA channel `main/ota/eth/` does not exist yet; publishing needs its own
  release/promote path (the current workflows only know `main/ota/`). Until
  then a 2.x unit finds no manifest and stays on its version.
- Static IP on the Ethernet (today DHCP only).
- UM980 COM2 (GPIO32/33) for on-device NMEA/quality and 1PPS (GPIO39) unused.
- Vendor: PoE module type (802.3af/at, isolation), CE/RED/FCC, R101 to 4.7k
  on the series boards, corrected handoff sheet.
- Web UI does not render the `eth` block yet (JSON only).
