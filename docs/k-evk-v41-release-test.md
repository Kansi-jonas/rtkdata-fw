# K_EVK_V4.1 release test (PO 6: 100 Ethernet + 100 PoE sets)

Three stages. Stage 1 is the gate in the purchase order: mass production of
the PCBA starts only after the two pre-production units pass it. Stage 2 runs
at the vendor on every unit, stage 3 here on every unit.

Numbers are fixed here and not negotiated per run. Tools: `tools/eth_bench.py`
(`reset`, `net`, `soak`), `tools/bench_caster.py` (local caster with raw log),
`tools/rtcm_cnr.py` (C/N0 comparison), the board test build
(`-DRTK_TEST_PAGE=1`, `/test.html`), `tools/provision_device.py`.

## Pre-production units

The two samples the vendor sent in November are the stage 1 units (decision
14.11.): V4.0 boards the vendor reworked to the V4.1 design.

| Unit | Set | ETH MAC | Station ID | FTDI |
|---|---|---|---|---|
| A | PoE | 28:56:2f:ac:a0:53 | RTK28562FACA050 | DK0E4YJC |
| B | Ethernet | 28:56:2f:ac:a0:23 | RTK28562FACA020 | DK0E5H98A |

Both read the revision ID at about 143 mV ("no divider"; a V4.1 board reads
about 1650 mV), unit B reports PoE sense with LAN only (pin floating), and
neither carries the final LED panel. Unit A enables the 50 MHz oscillator on
GPIO2 and streams through the IE since 13.11. (2.1.4, enrolled).

Items marked **V4.1** that the samples cannot show (revision ID, PoE sense,
LED panel) are covered by F2 on every production unit. Proposed 14.11., to
agree with the vendor: the F2 reports of the first ten units come to us
before the rest is packed, and a failure there stops the run.

## Stage 1: design validation (gate before mass production)

Units: the two pre-production units above, 1 Ethernet set and 1 PoE set.
Firmware: the release candidate for the eth41 channel; the test build only
for D1, D10, D11. Unit A runs the D16 soak, so the tests that reboot or cut
the uplink run on unit B (or on unit A after the soak).

| ID | Test | How | Pass |
|---|---|---|---|
| D1 | Identity **V4.1** | test page | revision ID 1650 mV +-10 %, chip v3.1, 16 MB flash, 2 MB PSRAM |
| D2 | USB flash | 5 flashes through the auto-program circuit, then 5 more with the Ethernet cable plugged and the unit streaming | 10/10, serial terminal opens while Ethernet runs |
| D3 | Reset soak **V4.1** | `eth_bench.py reset --cycles 100`; 20 power cycles by hand; 20 EN resets while streaming over Ethernet | 100/100 flash boot; 20/20 link; 20/20 streaming again within 120 s. This is the EN-reset hang of 12.10. that the oscillator fixes |
| D4 | Boot with cable | serial log | link within 5 s of the banner, lease within 15 s, streaming within 120 s |
| D5 | LAN load with the setup AP on | `eth_bench.py net --minutes 10` within 15 min of boot | 0 failed fetches, 0 link flaps, 0 symbol errors |
| D6 | Long cable | 50 m Cat6a, `net --minutes 10`. With a managed switch also forced 10 Mbit full on both ends | 0 / 0 / 0; at 10 Mbit 0 failed fetches. The series board has the CT/49.9 R fix the samples lacked |
| D7 | PoE **V4.1** | TL-PoE150S (802.3af) plus 50 m: boot, `net --minutes 10`; 10 PoE unplug/replug; USB plugged and pulled 5x each while on PoE; SW2 off for 10 min, then on | boots, blue LED on, test page PoE sense = PoE; 0 / 0 / 0; 10/10 up; no reboot from USB; after SW2 on the unit boots within 10 s. Record whether the injector cycles while SW2 is off (DP1435H wants 100 mA minimum load) |
| D8 | Failover | Wi-Fi configured; pull the cable while streaming, plug it back | streaming over Wi-Fi within 90 s; back on the cable, STA parked, no reboot |
| D9 | Power loss | 20 power cuts at random times while streaming | 20/20 streaming again within 120 s, same device in the IE, enroll key still there (SOTA item O3 is open) |
| D10 | Button and GNSS reset **V4.1** | test page reset test; short press while streaming; 10 EN resets; 6 s hold | reset test PASS (PPS pauses); stream back within 60 s without ESP32 reboot; PPS keeps running through all 10 EN resets; 6 s hold clears the Wi-Fi setup only (SSID, password, static mode); the enroll key, station token, caster and fixed coordinate stay |
| D11 | Crypto chip **V4.1** | test page ATECC wake probe | PASS |
| D12 | LEDs **V4.1** | compare with the panel definition of 16.10. | Power red = powered; Online green: on streaming, 1 Hz network up not streaming, off no network; GNSS yellow = PPS; Wi-Fi white: on Wi-Fi uplink, 4 Hz setup AP open, off otherwise; PoE blue = PoE present. All five about equally bright through the panel, daylight and dark room |
| D13 | Antenna chain | 6618 antenna on the roof, 30 min with the 9 m set cable and 30 min with the shortest cable, both recorded with `bench_caster.py --log`; `rtcm_cnr.py short.rtcm3 set9m.rtcm3` | every signal tracked with the short cable is also tracked with the 9 m cable, no median more than 2 dB-Hz lower |
| D14 | End to end on the IE | enroll, survey, coordinate push from the IE, then decode the box's own 1005 | 1005 equals the IE coordinate within 0.01 m in ECEF (no geoid offset like the 1.1.5 boxes of 13.11.); antenna descriptor as the IE expects for the 6618 |
| D14b | Moved unit | after D14, move the powered antenna more than 20 m (long antenna cable, or unit and antenna on a power bank). A pushed coordinate cannot stand in for the move: the agreement gate refuses it before MODE. Repeat once while the unit is still in its own survey (before any IE coordinate) | within 7 min of the move: `$PESP,RTK,GNSS,RELOCATED`, the fixed coordinate is gone from NVS, survey-in starts, and RTCM is back about 5 min later; the old IE coordinate is then refused (`FIXEDBASE,REFUSED`, heartbeat health `moved_m` > 0). The IE does not re-survey a moved station on its own today (IE follow-up), so reset its coordinate by hand after the test. Also once on the bench: serial log across one survey from the MODE ACK to RTCM output, with every BASEPOS and 1005; a unit that does not move must show no `RELOCATED` (whether the UM980 sends interim 1005s during the survey is not documented by Unicore) |
| D15 | OTA over Ethernet | publish 2.1.x+1 on the eth41 channel, unit on cable only, STA parked; second run with the cable pulled mid-download | installs, one reboot, reports the new version; interrupted download keeps the old version running. **Blocked today:** the channel `main/ota/eth41/` and its promote path do not exist (13.11. boot log: "invalid manifest") |
| D16 | 72 h soak | streaming on the cable, `eth_bench.py soak --hours 72` | tool says PASS: 0 reboots, 0 link flaps, 0 symbol/CRC/UART errors, at most 6 caster reconnects, at most 1 % of polls without the full message set. 72 h covers at least two DHCP renewals on a 24 h lease |
| D17 | Data volume | MB/day from the D16 summary | record only (the 2 kB/s for the 4G kit was never measured) |

D15 must be closed before the first unit ships: without OTA every field fix
is a USB reflash at the customer, as with the three 1.1.5/1.0.0 boxes today.

## Stage 2: factory test at the vendor (every unit)

Target about 5 minutes per unit. The vendor sends the test reports and one
CSV with the shipment.

| ID | Step | Pass |
|---|---|---|
| F1 | FT_Prog: CBUS3 = VBUS_Sense, max bus power 500 mA | set and read back |
| F2 | Flash the test build, open `/test.html` over the LAN (PoE units powered by a PoE injector, not USB) | Ethernet 100 Mbit full with IP; Wi-Fi scan finds networks; UM980 version read; PPS present with an antenna; GNSS reset test PASS; ATECC PASS; button counted; PoE sense matches the variant; revision ID V4.1; each LED on and off by override. Save the report |
| F3 | Flash the production image with the Kansi factory tool | the tool prints the station ID (RTK + 12 characters) |
| F4 | Print two labels with the station ID as text and QR code: device bottom, box outside | both labels scan as the printed ID |
| F5 | Power cycle on the LAN | link LEDs on the jack; Wi-Fi setup AP `RTKdata_XXXXXX` visible, suffix matches the label |
| F6 | Pack the set, add the CSV row | station ID, Ethernet MAC, variant (ETH/PoE), test result, date, box number |

The vendor never gets the enroll master. The factory tool writes firmware
only; the per-device key goes in at stage 3.

## Stage 3: incoming test in Namborn (every unit)

| ID | Step | Pass |
|---|---|---|
| I1 | Scan the box label and the device label | same ID, ID is in the vendor CSV |
| I2 | `provision_device.py --port COMx --flash` | the ID the tool reads from the chip equals the label; otherwise quarantine the unit |
| I3 | Power on with LAN, on an isolated LAN WITHOUT internet (switch plus the bench PC as DHCP server, `bench_dhcp.py`) | link, IP, setup AP `RTKdata_` + the last six characters of the station ID on the label (2.1.4 and later on freshly provisioned NVS, which I2 always writes; a unit that first ran an older image keeps the stored AP-MAC name, one higher). The unit must not reach the IE here: with the key from I2 it would enroll at once, and a few hours of data let the IE survey it in Namborn |
| I4 | Burn-in sample, before I2: 10 units per variant, picked at random; `bench_point_caster.py --port COMx --host <PC IP>` (writes the caster into NVS, the web UI no longer takes it), `bench_caster.py`, `eth_bench.py soak --hours 24` | all PASS. One FAIL: burn-in of every unit of that variant, failed unit back to the vendor for root cause. I2 afterwards rewrites NVS and removes the bench caster |

Why a sample in I4 is enough: random part defects show up in F2 and I3 on
every unit; the soak finds systematic faults (layout, firmware, PoE module),
which hit all units alike, so ten per variant find them. Burn-in runs against
the local caster, not the IE: a unit enrolled at our office would start its
survey here and carry the Namborn coordinate to the customer.
