# RTKdata Reference Firmware

Managed reference-station firmware for the **Unicore UM980 + ESP32** (OnoLink-class hardware).
Turns the receiver into an RTCM3 base that streams to the RTKdata caster and is onboarded,
positioned, and lifecycle-managed by the Integrity Engine (IE).

- **Maximally stable**: an end-to-end data-path health supervisor with an escalation ladder
  (GNSS reset -> socket reconnect -> Wi-Fi restart -> reboot), fixing the "goes offline, needs a
  restart" failure modes of the stock firmware.
- **Schicke Onboarding-UI**: a self-contained RTKdata Wi-Fi onboarding flow served from the device,
  with a live 48 h "maturing" view.
- **Zero-touch provisioning**: the device enrolls with the IE, pulls its caster credentials, and is
  managed centrally.
- **48 h DB-gating**: a station is registered in the live network **only after ~48 h of clean data
  and a converged, centimetre-accurate position** (computed server-side).
- **Independent OTA**: self-hosted release channel, fully decoupled from the OnoLink firmware.

See **[ARCHITECTURE.md](ARCHITECTURE.md)** for the full design and the device <-> IE contract.

## Hardware

- Unicore **UM980** GNSS receiver (NovAtel/Unicore command dialect), RTCM3 MSM7 + 1005 + 1033.
- **ESP32** (ESP-IDF / FreeRTOS), UART to the UM980, Wi-Fi STA+AP.

## Build & flash (ESP-IDF)

```bash
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```

The web UI ships in the `www` SPIFFS partition (see `partitions.csv`). Build the `www.bin` image from
`www/` and flash it to the `www` partition, or push it via OTA (`ota/release.json`).

## Onboarding

1. Power the station. It opens a Wi-Fi AP named `RTKdata_XXXXXX`.
2. Connect and open `http://192.168.4.1` -> the onboarding UI.
3. Pick your Wi-Fi, enter the password. The station joins your network and enrolls with RTKdata.
4. Watch the 48 h maturing ring. The station goes live automatically once its position is locked and
   its data is clean.

## OTA (independent channel)

Configured by `UPDATE_SERVER_URL` in `main/include/config.h`, pointing at this repo's `ota/`.
The device fetches `ota/release.json`, compares `version` to the compiled `FW_VERSION`, and updates
app + UI partitions on mismatch (boot check + daily 00:00).

To cut a release: bump `FW_VERSION`, `idf.py build`, publish `rtkdata-fw.bin` + `www.bin` under
`ota/`, bump `version` in `ota/release.json`.

## Repository layout

```
main/                 firmware sources (see ARCHITECTURE.md "Module map")
www/index.html        self-contained onboarding + maturing UI
ota/release.json      OTA manifest (RTKdata channel)
partitions.csv        flash layout (factory + ota_0/1 + www/spiffs + coredump)
ARCHITECTURE.md       design, stability supervisor, IE 48h-gating contract
```

## License

GPLv3. Forked from the OnoLink firmware (fork of nebkat's
[ESP32-XBee](https://github.com/nebkat/esp32-xbee)). Upstream copyright headers are preserved; new
RTKdata sources carry the same license.
