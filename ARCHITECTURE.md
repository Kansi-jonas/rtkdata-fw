# RTKdata Reference Firmware - Architecture

RTKdata reference-station firmware for the **Unicore UM980 + ESP32** hardware (OnoLink miner class).
It turns the receiver into a managed RTCM3 reference base that streams to the RTKdata caster and is
onboarded, positioned, and lifecycle-managed by the Integrity Engine (IE).

## Lineage

Forked from the OnoLink firmware, itself a fork of nebkat's **ESP32-XBee** (Ardusimple WiFi NTRIP
Master), ESP-IDF / FreeRTOS. The networking, NVS config store, web server, and multi-instance NTRIP
server are inherited and kept. Everything identity-, stability-, and provisioning-related is RTKdata's.

> License: the upstream is **GPLv3** (`LICENSE`). This fork stays GPLv3. Keep the upstream copyright
> headers; new RTKdata files carry the same license.

What this fork changes versus stock OnoLink:

| Area | OnoLink | RTKdata FW |
|---|---|---|
| OTA source | `gitlab.com/moonsystems1/onolink-2` | RTKdata-owned (`ota/release.json`), fully decoupled |
| Identity | `OnoLink_xxxx` AP, `ESP32-XBee` NTRIP agent | `RTKdata_xxxx` AP, `RTKdata_Server` agent |
| 1005 (base ARP) | **disabled** (`gnss.c`, commented out) | **enabled** - the station identity anchor |
| Survey-in | blind 60 s auto-survey (~1-2 m) | provisional; precise position computed server-side |
| Stability | local reconnects only, no end-to-end self-heal | **health supervisor** with escalation ladder |
| Onboarding | generic Angular config SPA | focused RTKdata Wi-Fi onboarding + 48 h maturing UI |
| Lifecycle | manual config | **zero-touch** enroll + IE-managed credentials |
| DB registration | n/a | **gated**: station created only after 48 h clean data + converged position |

## 1. Independent OTA pipeline

Self-hosted, decoupled from OnoLink. Mechanism is the inherited `update.c` (HTTPS + ESP cert bundle):
the device fetches `UPDATE_SERVER_URL + release.json`, compares `version` against the compiled
`FW_VERSION`, and on mismatch downloads each URL in `update_files_urls` (app `*.bin` -> OTA partition,
`www.bin` -> SPIFFS -> www partition), then reboots.

- `UPDATE_SERVER_URL` = `https://raw.githubusercontent.com/Kansi-jonas/rtkdata-fw/main/ota/` (see
  `main/include/config.h`). Swap to a CDN / S3 / the IE later without code change.
- Manifest: [`ota/release.json`](ota/release.json).
- Cutting a release: bump `FW_VERSION` in `config.h`, `idf.py build`, publish `rtkdata-fw.bin` +
  `www.bin` under `ota/`, bump `version` in `release.json`. Daily 00:00 check + boot check (`main.c`).
- Roadmap: signed manifests (HMAC over `release.json`) and staged rollout keyed by `device_id`.

## 2. Stability ("maximal stabil") - the health supervisor

Root cause of the field "goes offline, needs a restart" reports: **there is no end-to-end self-heal
for the data path.** Every recovery in stock firmware is local (Wi-Fi reconnect, socket reconnect on a
send error) and nothing supervises "is RTCM actually reaching the caster." Specific failure modes:

1. **Suspend-forever** (`interface/ntrip_server.c`): after connect the server task does
   `vTaskSuspend(NULL)` and is only resumed by a *send error*. Sends only happen while UART data
   flows. If the GNSS feed stalls, the keep-alive clears `DATA_READY_BIT` ("will not reconnect"), no
   send is attempted, a dead socket is never detected, and the task is parked forever -> dark until
   reboot.
2. **No socket read** after the NTRIP handshake -> half-open TCP (NAT timeout, caster restart without
   RST) is invisible until a write fails, which (see 1) may never happen.
3. **GNSS has no self-heal**: if the UM980 stops emitting (brownout, reset glitch, stuck re-survey,
   half-applied open-loop config) nothing resets it.
4. **Event-loop stall under dual-cast**: the UART handler holds a mutex and does blocking 500 ms
   writes per instance inside the default event loop; this backs up with each extra cast target.

### Design: `supervisor.c` / `supervisor.h` (new)

A single high-priority task owns data-path liveness via three monotonic timestamps updated from the
hot paths:

- `t_gnss_rx`  - last byte from the UM980 (hooked in `uart_task`).
- `t_caster_tx` - last **successful** caster send (hooked in `ntrip_server_send_data`).
- `t_sta_ip`   - last time STA had an IP (hooked in `wifi.c`).

Escalation ladder (each rung only fires if cheaper rungs did not recover):

| Symptom | Threshold | Action |
|---|---|---|
| no GNSS byte | > 10 s | reset UM980 (GPIO) + re-run `config_gnss_base()` with ACK gating |
| no caster send | > 30 s | force-close socket, resume the NTRIP server task to reconnect |
| no STA IP | > 60 s | restart the Wi-Fi driver (don't wait the stock 5 min / 60 attempts) |
| total wedge | > 120 s | `esp_restart()` - last resort, reason logged for the IE |

Plus two correctness fixes in `ntrip_server.c`:
- replace `vTaskSuspend(NULL)` with a **bounded** wait + an idle socket probe (non-blocking
  `recv` / `MSG_PEEK`) so a half-open socket is detected without depending on UART flow;
- decouple "no UART data" from "must not reconnect" - the caster uplink is held/re-established
  regardless of a transient GNSS stall.

The supervisor also exposes a `health_snapshot()` consumed by the provisioning heartbeat (section 4),
so the IE sees the same liveness the device acts on.

## 3. GNSS hardening (`gnss.c`)

- **Enable 1005** (the base ARP): re-enabled and emitted at a tight cadence. 1005 is the station
  identity anchor - without it the served base is unidentifiable downstream (the used != served /
  missing-baseline problem). The precise coordinate is *not* trusted from the device.
- **ACK-gated config** instead of the blind `vTaskDelay(500ms)` open loop: read the UM980 response
  and verify each command (esp. `mode base`) actually took; retry on miss. This is the difference
  between "usually configures" and "provably configured."
- **Position is server-side**: the 60 s survey-in is provisional only. Once the IE delivers the
  converged coordinate (section 4) the device sets `mode base <lat> <lon> <h>` and broadcasts an
  authoritative 1005. (Downstream, Lighthouse also rewrites 1005 to the catalog coordinate, so the
  device only ever needs to broadcast *a* 1005; the server owns the exact value.)

## 4. Zero-touch provisioning + the 48 h DB-gating contract

The headline lifecycle rule: **a station is created in the database only after ~48 h of clean data AND
a converged, centimetre-accurate position.** Until then the device is enrolled but *provisional* - it
streams to a staging mountpoint and is not part of the live RTKdata network.

### Device side: `provisioning.c` / `provisioning.h` (new)

Reuses the `esp_http_client` + `esp_crt_bundle` pattern from `update.c`. Device identity is the MAC
(already the basis of the `RTKdata_xxxx` AP SSID). State machine, persisted in NVS:

```
  UNCLAIMED --enroll--> PROVISIONING --stream clean--> MATURING --48h+converged--> ACTIVE
                              ^                              |
                              +---------- stream gap --------+   (resets the clean-data window)
```

- On first STA-online, `enroll` with the IE; receive provisional caster credentials, write them to the
  inherited NVS NTRIP keys (`ntr_srv_host/port/mp/user/pass`), and trigger the NTRIP server to connect
  (it re-reads config every loop iteration - clean injection point).
- `heartbeat` every `ie_poll_s` carries the supervisor `health_snapshot` + GNSS quality (sats, fix,
  HDOP, rate). The IE replies with `{state, progress_pct, matures_in_s, position?, config_delta?}`.
- `config_delta` lets the IE re-issue credentials, reposition (push the converged coord ->
  `mode base <lat lon h>`), blacklist, or move the device to the production mountpoint at go-live -
  this is what "lives cleanly on the RTKdata caster and is managed" means.
- The device serves `/rtk/status` locally (consumed by the onboarding UI) reflecting the last
  heartbeat reply + local quality.

### IE side (contract; implemented in the integrity-engine repo)

HMAC-signed (shared-secret) JSON over HTTPS, same discipline as the existing LH <-> IE loop.

```
POST /api/edge/enroll
  body: { device_id, mac, fw_version, hw:"um980", nonce, hmac }
  ->    { station_token, caster:{host,port,mountpoint,user,pass}, ie_poll_s }
  effect: create a PROVISIONAL device record. NOT a live station. Staging mountpoint only.

POST /api/edge/heartbeat
  body: { device_id, station_token, uptime, health:{...}, quality:{sats,fix,hdop,rate}, nonce, hmac }
  ->    { state, progress_pct, matures_in_s, position?:{lat,lon,h,valid}, config_delta? }
```

Promotion logic (PROVISIONING/MATURING -> ACTIVE) requires **both**:
1. **Time**: continuous clean-data window >= 48 h. A stream gap, position jump, or quality drop resets
   the window (`clean_since`).
2. **Position**: server-side PPP/PPK has converged (e.g. horizontal sigma < 1 cm, stable over the last
   N hours).

`progress_pct = min(time_progress, convergence_progress)` so the UI ring shows the binding constraint.
On ACTIVE the IE creates the real `station_coords` (Tier-1) record and provisions the production
`RTK_<id>_ITRF2020` mountpoint through the existing cfg-generator, then pushes the go-live
`config_delta`. This reuses the IE pipeline already in production for the GEODNET/ONOCOY catalog.

### New NVS config keys (added to `config.c` / `config.h`)

```
rtk_ie_host      string   IE enrollment host (default: api.rtkdata.com)
rtk_device_id    string   stable device id (default: derived from MAC)
rtk_station_tok  string   secret, station token from enroll
rtk_state        uint8    0 UNCLAIMED .. 4 ACTIVE (persisted state machine)
rtk_clean_since  uint32   epoch of the current clean-data window start (IE-authoritative mirror)
rtk_ota_base     string   OTA base url override (default: compile-time UPDATE_SERVER_URL)
```

## 5. Web onboarding UI (`www/index.html`)

Single self-contained file (no CDN - the setup AP has no internet), served from the SPIFFS `www`
partition by the inherited web server. Talks to existing endpoints (`/config`, `/wifi/scan`,
`/status`) plus the new `/rtk/status`:

1. Identity (device id + firmware from `/config`).
2. Wi-Fi scan/select/password -> `POST /config {w_sta_ssid,w_sta_pass}` (device reboots into STA).
3. Robust "joining" poll of `/status` that tolerates the reboot gap.
4. Online -> 48 h **maturing ring** + live quality (sats, fix, uplink, uptime) from `/rtk/status`.
   Degrades gracefully to "provisioning" when `/rtk/status` is not yet served.

## Module map

```
main/
  main.c                 boot/wiring (+ supervisor & provisioning init)   [edit]
  interface/ntrip_server.c   NTRIP caster push (suspend-fix, socket probe) [edit]
  gnss.c                 UM980 config (1005 on, ACK-gated, mode base)      [edit]
  uart.c                 GNSS UART feed (t_gnss_rx hook)                   [edit]
  wifi.c                 STA/AP (t_sta_ip hook, RTKdata_ AP)              [edit]
  update.c               OTA (RTKdata release channel)                    [done]
  web_server.c           HTTP + /rtk/status route                         [edit]
  supervisor.c/.h        data-path health + escalation ladder             [new]
  provisioning.c/.h      IE enroll + heartbeat + 48h state machine        [new]
  config.c / include/config.h  NVS schema (+ rtk_* keys, RTKdata identity) [edit]
ota/release.json         OTA manifest (RTKdata channel)
www/index.html           RTKdata onboarding + maturing UI                 [new]
```

Status: identity/OTA decoupling, onboarding UI, and this contract are in. The `supervisor` and
`provisioning` C modules + the `gnss.c` / `ntrip_server.c` edits are the next implementation pass.
