# Ship Runbook: RTKdata Edge Device (UM980 / ESP32)

Repeatable process to flash, provision, ship, and bring up a device end-to-end.
Cross-references: docs/enroll-per-device-key.md (security), docs/fw-correction-flow.png (flow).

## 0. One-time server-side prerequisites (do ONCE before any shipping)

- [ ] **IE `EDGE_ENROLL_MASTER_V1`** set on Render (64-hex master). Generate locally
      with `openssl rand -hex 32`; keep it in the password manager + on the flash host.
      NEVER commit / log it. (Without it, per-device enroll cannot be verified by the IE.)
- [ ] **IE `EDGE_CASTER_HOST/PORT/USER/PASS`** set (the upload caster the device pushes to).
      Sanity: the existing test device uploads, so these are effectively correct already.
- [ ] **Caster accepts new mountpoints** (THE blocker, [[caster-cfg-sync-stale]]): the
      caster box must pull `s3://rtkdata-ntrips-configs-prod` and reload, so each new
      device's `--user`/`--marker` (emitted by the IE cfg-generator from edge_credentials)
      reaches the caster. Upload is Model A: the caster authenticates the SOURCE password,
      so a mountpoint absent from the caster cfg is REJECTED even though the LH edge gate
      accepts it. Fix the sync daemon (Kevin) OR do the manual interim below per batch.
- [ ] **Flash host** has ESP-IDF (esptool + nvs_partition_gen) and `RTK_ENROLL_MASTER`
      exported = the SAME master as `EDGE_ENROLL_MASTER_V1`.

## 1. Per-device factory flash (at the bench, USB)

1. `esptool.py -p COM4 erase_flash`            # clean slate (wipes any stale NVS)
2. Full firmware flash (from the build dir, build with `idf.py build` first):
   `idf.py -p COM4 flash`                       # bootloader + partition-table + app + www + ota_data
   (NVS is NOT written by idf.py flash -> it stays empty, device will come up as AP.)
3. Provision the per-device enroll key into NVS:
   `python tools/provision_device.py --port COM4 --flash`
   - reads the chip MAC, derives `enroll_key = HMAC(master, "rtkdata-enroll-key-v1:"+chip_id)`,
     writes NVS `rtk_enr_key` (64-hex) + `rtk_key_ver=1`. (Do this AFTER the full flash.)
4. Record for inventory: device_id = `rtk-<12hexmac>`, mountpoint = `RTK_<12HEXMAC>`,
     AP SSID = `RTKdata_<last 3 mac bytes>`. (NO secret recorded; the key stays on-device.)

## 2. First boot + WiFi onboarding (installer at site)

1. Power the device. It always comes up as a WiFi AP **`RTKdata_XXXXXX`** (last 3 MAC bytes).
2. Connect a phone/laptop to that AP.
3. Open the device web UI (the AP gateway, typically `http://192.168.4.1`).
4. Scan -> pick the site WiFi -> enter the password -> save. The device joins as STA
   (the AP auto-disables ~15 min after a successful STA association).

## 3. Automatic onboarding (no action needed)

5. Device enrolls: per-device key -> `POST /api/edge/enroll` -> IE returns `{station_token, caster}`.
6. cfg-generator emits the device's caster upload account (cron, or trigger `generateAllCfgs`).
7. UM980 survey-in -> provisional position -> RTCM streamed to the caster (via the LH upload gate).
8. IE captures the stream; once the day's precise products publish (~2-3 days) the PPP solve
   produces the cm coordinate, the orchestrator writes it, and the heartbeat pushes it back
   (`gnss_set_fixed_base`) -> device broadcasts the precise 1005 -> state = active.

## 4. Verify (server-side, /dashboard/edge)

- Device appears with: state, online, health (GNSS / Caster / Link all green), einmessung
  progress + ETA, coordinate + confidence.
- **Caster = green** confirms the upload is accepted (the section-0 blocker is OK for this device).
- Lifecycle: provisioning -> maturing -> (after einmessung) active.

## Manual caster interim (if the S3 sync is not fixed in time)

The IE generates the correct cfg; only the caster-box pull is missing. Per batch:
1. Enroll the devices first (so their edge_credentials exist), OR pre-create them.
2. Trigger the IE to regenerate + push the cfg to S3: `GET /api/trigger?pipeline=generate-cfgs`
   (calls `generateAllCfgs()`; optional `&region=us|eu|ap`).
3. On the caster box: pull the latest cfg from `s3://rtkdata-ntrips-configs-prod` and reload
   the Alberding caster (the step the daemon would automate). Verify the new `RTK_<id>`
   mountpoints are registered (sourcetable / accepts SOURCE).

## Troubleshooting

- **Not enrolling**: check `EDGE_ENROLL_MASTER_V1` is set + the NVS key was provisioned
  (step 1.3). IE log shows `per-device-v1` (good) vs `fleet-legacy` (old path) per enroll.
- **Upload rejected (Caster red)**: the mountpoint is not on the caster -> section-0 blocker /
  manual interim above.
- **Stuck maturing**: einmessung is product-latency-bound (~2-3 days); /dashboard/edge shows
  the honest ETA. Not an error.
- **No AP on first boot**: the device already has STA creds (re-flash/erase to reset), or it
  already associated and auto-disabled the AP.
