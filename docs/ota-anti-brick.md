# OTA anti-brick design (FW 1.0.4+)

## Why this exists

On 2026-06-20 a published OTA (1.0.3) crash-looped a device into an unusable
state: the v1.0.1-era OTA download ran the synchronous mbedTLS download on the
**3584-byte main task** (`CONFIG_ESP_MAIN_TASK_STACK_SIZE=3584`), overflowed the
stack mid-download (`Backtrace CORRUPTED`, `0xa5a5a5a5` = the FreeRTOS stack
fill), PANIC-rebooted, and re-attempted the same download on every boot. Two more
loop sources made it unrecoverable: `updateFirmware()` called `esp_restart()`
**unconditionally** (even on a failed download), and there was no per-version
attempt limit.

Hard requirement (Jonas): **the device must NEVER fall into an unusable state.**

## The four mechanisms

All four are ESP-IDF-standard, low-risk. They are layered so any single bad
update is caught:

1. **16K-stack download task** (`ota_boot_check_blocking` in `update.c`).
   The boot OTA check now runs on a dedicated 16 KB-stack FreeRTOS task instead
   of the 3584-byte main task; `app_main` blocks on a binary semaphore until it
   finishes. Fixes the stack-smash directly. The daily `OTA_Sched_Task` stack was
   raised 4096 -> 8192 (its manifest GET still does a TLS handshake).

2. **NVS download-attempt counter** (`ota_attempt_allowed`, keys
   `ota/fail_ver` + `ota/fail_cnt`). Recorded BEFORE the download (so a crash
   mid-download still counts). After `MAX_OTA_ATTEMPTS` (3) failed installs of a
   given version, that version is no longer attempted; the device boots the
   current, working FW. `updateFirmware()` now returns `esp_err_t` and only
   `esp_restart()`s on SUCCESS, so a failed install falls through to the running
   FW instead of rebooting.

3. **ESP-IDF app rollback** (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`).
   A freshly-OTA'd image boots `PENDING_VERIFY`. `ota_mark_valid_task` waits
   `OTA_HEALTHY_MS` (60 s) of stable uptime, then calls
   `esp_ota_mark_app_valid_cancel_rollback()`. If the new image crashes before
   confirming, the bootloader auto-reverts to the previous working app on the
   next boot. **Requires the new bootloader (USB reflash); OTA does not update
   the bootloader, so existing field units are not retro-protected.**

4. **Boot-loop guard -> factory fallback** (`ota_boot_loop_guard`, key
   `ota/bootloop`). Called early in `app_main` (after NVS + UART). Counts
   consecutive boots that never reached a healthy run; after `MAX_BOOT_LOOPS` (5)
   it erases the otadata partition and reboots into the **factory** app (the
   bench-flashed known-good image). This is the catch-all backstop for any
   crash-loop cause the rollback does not cover (e.g. a runtime crash-loop on an
   already-validated image). `ota_mark_valid_task` resets this counter (and the
   download-failure counter) after 60 s of stable uptime.

Partition layout (`partitions.csv`): `factory` (2M) + `app0`/`app1`
(ota_0/ota_1, 4M each) + `otadata`. The initial USB flash lands in `factory`;
OTA writes to app0/app1. Factory is the guaranteed fallback.

## CONTRACT: every future build MUST mark itself valid

With rollback enabled, **any OTA image that does not call
`esp_ota_mark_app_valid_cancel_rollback()` within 60 s will be rolled back.**
`ota_mark_valid_task` (spawned at the end of `app_main`) does this. Do NOT remove
that task, and do NOT delay full `app_main` init past ~60 s, or every OTA will
revert. (Factory and already-valid images are unaffected: the task only confirms
`PENDING_VERIFY` images.)

## Bench verification procedure (DO before raising release.json)

1. **Clean boot.** USB-flash 1.0.4 (factory) -> serial shows a clean `POWERON`
   boot, no PANIC; ~60 s later: `firmware healthy after 60s (not a pending-verify
   image; nothing to confirm)` + `$PESP,OTA,...`. Data plane comes up.
2. **OTA success path.** Point a TEST manifest at 1.0.5 (a real, good build).
   Device on 1.0.4 downloads on the 16K task -> NO crash (the core fix) -> reboots
   -> runs 1.0.5 as PENDING_VERIFY -> after 60 s logs `OTA image confirmed healthy
   -> rollback cancelled` (`$PESP,OTA,CONFIRMED,1.0.5`).
3. **OTA failure -> recovery.** Two sub-cases, both must end USABLE:
   a. Interrupt the download (kill wifi mid-transfer) repeatedly -> after 3
      attempts: `version ... failed 3/3 times -> NOT retrying` (`$PESP,OTA,GIVEUP`)
      -> boots current FW.
   b. Publish a 1.0.5 that panics on boot -> it never confirms -> bootloader
      rolls back to 1.0.4 (or, after 5 crash-boots, `$PESP,OTA,FACTORYFALLBACK`
      -> factory). Device ends usable, never bricked.

Only after all three pass: stage `ota/rtkdata-fw.bin` + `ota/www.bin` and raise
`ota/release.json` `version` above the deployed FW. Buggy-OTA field devices
(v1.0.1-era download code) CANNOT self-OTA to this fix -> USB-reflash them.

## A 5th fix the bench test surfaced: wifi.c restart-abort

Test (b) exposed a latent bug NOT in the OTA code: `wifi.c::wifi_ap_start_only()`
did `ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA))`. On `esp_restart()` the
Wi-Fi driver is stopped, which fires `STA_DISCONNECTED` -> this handler ->
`esp_wifi_set_mode` returns `ESP_ERR_WIFI_STOP_STATE` -> `ESP_ERROR_CHECK`
abort()s. Result: a PANIC on EVERY planned restart while connected (OTA reboot,
daily-check reboot, factory fallback). The device still recovered (the new image
was already staged), but it inflated the boot-loop counter and was ugly. Fixed to
soft-fail (log + return). The 1.0.2 "verified" build never hit this because it
never restarted while connected.

## Verification results (2026-06-20, device rtk-38182bf7e454)

Tested via an isolated `ota-selftest` git branch (a build with UPDATE_SERVER_URL
pointed at the branch) so no prod-channel field device was touched. ALL PASSED:

- **(a)** USB-flash 1.0.4 -> clean POWERON boot, no PANIC, boot-loop guard runs,
  boot OTA check on the 16K task (139 KB free), `firmware healthy after 60s`.
- **(b)** OTA 1.0.4 -> 1.0.5: app+www (~3.3 MB) downloaded on the 16K task with
  NO stack crash; rebooted; `OTA image confirmed healthy after 60s`. (This is
  where the wifi.c abort was found + fixed.)
- **(c1)** manifest -> a 404 binary: `OTA write failed: ESP_ERR_OTA_VALIDATE_FAILED`
  -> `OTA update FAILED -> staying on current FW (no reboot)` -> exactly ONE boot,
  device usable. The original crash-loop brick-class is gone.
- **(c2)** manifest -> a deliberately-broken build (abort on boot): an 8-boot
  sequence in which all three recovery layers fired in turn -- ESP-IDF rollback
  (each PENDING_VERIFY crash reverted), boot-loop guard `FACTORYFALLBACK` at 5
  boots, then failure-counter `GIVEUP` after 3 attempts -> device ended healthy on
  the factory app, NEVER bricked. Reset reasons turned clean (`SW_CPU_RESET`)
  once running the wifi-fixed factory build, confirming the wifi.c fix.
