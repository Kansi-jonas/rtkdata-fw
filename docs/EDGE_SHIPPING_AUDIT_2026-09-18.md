# Edge shipping audit — 2026-09-18

Historical snapshot. The subsequent local 1.1.10 implementation, successful
ESP32 build and revised test evidence are documented in
[Automatic setup and relocation](AUTOMATIC_SITE_LIFECYCLE_2026-09-19.md).
In particular, its 401/404 repair is implemented there; the older artifact
limitations below describe the September 18 audit, not the new local candidate.

Scope: firmware enrollment/heartbeat contract, fixed base configuration, RTCM
upload health, recovery, and shipping artifacts. Local source review and host
tests only; no hardware flash or production changes.

## Fixed in source

- **Fixed height requires confirmed zero undulation.** `main/gnss.c` now requires
  an ACK for `CONFIG UNDULATION 0.0` before applying a fixed `mode base`, both at
  boot restore and on IE position updates. Previously a rejected zero-undulation
  command could still latch the intended coordinate as applied, leaving the
  receiver using the wrong height datum and preventing later corrective retries.
- **Caster health requires payload progress.** `main/supervisor.c` no longer
  reports healthy before the first successful RTCM send. A SOURCE handshake
  still arms recovery, but does not by itself count as healthy. Send success
  means bytes accepted by the local TCP stack; it is not an end-to-end delivery
  acknowledgement from a rover.

## Validation

All compiled host tests used `gcc -std=c11 -Wall -Wextra -Werror -Imain/include`.
Write executables to a temporary directory, then execute each:

| Test sources | Checks |
| --- | ---: |
| `tests/host/test_gnss_ack.c main/gnss_ack.c` | 41 |
| `tests/host/test_ntrip_tx_core.c main/interface/ntrip_tx_core.c` | 138 |
| `tests/host/test_telemetry_json.c main/telemetry_json.c` | 14 |
| `python tests/host/test_station_health.py` | 25 |

**218 checks passed.** The new Python harness compiles the unchanged production
control-function bodies with mocked UART, NVS and time services. It covers
undulation failure, boot restore, cooldown/retry, unchanged coordinates, MODE
rejection, SAVE failure/NVS repair, and caster health before/after send progress.
In-memory mutation checks reintroduced each original defect independently; both
were caught. `git diff --check` passed, with existing line-ending warnings only.

## Remaining issues

- `main/provisioning.c`: heartbeat HTTP 401/404 only logs failure. A device whose
  server record/token is lost or changed does not automatically re-enroll.
- `main/interface/ntrip_server.c`: the SOURCE response is parsed after one TCP
  read. A status line split across packets causes an unnecessary reconnect.
- `docs/ship-runbook.md`: documented mountpoint `RTK_<MAC>` differs from the
  current IE-derived `RTK_RTK<MAC>` for a device ID `rtk-<mac>`. Use the actual
  enrollment response when checking caster configuration.

## Artifact and shipping limits

At audit time, `main/include/config.h` declared **1.1.9**, `ota/release.json`
declared **1.1.8**, and a pre-existing `dist-ship-1.1.9/` directory was present.
These are local observations, not verification of the published channel or the
version on a customer device. Existing shipping binaries were not rebuilt and
do **not** include this audit's source fixes.

A complete ESP-IDF build was not available: `IDF_PATH`, `idf.py` and the Xtensa
compiler were absent from the environment, and Docker Desktop's Linux engine
was stopped. Host tests do not validate the complete firmware image, physical
UM980 command handling, Wi-Fi behavior, or the deployed LH/caster configuration.
Rebuild a distinct release and verify its actual upload/1005 behavior on hardware
before treating these source fixes as delivered. Existing local edits and
shipping artifacts were preserved.
