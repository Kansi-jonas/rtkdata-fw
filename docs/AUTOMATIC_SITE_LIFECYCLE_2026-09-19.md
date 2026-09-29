# Automatic setup and relocation, local 1.1.10 candidate

This implements the firmware part of the coordinated IE/Lighthouse site-v1
protocol. It is a local build, not a production release. Existing 1.1.9 devices
and binaries do not acquire this behavior from source changes.

## Receiver evidence and behavior

The [Unicore N4 reference manual R1.14](https://en.unicore.com/uploads/file/Unicore%20Reference%20Commands%20Manual%20For%20N4%20High%20Precision%20Products_V2_EN_R1.14.pdf),
sections 3.3, 3.3.1 and 7.5.14, distinguishes fixed-mode GGA (configured position)
from BASEPOS (independently computed actual base position). BASEPOSA 1 is
therefore monitored in fixed mode too. The manual also describes suppression of
RTCM when actual and configured base positions differ by more than 300 m.
These are documented receiver behaviors, not newly observed hardware results.

MODE BASE TIME 300 0 uses a zero saved-position reuse distance. The old second
argument 1.5 was a reuse distance, not a survey accuracy requirement.

BASEPOS parsing checks the complete ASCII CRC32, header/body structure, GPS
epoch, finite coordinates and SINGLE/SOL_COMPUTED quality. Ellipsoidal height
is MSL height plus reported undulation. Repeated/reversed epochs cannot count
as new samples. The manual's printed BASEPOS example has a CRC/undulation sign
inconsistency; the test explicitly uses the negative undulation matching the
published CRC, without relaxing CRC validation.

Stable evidence requires at least 6 satellites, horizontal sigma <= 3 m,
vertical sigma <= 6 m, at least 30 distinct samples over >= 60 s, and age <= 5 s.
A bounded 64-sample window uses medians and conservatively includes horizontal
spread in its reported uncertainty. Bad quality, gaps and stale observations
invalidate it. This is a location consistency check, not a precise survey.

## Safety and recovery

- Every ESP32 boot starts with upload held. Fresh independent evidence and an
  explicit grant for this boot are both required. Normal repeated grants do
  not reconnect or reconfigure a healthy stream.
- One quality-qualified horizontal outlier beyond max(10 m, 5*sigma) closes
  the local gate immediately. Survey grants retain their physical anchor
  through receiver resets/retries; only a newer persisted revision replaces
  that anchor. The sequence remains monotonic across receiver resets.
- Desired revision/mode/coordinate are one authoritative NVS blob. A survey
  intent invalidates the legacy fixed coordinate before any receiver command.
  Failed NVS writes cannot apply a command; corrupt managed state cannot fall
  back to an older legacy coordinate. Upload grants are never persisted.
- Applied revision requires acknowledged mode and SAVE commands. Fixed mode
  also requires acknowledged zero undulation. A failed SAVE retries SAVE
  without repeating an unchanged MODE. Recovery and revision acknowledgement
  share the receiver transaction lock.
- Closing the gate changes a generation, clears queued frames and rejects old
  partial socket sends. A guard under the queue lock prevents late enqueue
  after a flush; partial UART frames cannot cross generations.
- Missing BASEPOS reports trigger bounded receiver recovery even if RTCM still
  arrives: 60 s grace after receiver initialization/reset, then a 10 s report
  silence threshold. Checksummed no-solution reports keep logging liveness
  alive, while leaving upload held. Intentional holds do not cause caster
  recovery/reboot storms or force OTA rollback while location evidence is good.
- Missing RTCM observations also trigger receiver reset/reconfiguration when
  BASEPOS stays alive. Output is expected only with an effective upload grant
  and acknowledged MODE, after 30 s for fixed mode or 330 s for the deliberate
  300 s survey. After that grace, 10 s without usable observations triggers
  the existing recovery path, with its 20 s debounce. The callback checks
  health again under the receiver lock, so a newly started survey/hold cancels
  an obsolete recovery request. Repeated grants cannot renew the grace.
- Observation progress requires a complete CRC-valid configured MSM7 frame,
  consistent masks/body length, at least one cell with usable code and carrier
  range, and a new epoch in that constellation. Metadata, empty/all-invalid
  MSMs and repeated epochs cannot conceal silence. One useful constellation
  suffices: an empty NavIC stream alongside healthy GPS is valid behavior.
  Receiver-epoch tags discard pre-reset evidence and partial frames; week/day
  rollover and the 32-bit uptime wrap are handled. Completed MODE warmup stays
  latched across holds and uptime wrap until actual reconfiguration/reset.
- Heartbeat health and local status now include obs_expected and obs_silent_s
  (-1 means no observation evidence in this receiver epoch). Recovery logs
  distinguish observation, BASEPOS and general receiver silence, including
  their ages; fresh BASEPOS cannot misleadingly label this fault as 0 s stale.
- Confirmed heartbeat 401/404 closes upload and repairs enrollment using the
  separate per-device key, with enrollment retries capped at 300 s. A 403 holds
  upload without resetting enrollment or bypassing disable/quarantine.
- Customer UI follows checking/fencing/surveying/positioned/publishing phases.
  Unknown completion time is not shown as a countdown to zero.

The signed heartbeat canonical form and integer fields match the IE
docs/EDGE_SITE_PROTOCOL.md implementation. An old IE that lacks site-v1 cannot
open this firmware's boot gate. IE must receive strong exclusion/application
proofs from every serving Lighthouse before permitting a new survey upload.

## Validation

538 assertions passed across these suites (each suite counted once):

| Command or test sources | Checks |
| --- | ---: |
| test_gnss_site.c + main/gnss_site.c | 101 |
| python tests/host/test_gnss_site_runtime.py | 83 |
| python tests/host/test_site_boundaries.py | 27 |
| python tests/host/test_station_health.py | 33 |
| test_ntrip_tx_core.c + main/interface/ntrip_tx_core.c | 146 |
| test_rtcm_observations.c + main/interface/ntrip_tx_core.c | 63 |
| python tests/host/test_observation_adapter.py | 30 |
| test_gnss_ack.c + main/gnss_ack.c | 41 |
| test_telemetry_json.c + main/telemetry_json.c | 14 |

Compile C suites with gcc -std=c11 -Wall -Wextra -Werror -Imain/include -Itests/host,
the listed source files and -lm; write the executable into a temporary
directory, then run it. Python harnesses compile actual production function
bodies/runtime with mocked platform services. They cover failed persistence,
failed ACK/SAVE, replayed/conflicting revisions, stale/wrong-boot grants,
receiver resets, repeated relocation, authentication repair and final socket
send rejection. The TX property test includes 1,184 valid frames.

The observation/runtime regressions inject fresh BASEPOS with no RTCM,
metadata-only output, repeated epochs and healthy GPS with empty NavIC. They
exercise the actual supervisor fault helper, 330 s survey grace, duplicate
grants, holds/movement, failed MODE acknowledgement and reset/reapplication.
The separately authored UART adapter harness places a reset between chunk
capture and callback and verifies partial-frame/epoch handling, including
24.9/49.7-day clock boundaries. Recorded Italy/NZ RTCM was also parsed offline
with the production monitor by an independent reviewer; this establishes
parser compatibility, not hardware recovery or a rover FIX.

ESP32 app, bootloader, partition table and web filesystem successfully built
with the existing espressif/idf:v5.3.1 image. Build source was mounted read-only;
build/config outputs were isolated in %TEMP%\rtkdata-site-v1-build.
App descriptor and firmware telemetry both report **1.1.10**. Source version now
also drives the CMake app descriptor. dist-ship-1.1.10/manifest.json identifies
the exact local files, hashes, source base and modified source hashes.
The production OTA manifest and old binaries were not changed.
The observation-silence follow-up rebuild replaces the earlier September 19
local candidate; its old binary/manifest hashes are superseded. Use only the
current manifest in that directory to identify this candidate. It also records
a canonical source-set hash and per-suite assertion counts.

Independent reviews of firmware queue/send fencing and revision/persistence
found no remaining blocker after the receiver-reset and BASEPOS-liveness fixes.
An independent production review of observation recovery, reset/uptime tags
and the final warmup latch also found no further blocker in that scoped change.
The corresponding Lighthouse data-path review and focused -race tests also
passed; those are separate server changes that must be released coherently.

## Hardware acceptance still required

No station was flashed and no device command was sent in this implementation.
Before customer rollout, test the exact shipped UM980 firmware, antenna and
ESP32 board with these artifacts:

1. Confirm BASEPOSA output/CRC and independent coordinates while MODE BASE is
   fixed; confirm fresh survey command/ACK behavior. Test a factory-fixed
   station moved > 300 m, including no RTCM until resurvey.
2. Test unchanged-site boot, initial setup, displacement during fixed/survey
   modes, receiver-only reset, ESP32 reboot, interrupted NVS persistence,
   rejected mode/SAVE, missing BASEPOS output and network/server interruption.
   Specifically stop all MSM observations while keeping BASEPOS alive, and
   repeat with metadata-only output. Confirm bounded output recovery, no reset
   during normal 300 s survey-in, and useful fault diagnostics. Keep valid GPS
   with empty NavIC as a no-reset control case.
   Inspect each region's fence/coordinate proof before new-site corrections.
3. Measure UART stack high-water and heap under maximum RTCM + BASEPOS traffic,
   TLS heartbeat, reconnects and configuration retries. Xtensa
   -fstack-usage measured 5,744 bytes of our UART parsing call chain
   (4,144 UART task + 32 ingest + 112 runtime + 192 feed + 1,200 parser +
   64 numeric conversion wrapper). This excludes transitive newlib/RTOS calls.
   UART allocation is now 10,240 bytes, giving 4,496 bytes before those
   unmeasured calls. This is not a measured runtime reserve; require at least
   1 KiB observed minimum free stack during the hardware stress test.
   The added RTCM observation parser uses 80 bytes of compiler-reported stack,
   its UART adapter 48 bytes and frame callback 32 bytes; the documented
   BASEPOS call-chain maximum remains unchanged.
4. Verify actual rover RTK FIX, coordinates and reacquisition after relocation.
   Existing observed MSM7 Doppler sign behavior and absent GLONASS 1230 remain
   receiver/interoperability checks; parsing valid observations alone is no
   rover-FIX proof.

Horizontal moves below the uncertainty-qualified 10-15 m threshold and
vertical-only moves are outside automatic movement detection v1. Reported
sigma is a receiver estimate; multipath can defeat an SPP consistency check.
No software gate can recall bytes already accepted by TCP or received by a
rover. The server fence is required in addition to the local upload gate.
The pre-existing fragmented SOURCE handshake parser is a separate remaining
robustness issue, not silently claimed fixed by this change.

The historical Zak/NZ reports describe live-TCP send silence and a roughly
20.47 m raw-versus-served vertical discrepancy. This follow-up closes a
reproducible software recovery gap; it does not establish the historical cause
of either symptom. Zero-undulation/MODE/SAVE acknowledgement protects the
command sequence, but raw 1005 ellipsoidal coordinates and the served reference
frame still need comparison on the exact receiver/antenna setup. No new
RTCM clock/range-rate configuration or 1230 compatibility is claimed here.
