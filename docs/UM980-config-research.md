# UM980 reference-station configuration - SOTA research

Research backing the SOTA `config_gnss_base()` rewrite (`main/gnss.c`). Goal: a best-in-class RTCM3
reference base on the Unicore UM980, configured deterministically (ACK-gated) and matched to the
RTKdata downstream pipeline (Lighthouse 1005 rewrite + IE-computed position).

## Sources

- Unicore *Reference Commands Manual for N4 High Precision Products* (UM980 family command set).
- ArduSimple, *How to configure Unicore UM980/UM981/UM982* - practical base RTCM set + rates.
- s-taka.org, *Control command for GNSS receiver UM982* - **command response/ACK format**.
- SNIP, *An RTCM 3 message cheat sheet*; NovAtel *RTCMV3 standard logs* - message semantics.
- AgOpenGPS forum, *MSM4 vs MSM7 for optimal vertical accuracy* - observable-format tradeoff.

## Command response (ACK) format - the key to deterministic config

The UM980 echoes every command with a NMEA-style response:

```
success:  $command,<command>,response: OK*<crc>
failure:  $command,response: PARSING FAILD NO MATCHING FUNC <command>*<crc>
```

(The "FAILD" typo is in Unicore's firmware; match on `PARSING FAIL` to be safe.) This lets the firmware
send a command, read the UM980 reply, confirm `response: OK`, and retry on timeout. That replaces the
stock blind `vTaskDelay(500ms)` open loop - the difference between "usually configures" and "provably
configured."

## RTCM message set - what a SOTA base must broadcast

| Msg | Content | Rate | Why |
|---|---|---|---|
| **1005** | Stationary ARP (ECEF) | 10 s | Base identity anchor. **1005 not 1006** on purpose: Lighthouse rewrites the **1005** to the catalog ECEF downstream, so the device only needs to broadcast *a* 1005; the server owns the exact value. |
| **1033** | Receiver & antenna descriptor | 10 s | Lets rovers apply antenna phase-center corrections; carries the receiver/antenna strings. |
| **1230** | GLONASS L1/L2 code-phase biases | 10 s | **Missing in stock OnoLink.** Essential for GLONASS RTK across mixed receiver brands; without it rovers drop GLONASS or get worse fixes. |
| **1077** | GPS MSM7 | 1 s | full-resolution observables |
| **1087** | GLONASS MSM7 | 1 s | |
| **1097** | Galileo MSM7 | 1 s | |
| **1117** | QZSS MSM7 | 1 s | regional (APAC), harmless elsewhere |
| **1127** | BeiDou MSM7 | 1 s | |
| **1137** | NavIC MSM7 | 1 s | regional (India), harmless elsewhere |

Decisions:
- **MSM7 over MSM4**: MSM7 carries full-resolution pseudorange/phase + phase-range-rate -> best RTK
  performance and vertical accuracy. The uplink is Wi-Fi (not bandwidth-constrained like LoRa), so the
  ~2x size over MSM4 is acceptable. (MSM4 stays the fallback if a target network rejects MSM7.)
- **Never mix legacy (1004/1012) with MSM** in one stream - breaks remote rovers (SNIP). MSM only.
- **1005 over 1006**: 1006 adds antenna height, but the RTKdata pipeline rewrites **1005**; the precise
  ARP (height folded in) is computed server-side. Matching the pipeline beats generic "1006 is richer."

## Base mode + position

- **Provisional survey-in** at boot for a rough seed: `mode base time 300 1.5` (5 min, 1.5 m std) -
  longer than the stock 60 s for a better provisional. The autonomous position is only ~m-accurate and
  is **not trusted**.
- **Precise position is server-side**: once the IE's PPP/PPK pipeline converges (see ARCHITECTURE.md
  48 h gating), it pushes the exact coordinate and the firmware sets a **fixed** base:
  `mode base <lat> <lon> <height>` + `saveconfig`. From then the broadcast 1005 is authoritative.
  (Lighthouse also rewrites 1005 to the catalog value, so correctness holds even before the push.)
- Frame: the IE works in **ITRF2020**; the coordinate it pushes is already in that frame. The device
  just sets what it is given.

## Signal / tracking

- `CONFIG SIGNALGROUP 2` - enables all bands incl. **Galileo E6** (default group 1 omits it).
- `MASK 10` - 10 deg elevation cutoff: clean corrections (low-elevation sats carry multipath /
  atmosphere that would degrade the base's own observables). Configurable.
- On-device PPP (`CONFIG PPP ENABLE E6-HAS`) is **dropped for the base**: the base broadcasts raw
  observables + a fixed ARP; the precise position comes from the IE, not on-device PPP. Removing it is
  one less moving part. (It remains a rover-side feature.)

## Resulting ACK-gated sequence (`config_gnss_base`)

```
unlog com1
CONFIG SIGNALGROUP 2
MASK 10
rtcm1077 com1 1     rtcm1087 com1 1     rtcm1097 com1 1
rtcm1117 com1 1     rtcm1127 com1 1     rtcm1137 com1 1
rtcm1005 com1 10    rtcm1033 com1 10    rtcm1230 com1 10
mode base time 300 1.5
saveconfig
```

Each line is sent, then the firmware waits for `response: OK` (retry up to 3x; `PARSING FAIL` -> log and
move on). `gnss_set_fixed_base(lat,lon,h)` later swaps the survey-in for the IE coordinate.
`gnss_recover()` (called by the supervisor on a GNSS stall) hardware-resets the UM980 and re-runs this
sequence.

> com1 = UART2 (the caster-forwarded data port). It must stay pure RTCM, so GNSS quality (sats/fix) is
> assessed by the IE from the received stream, not parsed on-device (which would inject NMEA into the
> caster feed).
