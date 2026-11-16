#!/usr/bin/env python3
"""Bench checks for the K_EVK_V4 Ethernet board (2.x image).

Two tests the sample boards have to pass before a field unit ships:

  reset  -- strap soak. Pulses EN through the FT231X auto-program circuit
            (RTS -> EN, DTR left released so GPIO0 is NOT pulled low) and reads
            the ROM banner after every reset. GPIO0 doubles as the RMII clock
            input from the LAN8720; if the PHY clock were running while the
            straps are sampled, the chip would land in download mode
            ("DOWNLOAD_BOOT") instead of booting ("boot:0x16
            SPI_FAST_FLASH_BOOT" on this board). PASS = every cycle boots
            from flash.

  net    -- LAN soak. Fetches the web UI over the cable in a loop and diffs the
            /status Ethernet counters (link flaps, LAN8720 symbol errors)
            before and after. Covers the RMII timing question (REF_CLK-out
            mode is not RMII-compliant per Microchip, the margin has to be
            measured). Run it inside the first 15 min after boot so the setup
            AP is up at the same time (Wi-Fi + Ethernet coexistence).
            PASS = no failed fetch, no link flap, no symbol error.

  soak   -- long run while the unit streams to a caster. Polls /status and
            /ntrip/tx_stats once a minute, writes a CSV row per poll and
            counts reboots (uptime falls), link flaps, symbol errors, caster
            reconnects, RTCM CRC errors, UART overflows and polls where the
            receiver did not produce the full message set.
            PASS = 0 reboots, 0 flaps, 0 symbol/CRC/overflow errors, at most
            2 caster reconnects per started day, at most 1 % of polls not
            fresh or unreachable.

Usage:
  python tools/eth_bench.py reset --port COM7 --cycles 100
  python tools/eth_bench.py net --ip 192.168.1.50 --minutes 10
  python tools/eth_bench.py soak --ip 192.168.1.50 --hours 72 --csv soak_unit1.csv
"""
import argparse
import json
import re
import sys
import time
import urllib.request

ROM_BOOT = re.compile(r"rst:0x([0-9a-f]+) \(([^)]*)\),boot:0x([0-9a-f]+) \(([^)]*)\)")


def cmd_reset(args):
    import serial

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = 115200
    ser.timeout = 0.2
    # Open with both lines released: an asserted RTS holds EN low (reset), an
    # asserted DTR pulls GPIO0 low (download mode).
    ser.dtr = False
    ser.rts = False
    ser.open()

    normal, download, unknown = 0, 0, 0
    for i in range(1, args.cycles + 1):
        ser.reset_input_buffer()
        ser.dtr = False
        ser.rts = True           # EN low
        time.sleep(0.1)
        ser.rts = False          # EN high -> boot, straps sampled now
        deadline = time.time() + args.wait
        buf = b""
        verdict = None
        while time.time() < deadline:
            buf += ser.read(4096)
            m = ROM_BOOT.search(buf.decode("latin-1"))
            if m:
                verdict = int(m.group(3), 16)
                break
        # Judge the boot MODE, not the raw strap byte: on this board a normal
        # boot reads boot:0x16 (GPIO5 = LAN_RST sits low on R101, GPIO4 = I2C
        # SCL high on its pull-up), the OnoLink reads 0x13. Both are
        # SPI_FAST_FLASH_BOOT; only GPIO0 low (bit 4 clear) means download mode.
        if verdict is None:
            unknown += 1
            tag = "NO ROM BANNER"
        elif "SPI_FAST_FLASH_BOOT" in m.group(4):
            normal += 1
            tag = f"flash boot (0x{verdict:x})"
        else:
            download += 1
            tag = f"boot:0x{verdict:x} ({m.group(4)})  <-- FAIL"
        print(f"[{i:4d}/{args.cycles}] {tag}", flush=True)
        time.sleep(args.settle)

    ser.close()
    print(f"\nflash boot {normal}, download/other {download}, no banner {unknown}")
    ok = download == 0 and unknown == 0
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


def get_status(ip):
    with urllib.request.urlopen(f"http://{ip}/status", timeout=5) as r:
        return json.loads(r.read().decode())


def cmd_net(args):
    before = get_status(args.ip)
    eth0 = before.get("eth")
    if not eth0:
        print("no 'eth' block in /status: not a 2.x Ethernet image?")
        return 2
    print(f"start: link={eth0.get('link')} speed={eth0.get('speed')} "
          f"link_ups={eth0.get('link_ups')} symbol_errors={eth0.get('symbol_errors')} "
          f"ap_active={before.get('wifi', {}).get('ap', {}).get('active')}")

    end = time.time() + args.minutes * 60
    fetches = fails = 0
    total_bytes = 0
    t0 = time.time()
    while time.time() < end:
        try:
            with urllib.request.urlopen(f"http://{args.ip}/", timeout=5) as r:
                total_bytes += len(r.read())
            fetches += 1
        except Exception as e:  # noqa: BLE001
            fails += 1
            print(f"  fetch failed: {e}", flush=True)
        if fetches % 50 == 0 and fetches:
            rate = total_bytes / max(1e-6, time.time() - t0) / 1024
            print(f"  {fetches} fetches, {fails} failed, {rate:.0f} KiB/s", flush=True)

    after = get_status(args.ip)
    eth1 = after.get("eth", {})
    flaps = eth1.get("link_ups", 0) - eth0.get("link_ups", 0)
    sym = eth1.get("symbol_errors", 0) - eth0.get("symbol_errors", 0)
    rate = total_bytes / max(1e-6, time.time() - t0) / 1024
    print(f"\n{fetches} fetches, {fails} failed, {rate:.0f} KiB/s average")
    print(f"link flaps {flaps}, LAN8720 symbol errors {sym}")
    ok = fails == 0 and flaps == 0 and sym == 0
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


def _get_json(ip, path, auth):
    req = urllib.request.Request(f"http://{ip}{path}")
    if auth:
        req.add_header("Authorization", "Basic " + auth)
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read().decode())


def cmd_soak(args):
    """Long soak while the unit streams to a caster: poll /status and
    /ntrip/tx_stats, write one CSV row per poll, judge at the end.

    Counters are per boot, so every delta is taken against the previous poll
    and a falling uptime counts as a reboot (the counters restart there)."""
    import base64
    import csv
    import math

    auth = None
    if args.user:
        auth = base64.b64encode(f"{args.user}:{args.password or ''}".encode()).decode()

    fields = ["utc", "uptime_s", "eth_link", "eth_speed", "eth_ip", "sta_parked",
              "link_ups", "symbol_errors", "caster_connected", "reconnects",
              "crc_errors", "uart_rx_overflows", "required_all_fresh",
              "bytes_out", "heap_free", "error"]
    out = open(args.csv, "a", newline="")
    w = csv.DictWriter(out, fieldnames=fields)
    if out.tell() == 0:
        w.writeheader()

    tot = {"reboots": 0, "link_flaps": 0, "symbol_errors": 0, "reconnects": 0,
           "crc_errors": 0, "uart_rx_overflows": 0, "not_fresh": 0,
           "polls": 0, "unreachable": 0, "disconnected": 0, "stalled": 0}
    prev = None
    bytes_out_total = 0
    end = time.time() + args.hours * 3600
    while True:
        row = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
        tot["polls"] += 1
        try:
            st = _get_json(args.ip, "/status", auth)
            tx = _get_json(args.ip, "/ntrip/tx_stats", auth)
        except Exception as e:  # noqa: BLE001
            tot["unreachable"] += 1
            row["error"] = str(e)[:120]
            w.writerow(row)
            out.flush()
            print(f"{row['utc']}  unreachable: {row['error']}", flush=True)
        else:
            eth = st.get("eth") or {}
            ing = tx.get("ingest") or {}
            inst = tx.get("instances") or []
            cur = {
                "uptime_s": st.get("uptime", 0),
                "link_ups": eth.get("link_ups", 0),
                "symbol_errors": eth.get("symbol_errors", 0),
                "reconnects": sum(i.get("reconnects", 0) for i in inst),
                "crc_errors": ing.get("crc_errors", 0),
                "uart_rx_overflows": ing.get("uart_rx_overflows", 0),
                # /status has one entry per stream (ntrip_server_0, uart, ...),
                # no "total" key: the upload is the sum of the NTRIP servers.
                "bytes_out": sum(((v or {}).get("total") or {}).get("out", 0)
                                 for k, v in (st.get("streams") or {}).items()
                                 if k.startswith("ntrip_server")),
            }
            row.update(cur)
            row.update({
                "eth_link": eth.get("link"), "eth_speed": eth.get("speed"),
                "eth_ip": eth.get("ip4"),
                "sta_parked": ((st.get("wifi") or {}).get("sta") or {}).get("parked"),
                "caster_connected": any(i.get("connected") for i in inst),
                "required_all_fresh": ing.get("required_all_fresh"),
                "heap_free": (st.get("heap") or {}).get("free"),
            })
            if not ing.get("required_all_fresh", False):
                tot["not_fresh"] += 1
            if not row["caster_connected"]:
                tot["disconnected"] += 1
            if prev is not None:
                if cur["uptime_s"] < prev["uptime_s"]:
                    tot["reboots"] += 1
                    print(f"{row['utc']}  REBOOT (uptime {prev['uptime_s']} -> {cur['uptime_s']})", flush=True)
                    # The new boot's first link-up and connect are normal; the
                    # reboot already fails the run, so restart the baseline.
                    base = dict(cur, bytes_out=0)
                else:
                    base = prev
                for k, t in (("link_ups", "link_flaps"), ("symbol_errors", "symbol_errors"),
                             ("reconnects", "reconnects"), ("crc_errors", "crc_errors"),
                             ("uart_rx_overflows", "uart_rx_overflows")):
                    d = cur[k] - base[k]
                    if d > 0:
                        tot[t] += d
                        print(f"{row['utc']}  {t} +{d}", flush=True)
                d_out = max(0, cur["bytes_out"] - base["bytes_out"])
                bytes_out_total += d_out
                # A poll interval without new upload bytes: fresh local RTCM
                # alone is not streaming (review 13.11. L3 #5).
                if d_out == 0:
                    tot["stalled"] += 1
            prev = cur
            w.writerow(row)
            out.flush()
        if time.time() >= end:
            break
        time.sleep(args.interval)
    out.close()

    days = max(1, math.ceil(args.hours / 24))
    max_rec = 2 * days
    max_not_fresh = max(1, tot["polls"] // 100)
    max_unreach = max(1, tot["polls"] // 100)
    print(f"\n{tot['polls']} polls over {args.hours} h, CSV: {args.csv}")
    print(f"reboots {tot['reboots']} (max 0), link flaps {tot['link_flaps']} (max 0), "
          f"symbol errors {tot['symbol_errors']} (max 0)")
    print(f"caster reconnects {tot['reconnects']} (max {max_rec}), RTCM CRC errors {tot['crc_errors']} (max 0), "
          f"UART overflows {tot['uart_rx_overflows']} (max 0)")
    print(f"polls without the full message set {tot['not_fresh']} (max {max_not_fresh}), "
          f"unreachable {tot['unreachable']} (max {max_unreach})")
    print(f"uploaded {bytes_out_total / 1e6:.1f} MB = {bytes_out_total / 1e6 / max(args.hours, 1e-6) * 24:.0f} MB/day")
    print(f"polls with the caster disconnected {tot['disconnected']} (max {max_not_fresh}), "
          f"poll intervals without upload {tot['stalled']} (max {max_not_fresh})")
    ok = (tot["reboots"] == 0 and tot["link_flaps"] == 0 and tot["symbol_errors"] == 0
          and tot["reconnects"] <= max_rec and tot["crc_errors"] == 0
          and tot["uart_rx_overflows"] == 0 and tot["not_fresh"] <= max_not_fresh
          and tot["unreachable"] <= max_unreach
          and tot["disconnected"] <= max_not_fresh and tot["stalled"] <= max_not_fresh
          and bytes_out_total > 0)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("reset", help="EN-reset strap soak over the USB serial port")
    r.add_argument("--port", required=True)
    r.add_argument("--cycles", type=int, default=100)
    r.add_argument("--wait", type=float, default=3.0, help="seconds to wait for the ROM banner")
    r.add_argument("--settle", type=float, default=2.0, help="seconds between cycles")
    n = sub.add_parser("net", help="LAN soak against the device's web server")
    n.add_argument("--ip", required=True)
    n.add_argument("--minutes", type=float, default=10)
    s = sub.add_parser("soak", help="long soak while streaming: poll /status + /ntrip/tx_stats, CSV log")
    s.add_argument("--ip", required=True)
    s.add_argument("--hours", type=float, default=72)
    s.add_argument("--interval", type=float, default=60, help="seconds between polls")
    s.add_argument("--csv", default="soak.csv")
    s.add_argument("--user", help="web UI user, only if basic auth is on")
    s.add_argument("--password")
    args = ap.parse_args()
    sys.exit({"reset": cmd_reset, "net": cmd_net, "soak": cmd_soak}[args.cmd](args))


if __name__ == "__main__":
    main()
