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

Usage:
  python tools/eth_bench.py reset --port COM7 --cycles 100
  python tools/eth_bench.py net --ip 192.168.1.50 --minutes 10
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
    args = ap.parse_args()
    sys.exit(cmd_reset(args) if args.cmd == "reset" else cmd_net(args))


if __name__ == "__main__":
    main()
