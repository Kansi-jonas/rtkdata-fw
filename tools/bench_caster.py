#!/usr/bin/env python3
"""Bench NTRIP-upload receiver + live RTCM dual-decoder for the BeiDou-drop bug.

Point a base unit's NTRIP-server upload at THIS machine (host = your PC's LAN IP,
port = --port) and watch, per constellation, how many RTCM3 frames arrive
CRC-VALID vs CRC-INVALID (truncated). The forward bug shows up as: large frames
(BeiDou 1127, GPS 1077) losing CRC while tiny ones (NavIC 1137 with 0 sats)
stay valid.

Reproduce the drop deterministically with --throttle-bps: cap how fast we DRAIN
the socket. A slow drain fills the unit's TX buffer -> SO_SNDTIMEO fires ->
ntrip_server_send_data drops the 100-byte chunk -> large frames truncate.

Usage:
  python bench_caster.py                       # accept upload on :2101, full speed
  python bench_caster.py --port 2101 --throttle-bps 1200   # induce back-pressure
  python bench_caster.py --log raw.rtcm3        # also dump raw bytes for offline diff

The unit's NTRIP-server sends:  SOURCE <pwd> /<mount>\r\nSource-Agent: ...\r\n\r\n
We reply ICY 200 OK and then read the RTCM stream.
"""
import argparse, socket, time, sys
from collections import defaultdict

CRC24Q = 0x1864CFB
def crc24q(d):
    c = 0
    for b in d:
        c ^= b << 16
        for _ in range(8):
            c <<= 1
            if c & 0x1000000:
                c ^= CRC24Q
    return c & 0xFFFFFF

MSG = {1074:"GPS",1077:"GPS",1084:"GLO",1087:"GLO",1094:"GAL",1097:"GAL",
       1114:"QZS",1117:"QZS",1124:"BDS",1127:"BDS",1134:"NavIC",1137:"NavIC",
       1005:"ARP",1006:"ARP",1033:"1033",1230:"1230"}

def satcount_msm(payload):
    # 64-bit satellite mask starts at payload bit 73 (after msgnum12+staid12+
    # epoch30+multi1+iods3+resv7+clk2+ext2+smth1+smthint3 = bit 73)
    if len(payload) < 18:
        return None
    v = 0
    for k in range(64):
        pos = 73 + k
        v = (v << 1) | ((payload[pos // 8] >> (7 - pos % 8)) & 1)
    return bin(v).count("1")

def handle(conn, addr, throttle_bps, logf):
    print(f"[+] upload from {addr}", flush=True)
    conn.settimeout(5)
    # read the SOURCE request line + headers (until blank line or first binary)
    head = b""
    try:
        while b"\r\n\r\n" not in head and b"\xd3" not in head and len(head) < 512:
            c = conn.recv(1)
            if not c:
                break
            head += c
            if head.endswith(b"\r\n") and (head.startswith(b"SOURCE") or head.startswith(b"POST")):
                break
    except socket.timeout:
        pass
    line0 = head.split(b"\r\n", 1)[0].decode("latin-1", "replace")
    print(f"[+] request: {line0!r}", flush=True)
    conn.sendall(b"ICY 200 OK\r\n\r\n")

    # any RTCM already read past the header
    leftover = head[head.find(b"\xd3"):] if b"\xd3" in head else b""

    win = defaultdict(lambda: [0, 0])  # name -> [crc_ok, crc_bad]
    bds_sats = []
    buf = bytearray(leftover)
    total = 0
    t_last = time.time()
    conn.settimeout(2)
    bytes_this_sec = 0
    sec_start = time.time()
    print("[*] streaming... (Ctrl-C to stop; live stats every 3s)\n"
          "    name   CRC-ok  CRC-bad   note", flush=True)
    while True:
        # throttle drain: cap bytes/sec pulled off the socket
        if throttle_bps:
            now = time.time()
            if now - sec_start >= 1.0:
                bytes_this_sec = 0; sec_start = now
            if bytes_this_sec >= throttle_bps:
                time.sleep(max(0.0, 1.0 - (now - sec_start)))
                continue
        try:
            chunk = conn.recv(256)
        except socket.timeout:
            chunk = b""
        if chunk:
            if logf:
                logf.write(chunk)
            buf += chunk
            total += len(chunk)
            bytes_this_sec += len(chunk)
        # parse frames
        i = 0
        while i + 6 <= len(buf):
            if buf[i] != 0xD3 or (buf[i+1] & 0xFC) != 0:
                i += 1; continue
            ln = ((buf[i+1] & 0x03) << 8) | buf[i+2]
            if i + 3 + ln + 3 > len(buf):
                break
            frame = bytes(buf[i:i+3+ln]); payload = frame[3:]
            mt = (frame[3] << 4) | (frame[4] >> 4)
            name = MSG.get(mt, str(mt))
            good = crc24q(frame) == ((buf[i+3+ln] << 16) | (buf[i+3+ln+1] << 8) | buf[i+3+ln+2])
            win[name][0 if good else 1] += 1
            if good and mt in (1127, 1124):
                sc = satcount_msm(payload)
                if sc is not None:
                    bds_sats.append(sc)
            i += 3 + ln + 3
        buf = buf[i:]
        # periodic report
        if time.time() - t_last >= 3.0:
            print(f"    --- {total} bytes total ---", flush=True)
            for name in ("GPS","GLO","GAL","QZS","BDS","NavIC","ARP","1033","1230"):
                if name in win:
                    ok, bad = win[name]
                    note = ""
                    if bad and not ok:
                        note = "<== ALL CORRUPT (truncated)"
                    elif bad:
                        note = f"<== {bad} corrupt"
                    print(f"    {name:6s} {ok:6d}  {bad:6d}   {note}", flush=True)
            if bds_sats:
                print(f"    (valid BeiDou frames carried {min(bds_sats)}-{max(bds_sats)} sats)", flush=True)
            t_last = time.time()
        if not chunk:
            # connection idle/closed check
            try:
                conn.settimeout(0.1)
                probe = conn.recv(1, socket.MSG_PEEK)
                conn.settimeout(2)
                if not probe:
                    print("[!] upload closed", flush=True); break
            except socket.timeout:
                conn.settimeout(2)
            except OSError:
                print("[!] upload closed", flush=True); break

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=2101)
    ap.add_argument("--throttle-bps", type=int, default=0,
                    help="cap socket drain to N bytes/s to induce back-pressure (0 = full speed)")
    ap.add_argument("--log", help="also write raw RTCM bytes to this file")
    args = ap.parse_args()
    logf = open(args.log, "wb") if args.log else None
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(1)
    ips = socket.gethostbyname_ex(socket.gethostname())[2]
    print(f"[*] bench caster listening on 0.0.0.0:{args.port}", flush=True)
    print(f"    point the unit's NTRIP caster host at one of: {ips}  port {args.port}", flush=True)
    if args.throttle_bps:
        print(f"    THROTTLE: draining only {args.throttle_bps} B/s (induces the drop)", flush=True)
    try:
        while True:
            conn, addr = srv.accept()
            try:
                handle(conn, addr, args.throttle_bps, logf)
            except (ConnectionResetError, BrokenPipeError):
                print("[!] connection reset", flush=True)
            finally:
                conn.close()
    except KeyboardInterrupt:
        print("\n[*] bye", flush=True)
    finally:
        if logf: logf.close()

if __name__ == "__main__":
    main()
