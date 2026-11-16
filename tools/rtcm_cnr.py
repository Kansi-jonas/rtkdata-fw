#!/usr/bin/env python3
"""Compare the signal strength (C/N0) of two raw RTCM3 recordings.

Used for the antenna chain test: record the same unit and antenna for the same
length of time once with the 9 m set cable and once with the shortest cable at
hand (`bench_caster.py --log a.rtcm3`), then

    python tools/rtcm_cnr.py short.rtcm3 set9m.rtcm3

prints the median C/N0 per constellation and signal for both files and the
difference. Only MSM4 to MSM7 carry C/N0; MSM6/7 use the 0.0625 dB-Hz field
(DF408), MSM4/5 the 1 dB-Hz field (DF403).

PASS (exit 0) = every signal tracked in the reference is also tracked in the
test file, and no signal's median is more than --max-drop dB (default 2) below
the reference. The LNA in the antenna has far more gain than 9 m of coax
loses, so a bigger drop points at the cable, a connector or the antenna feed.

Needs pyrtcm (pip install pyrtcm).
"""
import argparse
import statistics
import sys
from collections import defaultdict

from pyrtcm import RTCMReader
from pyrtcm.rtcmhelpers import parse_msm


def cnr_by_signal(path):
    vals = defaultdict(list)
    with open(path, "rb") as f:
        for _raw, msg in RTCMReader(f, quitonerror=0):
            if msg is None or not getattr(msg, "ismsm", False):
                continue
            parsed = parse_msm(msg)
            if not parsed:
                continue
            meta, _sats, cells = parsed
            for c in cells:
                cnr = c.get("DF408", c.get("DF403"))
                if cnr:  # 0 = not available
                    vals[(meta["gnss"], str(c.get("CELLSIG")))].append(float(cnr))
    return {k: statistics.median(v) for k, v in vals.items() if len(v) >= 30}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("reference", help="recording with the short cable")
    ap.add_argument("test", help="recording with the cable under test")
    ap.add_argument("--max-drop", type=float, default=2.0, help="dB-Hz allowed below the reference")
    args = ap.parse_args()

    ref = cnr_by_signal(args.reference)
    tst = cnr_by_signal(args.test)
    if not ref:
        print("no MSM4-7 C/N0 in the reference file")
        return 2

    ok = True
    print(f"{'GNSS':8} {'signal':8} {'ref':>6} {'test':>6} {'diff':>6}")
    for key in sorted(ref):
        r = ref[key]
        t = tst.get(key)
        if t is None:
            ok = False
            print(f"{key[0]:8} {key[1]:8} {r:6.1f} {'-':>6} {'-':>6}  <-- not tracked")
            continue
        d = t - r
        flag = ""
        if d < -args.max_drop:
            ok = False
            flag = "  <-- FAIL"
        print(f"{key[0]:8} {key[1]:8} {r:6.1f} {t:6.1f} {d:+6.1f}{flag}")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
