#!/usr/bin/env python3
"""Point a unit's NTRIP upload at a bench caster (tools/bench_caster.py),
without an enroll key and without the IE.

The web UI no longer accepts caster settings (POST /config takes Wi-Fi keys
only), so this writes them straight into the NVS partition over USB:

    python tools/bench_point_caster.py --port COM14 --host 192.168.5.227

WARNING: this rewrites the whole NVS partition. Wi-Fi settings, the enroll
key and the boot counters of the test build are gone afterwards. Use it on
bench and burn-in units only, and run provision_device.py --flash AFTER the
burn-in: that rewrite then also removes the bench caster again.

Needs esptool and esp-idf-nvs-partition-gen (pip install esptool
esp-idf-nvs-partition-gen).
"""
import argparse
import os
import subprocess
import sys
import tempfile

NVS_NAMESPACE = "config"   # main/config.c STORAGE
NVS_OFFSET = "0x9000"      # partition table: nvs
NVS_SIZE = "0x6000"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial port of the unit, e.g. COM14")
    ap.add_argument("--host", required=True, help="IP of the PC running bench_caster.py")
    ap.add_argument("--caster-port", type=int, default=2101)
    ap.add_argument("--mount", default="BENCH")
    ap.add_argument("--password", default="bench")
    args = ap.parse_args()

    csv = (
        "key,type,encoding,value\n"
        f"{NVS_NAMESPACE},namespace,,\n"
        f"ntr_srv_host,data,string,{args.host}\n"
        f"ntr_srv_port,data,u16,{args.caster_port}\n"
        f"ntr_srv_mp,data,string,{args.mount}\n"
        f"ntr_srv_pass,data,string,{args.password}\n"
    )
    tmp = tempfile.mkdtemp()
    csv_path = os.path.join(tmp, "bench_caster_nvs.csv")
    bin_path = os.path.join(tmp, "bench_caster_nvs.bin")
    with open(csv_path, "w", newline="\n") as f:
        f.write(csv)
    subprocess.check_call([sys.executable, "-m", "esp_idf_nvs_partition_gen", "generate",
                           csv_path, bin_path, NVS_SIZE])
    print(f"writing NVS at {NVS_OFFSET}: caster {args.host}:{args.caster_port}/{args.mount} (wipes other NVS keys)")
    subprocess.check_call([sys.executable, "-m", "esptool", "--port", args.port,
                           "write-flash", NVS_OFFSET, bin_path])
    print("done; the unit reboots and uploads to the bench caster once the receiver sends data")


if __name__ == "__main__":
    main()
