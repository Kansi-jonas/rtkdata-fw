#!/usr/bin/env python3
"""One-command factory flasher for an RTKdata edge device (UM980/ESP32).

Does the whole bench flow in ONE call:
  read MAC -> derive per-device key -> erase_flash -> write firmware ->
  write the per-device enroll key into NVS.

The master comes from RTK_ENROLL_MASTER (env) and is NEVER printed. Run once
per device; prints an inventory line (chip id / device id / mountpoint / AP SSID)
with NO secret.

  set RTK_ENROLL_MASTER once (User env), then:
  python flash_device.py --port COM4

Prereqs: pip install esptool esp-idf-nvs-partition-gen ; firmware built into
build/ (idf.py build, or the Docker build). Use a FRESH device (erase wipes NVS,
incl. WiFi) -- this is the factory flow.
"""
import argparse
import os
import subprocess
import sys
import tempfile

# Reuse the byte-exact key derivation + NVS image builder from provision_device.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from provision_device import (  # noqa: E402
    derive_enroll_key, normalize_chip_id, read_mac, build_nvs_bin, NVS_OFFSET,
)

# Standard offsets for the rtkdata-fw partition table (partitions.csv). These
# match the flash command idf.py emits for this project.
FW_PARTS = [
    ("0x1000", "bootloader/bootloader.bin"),
    ("0x8000", "partition_table/partition-table.bin"),
    ("0x10000", "rtkdata-fw.bin"),
    ("0x210000", "www.bin"),
    ("0x640000", "ota_data_initial.bin"),
]


def esptool(port, *args):
    cmd = ["python", "-m", "esptool"]
    if port:
        cmd += ["--port", port]
    cmd += list(args)
    print("  $", " ".join(str(c) for c in cmd))
    subprocess.check_call(cmd)


def main():
    ap = argparse.ArgumentParser(description="Factory-flash one RTKdata edge device end to end.")
    ap.add_argument("--port", required=True, help="serial port, e.g. COM4")
    ap.add_argument("--build-dir", default=None, help="FW build dir (default: ../build next to tools/)")
    ap.add_argument("--master", default=os.environ.get("RTK_ENROLL_MASTER"))
    ap.add_argument("--key-version", type=int, default=1)
    ap.add_argument("--baud", default="460800")
    ap.add_argument("--skip-erase", action="store_true", help="skip erase_flash (NOT recommended for a fresh device)")
    a = ap.parse_args()

    if not a.master:
        sys.exit("error: RTK_ENROLL_MASTER not set (env) and --master not given")

    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    build = a.build_dir or os.path.join(repo, "build")
    parts = []
    for off, rel in FW_PARTS:
        p = os.path.join(build, rel)
        if not os.path.exists(p):
            sys.exit(f"error: missing build artifact {p}\n  build first: idf.py build (or the Docker build)")
        parts += [off, p]

    # 1. identity + key (no secret printed)
    print("[1/4] reading device MAC ...")
    mac = read_mac(a.port)
    chip = normalize_chip_id(mac)
    if len(chip) != 12:
        sys.exit(f"error: bad MAC '{mac}'")
    enroll_key = derive_enroll_key(a.master, mac)
    if not enroll_key:
        sys.exit("error: could not derive enroll key (master must be 64 hex)")
    bareid = ("RTK" + chip).upper()
    print(f"      chip_id={chip}  device_id=rtk-{chip}  mountpoint=RTK_{bareid}  AP=RTKdata_{chip[6:].upper()}")
    print(f"      enroll_key={enroll_key[:8]}... (suppressed; written to NVS, key_version={a.key_version})")

    # 2. erase
    if not a.skip_erase:
        print("[2/4] erase_flash ...")
        esptool(a.port, "erase_flash")
    else:
        print("[2/4] erase skipped")

    # 3. firmware
    print("[3/4] writing firmware ...")
    esptool(a.port, "--chip", "esp32", "-b", a.baud, "--before", "default_reset", "--after", "hard_reset",
            "write_flash", "--flash_mode", "dio", "--flash_size", "16MB", "--flash_freq", "40m", *parts)

    # 4. per-device enroll key -> NVS
    print("[4/4] writing per-device enroll key to NVS ...")
    nvs = os.path.join(tempfile.gettempdir(), f"nvs_{chip}.bin")
    build_nvs_bin(enroll_key, a.key_version, nvs)
    esptool(a.port, "write_flash", NVS_OFFSET, nvs)
    try:
        os.unlink(nvs)
    except OSError:
        pass

    print("\nDONE. Power-cycle the device:")
    print(f"  - it comes up as WiFi AP 'RTKdata_{chip[6:].upper()}' -> connect -> set the site WiFi")
    print("  - it then enrolls automatically and appears on /dashboard/edge")
    print(f"  inventory: device_id=rtk-{chip}  mountpoint=RTK_{bareid}")


if __name__ == "__main__":
    main()
