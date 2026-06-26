#!/usr/bin/env python3
"""Flash-host provisioning: write a PER-DEVICE enroll key into the device NVS.

Replaces the baked fleet secret. The master key lives ONLY here (and on the IE),
NEVER in firmware. Byte-exact contract: docs/enroll-per-device-key.md
  k_bytes    = HMAC_SHA256(master_raw_32, "rtkdata-enroll-key-v1:" + chip_id_hex)
  enroll_key = hex(k_bytes)                       # 64 lowercase hex chars
The IE re-derives the same key from the request mac, so verification is stateless.

NVS facts (from main/config.c + partitions.csv):
  namespace = "config" ;  rtk_enr_key = string ;  rtk_key_ver = u8
  nvs partition: offset 0x9000, size 0x6000 (default partition-table layout)

Usage:
  export RTK_ENROLL_MASTER=<64-hex master>        # NEVER commit / log this
  # generate only (prints key + builds nvs.bin), reads MAC from the device:
  python3 provision_device.py --port COM4
  # generate + flash the nvs partition (CAUTION: wipes other NVS keys incl WiFi):
  python3 provision_device.py --port COM4 --flash
  # offline (no device), supply the mac:
  python3 provision_device.py --mac A1:B2:C3:D4:E5:F6

CAUTION: flashing the nvs partition erases the rest of NVS (WiFi creds, token).
Use it as part of a FACTORY flash of a new device. For an already-deployed unit,
provision the key without a wipe (serial/web config), then re-run enroll.
"""
import argparse
import hashlib
import hmac
import os
import re
import subprocess
import sys
import tempfile

KDF_LABEL = "rtkdata-enroll-key-v1:"
NVS_NAMESPACE = "config"
NVS_OFFSET = "0x9000"
NVS_SIZE = "0x6000"


def normalize_chip_id(mac: str) -> str:
    """A1:B2:.. -> a1b2.. (hex-only, lowercase). Mirrors the IE normalizeChipId."""
    return re.sub(r"[^0-9a-fA-F]", "", mac).lower()


def derive_enroll_key(master_hex: str, mac: str) -> str:
    chip = normalize_chip_id(mac)
    if len(chip) != 12:
        sys.exit(f"error: mac '{mac}' did not normalize to 12 hex chars (got '{chip}')")
    master = bytes.fromhex(master_hex.strip())
    if len(master) != 32:
        sys.exit("error: master must be 64 hex chars (32 bytes)")
    return hmac.new(master, (KDF_LABEL + chip).encode(), hashlib.sha256).hexdigest()


def read_mac(port: str) -> str:
    out = subprocess.check_output([sys.executable, "-m", "esptool", "--port", port, "read-mac"], text=True)
    m = re.search(r"MAC:\s*([0-9A-Fa-f:]{17})", out)
    if not m:
        sys.exit(f"error: could not parse MAC from esptool output:\n{out}")
    return m.group(1)


def build_nvs_bin(enroll_key: str, key_version: int, out_bin: str) -> None:
    csv = (
        "key,type,encoding,value\n"
        f"{NVS_NAMESPACE},namespace,,\n"
        f"rtk_enr_key,data,string,{enroll_key}\n"
        f"rtk_key_ver,data,u8,{key_version}\n"
    )
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, newline="\n") as f:
        f.write(csv)
        csv_path = f.name
    # Use the CURRENT interpreter (sys.executable), not a hardcoded "python3"
    # (Windows usually only has "python"). The NVS generator is the standalone
    # pip module esp_idf_nvs_partition_gen (pip install esp-idf-nvs-partition-gen)
    # or the ESP-IDF nvs_partition_gen module if on PYTHONPATH.
    if _has_module("esp_idf_nvs_partition_gen"):
        mod = "esp_idf_nvs_partition_gen"
    elif _has_module("nvs_partition_gen"):
        mod = "nvs_partition_gen"
    else:
        os.unlink(csv_path)
        sys.exit("error: NVS generator not found. Run: pip install esp-idf-nvs-partition-gen")
    subprocess.check_call([sys.executable, "-m", mod, "generate", csv_path, out_bin, NVS_SIZE])
    os.unlink(csv_path)


def _has_module(name: str) -> bool:
    try:
        __import__(name)
        return True
    except Exception:
        return False


def main() -> None:
    ap = argparse.ArgumentParser(description="Provision a per-device enroll key into NVS.")
    ap.add_argument("--master", default=os.environ.get("RTK_ENROLL_MASTER"),
                    help="64-hex master key (or env RTK_ENROLL_MASTER). Never commit/log this.")
    ap.add_argument("--port", help="serial port (e.g. COM4) to read the MAC and/or flash")
    ap.add_argument("--mac", help="MAC (offline mode, skips reading from the device)")
    ap.add_argument("--key-version", type=int, default=1)
    ap.add_argument("--out", default="nvs_enroll.bin", help="output NVS image path")
    ap.add_argument("--flash", action="store_true",
                    help="flash the nvs partition (CAUTION: wipes other NVS keys)")
    ap.add_argument("--show-key", action="store_true",
                    help="print the full derived enroll_key (per-device SECRET; trusted terminal only)")
    args = ap.parse_args()

    if not args.master:
        sys.exit("error: --master or env RTK_ENROLL_MASTER required (64-hex)")
    mac = args.mac or (read_mac(args.port) if args.port else None)
    if not mac:
        sys.exit("error: need --mac or --port to obtain the device MAC")

    enroll_key = derive_enroll_key(args.master, mac)
    chip = normalize_chip_id(mac)
    print(f"chip_id     : {chip}")
    print(f"key_version : {args.key_version}")
    # Do NOT print the full enroll_key (it is a per-device secret). A short
    # fingerprint is enough to match/verify without leaking it. Use --show-key
    # to print it in full (e.g. for manual NVS entry) on a trusted terminal.
    if args.show_key:
        print(f"enroll_key  : {enroll_key}")
    else:
        print(f"enroll_key  : {enroll_key[:8]}... (suppressed; full key written to NVS)")

    build_nvs_bin(enroll_key, args.key_version, args.out)
    print(f"nvs image   : {args.out}  (namespace='{NVS_NAMESPACE}', size {NVS_SIZE})")

    if args.flash:
        if not args.port:
            sys.exit("error: --flash needs --port")
        print(f"flashing nvs partition at {NVS_OFFSET} (this wipes other NVS keys)...")
        subprocess.check_call([sys.executable, "-m", "esptool", "--port", args.port, "write-flash", NVS_OFFSET, args.out])
        print("done. device will derive its key on next boot and enroll.")
    else:
        print(f"\nto flash:  esptool.py --port {args.port or 'COM4'} write_flash {NVS_OFFSET} {args.out}")
        print("(CAUTION: flashing nvs wipes WiFi creds + token; use on a factory flash.)")


if __name__ == "__main__":
    main()
