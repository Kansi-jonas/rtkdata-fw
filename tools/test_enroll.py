#!/usr/bin/env python3
"""Validate the per-device enroll path LIVE, without a physical device.

Derives the per-device key from RTK_ENROLL_MASTER (must equal the IE env
EDGE_ENROLL_MASTER_V1), signs an enroll request for a device, and POSTs it to
the IE. A 200 + station_token means the per-device path works and the Render
master is set correctly. Run on the flash host where RTK_ENROLL_MASTER lives;
the master NEVER leaves the machine (only the HMAC signature is sent).

Use the REAL device_id (e.g. an already-enrolled one) so the call is IDEMPOTENT
(the IE returns the existing token, creates nothing). The mac is only used to
derive the key + fill the canon; it does not need to be the device's true mac
for this IE-side check (the IE derives from the mac it receives).

  export RTK_ENROLL_MASTER=<the 64-hex master>
  python test_enroll.py --device-id RTK38182BF7E454
"""
import argparse, hashlib, hmac, json, os, re, sys
import urllib.request, urllib.error

KDF_LABEL = "rtkdata-enroll-key-v1:"


def chip_id_from(mac, device_id):
    src = mac if mac else re.sub(r"(?i)^rtk-?", "", device_id)
    return re.sub(r"[^0-9a-fA-F]", "", src).lower()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--master", default=os.environ.get("RTK_ENROLL_MASTER"))
    ap.add_argument("--ie", default="integrity-engine.onrender.com")
    ap.add_argument("--device-id", required=True)
    ap.add_argument("--mac", default=None, help="device MAC; if omitted, derived from device-id")
    ap.add_argument("--fw", default="1.0.2")
    ap.add_argument("--key-version", type=int, default=1)
    a = ap.parse_args()

    if not a.master:
        sys.exit("error: --master or env RTK_ENROLL_MASTER (64-hex) required")
    try:
        master = bytes.fromhex(a.master.strip())
    except ValueError:
        sys.exit("error: master is not valid hex")
    if len(master) != 32:
        sys.exit("error: master must be 64 hex chars (32 bytes)")

    chip = chip_id_from(a.mac, a.device_id)
    if len(chip) != 12:
        sys.exit(f"error: chip id not 12 hex (got '{chip}'); pass --mac AA:BB:CC:DD:EE:FF")

    enroll_key = hmac.new(master, (KDF_LABEL + chip).encode(), hashlib.sha256).hexdigest()
    mac_fmt = a.mac or ":".join(chip[i:i + 2] for i in range(0, 12, 2)).upper()
    nonce = int.from_bytes(os.urandom(4), "big")
    canon = f"{a.device_id}|{mac_fmt}|{a.fw}|um980|{nonce}"
    sig = hmac.new(enroll_key.encode(), canon.encode(), hashlib.sha256).hexdigest()
    body = json.dumps({
        "device_id": a.device_id, "mac": mac_fmt, "fw_version": a.fw, "hw": "um980",
        "nonce": nonce, "key_version": a.key_version, "hmac": sig,
    }).encode()

    req = urllib.request.Request(
        f"https://{a.ie}/api/edge/enroll", data=body,
        headers={"Content-Type": "application/json"}, method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            code, resp = r.status, json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        code, resp = e.code, json.loads(e.read() or b"{}")
    except Exception as e:  # noqa: BLE001
        sys.exit(f"request failed: {e}")

    print("HTTP", code)
    if code == 200 and resp.get("station_token"):
        print("OK: per-device enroll ACCEPTED -> Render master matches + per-device path works.")
        print("    caster creds returned:", bool(resp.get("caster")))
        print("    (Render log should show 'per-device-v1', NOT 'fleet-legacy'.)")
    elif code == 401:
        print("FAILED 401: master mismatch (Render EDGE_ENROLL_MASTER_V1 != your RTK_ENROLL_MASTER)")
        print("    or the mac/canon did not match. Response:", json.dumps(resp))
    else:
        print("UNEXPECTED:", json.dumps(resp))


if __name__ == "__main__":
    main()
