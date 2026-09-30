#!/usr/bin/env python3
"""One-click GUI factory-flasher for RTKdata edge devices (UM980/ESP32).

A friendly wrapper around the proven flash_device.py flow so the bench flash is
a double-click + one button, not a CLI over chat:
  pick COM port -> Flash -> read MAC -> derive per-device key -> erase ->
  write the ship firmware -> write the per-device enroll key into NVS.

SECURITY (CLAUDE.md): the enroll master comes from RTK_ENROLL_MASTER (env) and
is NEVER shown, stored, or logged by this tool; only the enroll-key fingerprint
(first 8 hex) appears, same as flash_device.py. The master MUST be set once as a
User env var on THIS bench machine; the tool never asks for it or writes it
anywhere. That is also why this is a LOCAL tool, not a browser/web flasher: the
master must never reach client-side code.

Run:  python tools/flash_gui.py        (or double-click flash_gui.cmd)
Prereqs: pip install esptool esp-idf-nvs-partition-gen ; a dist-ship-*/ folder
(the released binaries) next to the repo root, or build/ from idf.py build.
"""
import os
import re
import sys
import glob
import queue
import threading
import subprocess
import tempfile
import tkinter as tk
from tkinter import ttk, scrolledtext

# Reuse the byte-exact identity + key + NVS logic from provision_device.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from provision_device import (  # noqa: E402
    derive_enroll_key, normalize_chip_id, read_mac, build_nvs_bin, NVS_OFFSET,
)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Flat ship-binary layout (dist-ship-*/). Offsets match partitions.csv /
# flash_device.py. A nested build/ dir is also supported (see resolve_parts).
#
# Two boards, two images: the OnoLink Wi-Fi fleet (1.x, app rtkdata-fw.bin) and
# the K_EVK_V4 Ethernet board (2.x, app rtkdata-fw-eth.bin, folders
# dist-ship-eth-*/ or build_eth/). The app name in the folder decides which
# board an image is for; same partition table, same offsets.
APP_WIFI = "rtkdata-fw.bin"
APP_ETH = "rtkdata-fw-eth.bin"
SHIP_PARTS = [
    ("0x1000", "bootloader.bin"),
    ("0x8000", "partition-table.bin"),
    ("0x10000", "{app}"),
    ("0x210000", "www.bin"),
    ("0x410000", "spiffs.bin"),
    ("0x640000", "ota_data_initial.bin"),
]
BUILD_PARTS = [
    ("0x1000", "bootloader/bootloader.bin"),
    ("0x8000", "partition_table/partition-table.bin"),
    ("0x10000", "{app}"),
    ("0x210000", "www.bin"),
    ("0x410000", "spiffs.bin"),
    ("0x640000", "ota_data_initial.bin"),
]

# Parts that may be absent: resolve_parts skips them instead of failing.
# spiffs.bin is a pre-formatted, empty 2 MB SPIFFS image. Writing it is not
# cosmetic: erase_flash leaves that partition raw, so on its FIRST boot
# ota_update_init() mounts it with format_if_mount_failed=true and formats 2 MB
# inline. That blocks the main task for MINUTES, logs nothing while it runs, and
# the config UI never comes up -- and any reset during the format restarts it
# from scratch, so an impatient operator can loop forever. See tools/flash_device.py
# for how to regenerate the image.
OPTIONAL_PARTS = {"spiffs.bin"}


def list_ports():
    try:
        from serial.tools import list_ports as lp
        return [f"{p.device}  ({p.description})" for p in lp.comports()]
    except Exception:
        return []


def _version_key(path):
    """Sort key by semantic version: dist-ship-1.1.10 after dist-ship-1.1.9.

    A plain string sort put 1.1.9 above 1.1.10 and preselected the wrong
    (older, never published) image."""
    m = re.search(r"(\d+(?:\.\d+)*)$", os.path.basename(path))
    return tuple(int(x) for x in m.group(1).split(".")) if m else ()


def find_build_dirs():
    """Flashable source dirs: Ethernet ship builds, then Wi-Fi ship builds,
    each newest first, then the local build dirs."""
    eth = sorted(glob.glob(os.path.join(REPO, "dist-ship-eth-*")), key=_version_key, reverse=True)
    wifi = sorted((d for d in glob.glob(os.path.join(REPO, "dist-ship-*"))
                   if not os.path.basename(d).startswith("dist-ship-eth-")),
                  key=_version_key, reverse=True)
    dirs = eth + wifi
    for name in ("build_eth", "build"):
        b = os.path.join(REPO, name)
        if os.path.isdir(b):
            dirs.append(b)
    return dirs


def image_board(src_dir):
    """'eth' or 'wifi', from the app binary the folder carries."""
    if os.path.exists(os.path.join(src_dir, APP_ETH)):
        return "eth"
    return "wifi"


def resolve_parts(src_dir):
    """Return [(offset, abspath), ...] for the chosen source dir, or raise."""
    nested = os.path.basename(src_dir) in ("build", "build_eth")
    layout = BUILD_PARTS if nested else SHIP_PARTS
    app = APP_ETH if image_board(src_dir) == "eth" else APP_WIFI
    parts = []
    for off, rel in layout:
        rel = rel.format(app=app)
        p = os.path.join(src_dir, rel)
        if not os.path.exists(p):
            if os.path.basename(rel) in OPTIONAL_PARTS:
                continue
            raise FileNotFoundError(f"missing artifact: {p}")
        parts += [off, p]
    return parts


class FlasherGUI:
    def __init__(self, root):
        self.root = root
        self.q: "queue.Queue[str]" = queue.Queue()
        self.busy = False
        root.title("RTKdata Edge Flasher")
        root.geometry("760x520")

        top = ttk.Frame(root, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Serial port:").grid(row=0, column=0, sticky="w")
        self.port_cb = ttk.Combobox(top, width=46, state="readonly")
        self.port_cb.grid(row=0, column=1, sticky="w", padx=6)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=4)

        ttk.Label(top, text="Firmware:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.build_cb = ttk.Combobox(top, width=46, state="readonly")
        self.build_cb.grid(row=1, column=1, sticky="w", padx=6, pady=(6, 0))

        ttk.Label(top, text="Baud:").grid(row=2, column=0, sticky="w", pady=(6, 0))
        self.baud_cb = ttk.Combobox(top, width=12, state="readonly", values=["460800", "921600", "230400", "115200"])
        self.baud_cb.set("460800")
        self.baud_cb.grid(row=2, column=1, sticky="w", padx=6, pady=(6, 0))

        # Master presence indicator (never shows the value).
        self.master_ok = bool(os.environ.get("RTK_ENROLL_MASTER"))
        ttk.Label(
            top,
            text=("RTK_ENROLL_MASTER: set" if self.master_ok else "RTK_ENROLL_MASTER: NOT SET (set it as a User env var first)"),
            foreground=("#0a0" if self.master_ok else "#c00"),
        ).grid(row=3, column=0, columnspan=3, sticky="w", pady=(8, 0))

        self.flash_btn = ttk.Button(top, text="Flash device", command=self.start_flash)
        self.flash_btn.grid(row=4, column=0, columnspan=3, sticky="we", pady=(10, 0))

        self.log = scrolledtext.ScrolledText(root, height=22, font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self.refresh_ports()
        self.build_cb["values"] = [os.path.basename(d) for d in find_build_dirs()]
        if self.build_cb["values"]:
            self.build_cb.current(0)
        self.root.after(120, self.drain)

    def refresh_ports(self):
        ports = list_ports()
        self.port_cb["values"] = ports
        if ports and not self.port_cb.get():
            self.port_cb.current(0)

    def emit(self, line):
        self.q.put(line)

    def drain(self):
        try:
            while True:
                line = self.q.get_nowait()
                self.log.insert("end", line + "\n")
                self.log.see("end")
        except queue.Empty:
            pass
        self.root.after(120, self.drain)

    def start_flash(self):
        if self.busy:
            return
        if not self.master_ok:
            self.emit("ERROR: RTK_ENROLL_MASTER is not set. Set it once as a User env var, reopen, retry.")
            return
        port_sel = self.port_cb.get().split()[0] if self.port_cb.get() else ""
        if not port_sel:
            self.emit("ERROR: no serial port selected.")
            return
        build_sel = self.build_cb.get()
        if not build_sel:
            self.emit("ERROR: no firmware folder found (need dist-ship-*/ or build/).")
            return
        self.busy = True
        self.flash_btn.config(state="disabled", text="Flashing...")
        threading.Thread(target=self._flash_worker, args=(port_sel, build_sel, self.baud_cb.get()), daemon=True).start()

    def _esptool(self, port, *args):
        cmd = [sys.executable, "-m", "esptool"]
        if port:
            cmd += ["--port", port]
        cmd += [str(a) for a in args]
        self.emit("  $ " + " ".join(cmd))
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        for line in proc.stdout:
            self.emit("    " + line.rstrip())
        proc.wait()
        if proc.returncode != 0:
            raise RuntimeError(f"esptool exited {proc.returncode}")

    def _check_board(self, port, board):
        """Refuse to put one board's image on the other board.

        The K_EVK_V4 module (ESP32-WROOM-32UE-N16R2) has PSRAM inside the chip
        package (ESP32-D0WDR2-V3); esptool reports it as "Embedded PSRAM". The
        Wi-Fi image would drive RMII lines as LEDs there, and the Ethernet image
        would drive the OnoLink LED and reset pins as RMII."""
        out = subprocess.run([sys.executable, "-m", "esptool", "--port", port, "chip-id"],
                             capture_output=True, text=True).stdout
        # esptool 4.x prints "Chip is ESP32-D0WDR2-V3 (...)", 5.x "Chip type:  ESP32-D0WDR2-V3 (...)"
        chip = re.search(r"(?:Chip is|Chip type:)\s*(.+)", out)
        self.emit(f"      chip: {chip.group(1).strip() if chip else 'unknown'}")
        if not chip:
            self.emit("      WARNING: could not read the chip type, board check skipped")
            return
        psram = ("Embedded PSRAM" in out) or ("D0WDR2" in chip.group(1))
        if board == "eth" and not psram:
            raise RuntimeError("this is an ETHERNET image (K_EVK_V4) but the chip has no "
                               "embedded PSRAM: looks like an OnoLink Wi-Fi board. Pick a "
                               "dist-ship-1.x folder.")
        if board == "wifi" and psram:
            raise RuntimeError("this is a Wi-Fi image (OnoLink, 1.x) but the chip has embedded "
                               "PSRAM: looks like a K_EVK_V4 Ethernet board. Pick a "
                               "dist-ship-eth-* or build_eth folder.")

    def _flash_worker(self, port, build_name, baud):
        try:
            src = os.path.join(REPO, build_name)
            parts = resolve_parts(src)
            board = image_board(src)
            master = os.environ.get("RTK_ENROLL_MASTER")

            self.emit(f"[0/4] image is for the {'K_EVK_V4 Ethernet' if board == 'eth' else 'OnoLink Wi-Fi'} board; checking the chip ...")
            self._check_board(port, board)

            self.emit(f"[1/4] reading device MAC on {port} ...")
            mac = read_mac(port)
            chip = normalize_chip_id(mac)
            if len(chip) != 12:
                raise RuntimeError(f"bad MAC '{mac}'")
            enroll_key = derive_enroll_key(master, mac)
            if not enroll_key:
                raise RuntimeError("could not derive enroll key (master must be 64 hex)")
            bareid = ("RTK" + chip).upper()
            ap_ssid = "RTKdata_" + chip[6:10].upper() + "%02X" % ((int(chip[10:12], 16) + 1) & 0xFF)
            self.emit(f"      chip_id={chip}  device_id=rtk-{chip}  mountpoint=RTK_{bareid}  AP={ap_ssid}")
            self.emit(f"      enroll_key={enroll_key[:8]}... (suppressed; -> NVS)")

            self.emit("[2/4] erase-flash (wipes NVS + WiFi; factory flow) ...")
            self._esptool(port, "erase-flash")

            self.emit(f"[3/4] writing firmware from {build_name} ...")
            self._esptool(port, "--chip", "esp32", "--baud", baud,
                          "write-flash", "--flash-mode", "dio", "--flash-size", "16MB", "--flash-freq", "40m", *parts)

            self.emit("[4/4] writing per-device enroll key -> NVS ...")
            nvs = os.path.join(tempfile.gettempdir(), f"nvs_{chip}.bin")
            build_nvs_bin(enroll_key, 1, nvs)
            self._esptool(port, "write-flash", NVS_OFFSET, nvs)
            try:
                os.unlink(nvs)
            except OSError:
                pass

            self.emit("")
            self.emit("DONE. Power-cycle the device:")
            if board == "eth":
                self.emit(f"  - plug the LAN cable: DHCP, hostname rtkdata-{chip[6:12].lower()} (no WiFi needed)")
                self.emit(f"  - without cable: WiFi AP '{ap_ssid}' -> connect -> set site WiFi")
            else:
                self.emit(f"  - comes up as WiFi AP '{ap_ssid}' -> connect -> set site WiFi")
            self.emit("  - then enrolls automatically and appears on /dashboard/edge")
            self.emit(f"  - once active + matured, the einmessung autopilot surveys it (CSRS-PPP)")
            self.emit(f"  inventory: device_id=rtk-{chip}  mountpoint=RTK_{bareid}")
        except Exception as e:  # noqa: BLE001
            self.emit(f"FLASH FAILED: {e}")
        finally:
            self.busy = False
            self.root.after(0, lambda: self.flash_btn.config(state="normal", text="Flash device"))


def main():
    root = tk.Tk()
    FlasherGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
