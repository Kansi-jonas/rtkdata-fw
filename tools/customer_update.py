#!/usr/bin/env python3
"""RTKdata base station updater -- the CUSTOMER-facing one-click tool.

Purpose: bring an ALREADY ENROLLED device up to the shipped firmware over USB,
for the units that cannot take the update over the air (see docs: a device on
1.1.5 is ~5.4 KB short of heap at its boot OTA check and can never install
anything by itself).

WHAT IT DOES, and nothing else:
    write app      -> 0x10000   rtkdata-fw.bin
    write otadata  -> 0x640000  ota_data_initial.bin

WHAT IT DELIBERATELY DOES NOT DO -- read this before "improving" it:
  * It NEVER erases the flash. An erase would wipe NVS, and NVS holds the
    device's enroll key, its caster credentials and its surveyed position. The
    customer would be left with a brick that has to come back to us.
  * It NEVER touches the partition table, the bootloader or www.bin. Those are
    unchanged since 1.0.6, and a long multi-file write is exactly what wedges
    the USB-serial adapter mid-flash.
  * It carries NO secrets. tools/flash_gui.py is the FACTORY flasher: it needs
    RTK_ENROLL_MASTER to derive a per-device enroll key, so it must never be
    handed to a customer. This tool has no master, no key derivation, no NVS
    write, and therefore nothing to leak.

Ship it as: this file + rtkdata-fw.bin + ota_data_initial.bin in one folder
(or a single PyInstaller .exe, see build_customer_update.cmd).
"""
import os
import sys
import glob
import queue
import threading
import subprocess
import tkinter as tk
from tkinter import ttk, scrolledtext

APP_OFFSET     = "0x10000"
OTADATA_OFFSET = "0x640000"
APP_NAME       = "rtkdata-fw.bin"
OTADATA_NAME   = "ota_data_initial.bin"


def resource_dir():
    """Where the .bin files live: next to the script, or inside the PyInstaller
    bundle when frozen."""
    if getattr(sys, "frozen", False):
        return sys._MEIPASS  # noqa: SLF001  (PyInstaller's documented attribute)
    return os.path.dirname(os.path.abspath(__file__))


def find_images():
    base = resource_dir()
    app = os.path.join(base, APP_NAME)
    ota = os.path.join(base, OTADATA_NAME)
    missing = [p for p in (app, ota) if not os.path.exists(p)]
    if missing:
        raise FileNotFoundError(", ".join(os.path.basename(m) for m in missing))
    return app, ota


def list_ports():
    try:
        from serial.tools import list_ports as lp
        return [f"{p.device}  ({p.description})" for p in lp.comports()]
    except Exception:
        return []


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("RTKdata base station updater")
        self.geometry("720x460")
        self.q = queue.Queue()
        self.busy = False

        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="1. Connect the base station by USB, then pick the port:").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))
        ttk.Label(top, text="Port:").grid(row=1, column=0, sticky="w")
        self.port_cb = ttk.Combobox(top, width=46, state="readonly")
        self.port_cb.grid(row=1, column=1, sticky="w", padx=6)
        ttk.Button(top, text="Rescan", command=self.refresh).grid(row=1, column=2)

        self.flash_btn = ttk.Button(top, text="2. Start update", command=self.start)
        self.flash_btn.grid(row=2, column=0, columnspan=3, sticky="we", pady=(12, 0))

        self.status = ttk.Label(self, text="", padding=(10, 4))
        self.status.pack(fill="x")

        self.log = scrolledtext.ScrolledText(self, height=18, wrap="word")
        self.log.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self.refresh()
        self.after(120, self.drain)

    def emit(self, line):
        self.q.put(line)

    def drain(self):
        while True:
            try:
                line = self.q.get_nowait()
            except queue.Empty:
                break
            self.log.insert("end", line.rstrip() + "\n")
            self.log.see("end")
        self.after(120, self.drain)

    def refresh(self):
        ports = list_ports()
        self.port_cb["values"] = ports
        if ports and not self.port_cb.get():
            self.port_cb.current(0)
        if not ports:
            self.status.config(text="No device found. Check the USB cable, then press Rescan.")
        else:
            self.status.config(text="Ready.")

    def start(self):
        if self.busy:
            return
        sel = self.port_cb.get().split()[0] if self.port_cb.get() else ""
        if not sel:
            self.status.config(text="Pick a port first.")
            return
        self.busy = True
        self.flash_btn.state(["disabled"])
        self.status.config(text="Update running. Do NOT unplug the device.")
        threading.Thread(target=self.worker, args=(sel,), daemon=True).start()

    def worker(self, port):
        rc = 1
        try:
            app, ota = find_images()
            # esptool runs IN-PROCESS, not as "sys.executable -m esptool".
            # Inside a PyInstaller bundle sys.executable is this .exe, so the
            # subprocess form would relaunch the GUI instead of flashing.
            argv = ["--chip", "esp32", "-p", port,
                    "-b", "460800", "--before", "default-reset", "--after", "hard-reset",
                    "write-flash", "--flash-mode", "dio", "--flash-size", "16MB",
                    "--flash-freq", "40m",
                    APP_OFFSET, app, OTADATA_OFFSET, ota]
            self.emit("Writing firmware, this takes about a minute...")

            import contextlib
            import esptool

            class _Pipe:
                def __init__(self, emit):
                    self._emit, self._buf = emit, ""

                def write(self, s):
                    self._buf += s
                    while "\n" in self._buf:
                        line, self._buf = self._buf.split("\n", 1)
                        self._emit(line)

                def flush(self):
                    if self._buf:
                        self._emit(self._buf)
                        self._buf = ""

            pipe = _Pipe(self.emit)
            try:
                with contextlib.redirect_stdout(pipe), contextlib.redirect_stderr(pipe):
                    esptool.main(argv)
                rc = 0
            except SystemExit as e:
                rc = int(e.code or 0)
            finally:
                pipe.flush()
        except FileNotFoundError as e:
            self.emit(f"ERROR: file missing from the package: {e}")
        except Exception as e:  # noqa: BLE001  (surface anything to the customer log)
            self.emit(f"ERROR: {e}")
        finally:
            self.busy = False
            self.flash_btn.state(["!disabled"])
            if rc == 0:
                self.emit("")
                self.emit("DONE. The device reboots and is back online in about two minutes.")
                self.status.config(text="Update successful.")
            else:
                self.status.config(
                    text="Failed. Unplug and replug the USB cable, then try again.")


if __name__ == "__main__":
    App().mainloop()
