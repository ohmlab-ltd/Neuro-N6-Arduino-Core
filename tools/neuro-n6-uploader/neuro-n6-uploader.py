# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
# GUI uploader: writes one image.
import tkinter as tk
from tkinter import filedialog
import os
import sys
import time
import threading
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
try:
    import serial
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), "pyserial"))
    import serial

from utils.device import (
    find_app_port,
    find_bootloader_port,
    wait_for_bootloader_port,
    wait_for_port_gone,
)
from utils.dfu import DfuError, DfuSession, touch_1200


def resource_path(name):
    """Path to a bundled resource. A PyInstaller exe unpacks its data to
    sys._MEIPASS."""
    base = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base, name)

# Theme
BG = "#0f1115"
CARD = "#161b22"
CARD_ALT = "#1f242c"
ACCENT = "#58a6ff"
ACCENT_HOVER = "#6cb6ff"
SUCCESS = "#2ea043"
ERROR = "#da3633"
TEXT = "#e6edf3"
SUBTLE = "#8b949e"

FONT_TITLE  = ("Segoe UI", 13, "bold")
FONT_BODY   = ("Segoe UI", 11, "bold")
FONT_BUTTON = ("Segoe UI", 11, "bold")
FONT_SMALL  = ("Segoe UI", 9)

# DFU constants
BAUDRATE = 115200

XSPI_SECTOR_SIZE = 0x1000

# Flash address range, memory-mapped
FLASH_BASE = 0x90000000
FLASH_END  = 0x91FFFFFF
DEFAULT_APP_OFFSET = 0x00100000


class DFUGui:
    def __init__(self, root):
        self.root = root
        root.title("Neuro N6 DFU")
        root.configure(bg=BG)
        root.resizable(False, False)

        self.filepath = None
        self.filesize = 0
        self.flash_offset = DEFAULT_APP_OFFSET

        self.build_header()
        self.build_address()
        self.build_flow()
        self.build_controls()
        self.build_log()

    def build_header(self):
        header = tk.Frame(self.root, bg=BG)
        header.pack(fill="x", padx=20, pady=(20, 10))

        self.fw_name = tk.Label(header, text="No firmware selected",
                                fg=TEXT, bg=BG, font=FONT_TITLE)
        self.fw_name.pack(anchor="w")

        self.fw_size = tk.Label(header, text=" - ",
                                fg=SUBTLE, bg=BG, font=FONT_SMALL)
        self.fw_size.pack(anchor="w")

    def build_address(self):
        addr_frame = tk.Frame(self.root, bg=BG)
        addr_frame.pack(fill="x", padx=20, pady=(0, 10))

        lbl = tk.Label(addr_frame, text="Flash Address (hex):",
                       fg=TEXT, bg=BG, font=FONT_BODY)
        lbl.pack(side=tk.LEFT)

        self.addr_var = tk.StringVar(value=f"0x{FLASH_BASE + DEFAULT_APP_OFFSET:08X}")
        self.addr_entry = tk.Entry(addr_frame, textvariable=self.addr_var,
                                   bg=CARD, fg=TEXT, insertbackground=TEXT,
                                   font=FONT_BODY, width=14, relief="flat")
        self.addr_entry.pack(side=tk.LEFT, padx=(10, 10))

        range_lbl = tk.Label(addr_frame,
                             text=f"(0x{FLASH_BASE:08X} - 0x{FLASH_END:08X})",
                             fg=SUBTLE, bg=BG, font=FONT_SMALL)
        range_lbl.pack(side=tk.LEFT)

    def parse_address(self):
        """Returns the entry's offset from the flash base, or None when invalid."""
        try:
            addr_str = self.addr_var.get().strip()
            addr = int(addr_str, 16)

            if addr < FLASH_BASE or addr > FLASH_END:
                self.log_msg(f"Address 0x{addr:08X} outside flash range")
                return None

            offset = addr - FLASH_BASE
            if offset % XSPI_SECTOR_SIZE != 0:
                self.log_msg(f"Address must be sector-aligned (0x{XSPI_SECTOR_SIZE:X} bytes)")
                return None

            return offset
        except ValueError:
            self.log_msg("Invalid hex address format")
            return None

    def build_flow(self):
        flow = tk.Frame(self.root, bg=BG)
        flow.pack(padx=20, pady=10)

        self.steps = {}
        for name in ["Select File", "Erase Flash", "Write Firmware", "Verify", "Reset MCU"]:
            card = tk.Frame(flow, bg=CARD, width=160, height=90)
            card.pack_propagate(False)
            card.pack(side=tk.LEFT, padx=6)

            lbl = tk.Label(card, text=name, fg=TEXT, bg=CARD, font=FONT_BODY)
            lbl.pack(expand=True)

            self.steps[name] = card

    def build_controls(self):
        controls = tk.Frame(self.root, bg=BG)
        controls.pack(pady=15)

        self.modern_button(controls, "Select Firmware",
                           self.select_file).pack(side=tk.LEFT, padx=8)

        self.modern_button(controls, "Reboot to Bootloader",
                           self.start_reboot).pack(side=tk.LEFT, padx=8)

        self.modern_button(controls, "Start DFU",
                           self.start_dfu, primary=True).pack(side=tk.LEFT, padx=8)

    def modern_button(self, parent, text, command, primary=False):
        bg = ACCENT if primary else CARD_ALT
        fg = "black" if primary else TEXT
        hover = ACCENT_HOVER if primary else "#2a3038"

        btn = tk.Label(parent, text=text, bg=bg, fg=fg,
                       font=FONT_BUTTON, padx=20, pady=12, cursor="hand2")
        btn.bind("<Button-1>", lambda e: command())
        btn.bind("<Enter>", lambda e: btn.config(bg=hover))
        btn.bind("<Leave>", lambda e: btn.config(bg=bg))
        return btn

    def build_log(self):
        self.log = tk.Text(self.root, width=72, height=10,
                           bg=CARD, fg=TEXT, insertbackground=TEXT,
                           relief="flat", font=FONT_SMALL)
        self.log.pack(padx=20, pady=(5, 20))
        self.log.config(state=tk.DISABLED)

    def log_msg(self, msg):
        self.log.config(state=tk.NORMAL)
        self.log.insert(tk.END, msg + "\n")
        self.log.see(tk.END)
        self.log.config(state=tk.DISABLED)
        self.root.update_idletasks()

    def set_step(self, name, colour):
        card = self.steps[name]
        card.config(bg=colour)
        for c in card.winfo_children():
            c.config(bg=colour)
        self.root.update_idletasks()

    def select_file(self):
        path = filedialog.askopenfilename(filetypes=[("Binary", "*.bin")])
        if not path:
            return

        self.filepath = path
        self.filesize = os.path.getsize(path)

        self.fw_name.config(text=os.path.basename(path))
        self.fw_size.config(text=f"{self.filesize:,} bytes")

        self.set_step("Select File", SUCCESS)
        self.log_msg("Firmware selected")

    def start_dfu(self):
        if not self.filepath:
            self.log_msg("No firmware selected")
            self.set_step("Select File", ERROR)
            return

        threading.Thread(target=self.run_dfu, daemon=True).start()

    def start_reboot(self):
        threading.Thread(target=self.reboot_device, daemon=True).start()

    def reboot_device(self):
        """Reboots the running app into the bootloader: the 1200-bps touch,
        then the 'b' command for an app that ignores the touch."""
        if find_bootloader_port():
            self.log_msg("Already in bootloader - ready to Start DFU")
            return
        app = find_app_port()
        if not app:
            self.log_msg("No Neuro N6 app found (already rebooting? try Start DFU)")
            return
        self.log_msg(f"Rebooting {app} into bootloader...")
        touch_1200(app, lambda text: None)
        if not wait_for_port_gone(app, timeout_sec=3):
            try:
                ser = serial.Serial(app, BAUDRATE, timeout=1, write_timeout=1)
                ser.dtr = True
                ser.rts = True
                time.sleep(0.1)
                ser.reset_input_buffer()
                ser.write(b'b\n')
                ser.flush()
                time.sleep(0.2)
                ser.close()
                self.log_msg(f"No answer to the 1200-bps touch: sent 'b' to {app}")
            except serial.SerialException:
                pass  # the app port drops as the device resets
            except Exception as e:
                self.log_msg(f"Reboot note: {e}")
            wait_for_port_gone(app, timeout_sec=5)
        boot = wait_for_bootloader_port(timeout_sec=10)
        if boot:
            self.log_msg(f"Bootloader ready on {boot} - click Start DFU")
        else:
            self.log_msg("Bootloader not detected after reboot (double-tap reset?)")

    def run_dfu(self):
        flash_offset = self.parse_address()
        if flash_offset is None:
            return

        port = find_bootloader_port()
        if not port:
            self.log_msg("Device not in bootloader. Click 'Reboot to Bootloader' "
                         "first (or double-tap reset), then Start DFU.")
            return

        self.log_msg(f"Connected to {port}")
        self.log_msg(f"Target address: 0x{FLASH_BASE + flash_offset:08X}")

        with open(self.filepath, "rb") as f:
            file_data = f.read()
        host_crc = zlib.crc32(file_data) & 0xFFFFFFFF

        try:
            ses = DfuSession(port, log=self.log_msg).open()
        except DfuError as e:
            self.log_msg(str(e))
            return
        step = "Erase Flash"
        try:
            # A v2 bootloader keeps the session open after the image and reports
            # the CRC32 of the flash. An older one resets at the end.
            info = ses.probe_info()
            checks = bool(info) and ses.has_cap("MULTI") and ses.has_cap("CRC32")

            self.set_step("Erase Flash", ACCENT)
            self.set_step("Write Firmware", ACCENT)
            step = "Write Firmware"
            self.log_msg(f"Erasing and writing {len(file_data):,} bytes...")
            t0 = time.time()
            ses.program(file_data, flash_offset, "firmware", next_image=checks, progress=False)
            self.log_msg(f"Write complete in {time.time() - t0:.1f} s")
            self.set_step("Erase Flash", SUCCESS)
            self.set_step("Write Firmware", SUCCESS)

            step = "Verify"
            self.set_step("Verify", ACCENT)
            if not checks:
                self.log_msg("This bootloader cannot report a CRC32: not verified")
                self.set_step("Verify", SUBTLE)
            else:
                got = ses.crc32(flash_offset, len(file_data))
                if got != host_crc:
                    self.log_msg("Verify FAILED: device CRC32 %s, file 0x%08X. The board stays "
                                 "in its bootloader: Start DFU again."
                                 % ("0x%08X" % got if got is not None else "unreadable", host_crc))
                    self.set_step("Verify", ERROR)
                    return
                self.log_msg(f"Verified: device CRC32 0x{got:08X} matches the file")
                self.set_step("Verify", SUCCESS)
                ses.reset_device()
            self.set_step("Reset MCU", SUCCESS)
            self.log_msg("DFU complete")
        except DfuError as e:
            self.log_msg(f"DFU failed: {e}")
            self.set_step(step, ERROR)
        finally:
            ses.close()


if __name__ == "__main__":
    root = tk.Tk()
    try:
        root.iconbitmap(resource_path("SoM-3.ico"))
    except Exception:
        pass
    DFUGui(root)
    root.mainloop()
