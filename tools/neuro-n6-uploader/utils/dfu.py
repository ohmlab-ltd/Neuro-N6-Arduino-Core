# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
# Bootloader DFU session and entry helpers.
#
# Commands are framed as STX | LEN16-LE | DATA | XOR | ETX. Replies are CRLF
# text lines. Offsets are from the external-flash base, 0x90000000.
#
#   v1, bootloader 0xCAFE:0x4013:
#     "DFU START" + u32 offset   -> "ADDR_SET 0x...", "N6ACK"
#     "DFU_ERASE" + u32 size     -> "ERASE_START", "ERASE_PROGRESS 0x..." lines, "N6ACK"
#     "DFU_DATA"                 -> "N6ACK", then 4096-byte data frames, one
#                                   "N6ACK" per frame and one more per 16 KB
#     "DFU_END"                  -> "DFU_DONE", device reset
#     anything else              -> "N6ACK"
#
#   v2 additions, bootloader 0x1209:0x8009:
#     "DFU_INFO"                     -> "N6INFO <ver> <CAP>... [UID <serial>]", "N6ACK"
#     "DFU_CRC32" + u32 off + u32 n  -> "CRC32 0x%08X", "N6ACK", idle only
#     "DFU_NEXT"                     -> "NEXT_OK", "N6ACK", leaves data mode
#     "DFU_RESET"                    -> "RESETTING", device reset, idle only
#
#   A bootloader reporting FSBLLOCK refuses DFU START at offset 0 with
#   "FSBL_LOCKED" unless "DFU_FSBL_UNLOCK" -> "FSBL_UNLOCKED", "N6ACK" came first.

import struct
import sys
import time
import zlib

import serial
import serial.tools.list_ports

from utils.pack import make_packet
from utils.device import (APP_IDENTITIES, BOOTLOADER_IDENTITIES, find_app_port,
                          find_bootloader_port, is_bootloader_family)

FLASH_BASE = 0x90000000
FLASH_END = 0x91FFFFFF
XSPI_SECTOR_SIZE = 0x1000
CHUNK_SIZE = 4096
BAUDRATE = 115200


class DfuError(Exception):
    """Protocol/transport failure talking to the bootloader."""


def human(n):
    if n >= 1024 * 1024:
        return "%.2f MB" % (n / (1024.0 * 1024.0))
    if n >= 1024:
        return "%.1f KB" % (n / 1024.0)
    return "%d B" % n


def raw(text):
    """Writes text with no newline. The Arduino IDE output panel does not
    rewrite lines. sys.stdout is None in a windowed build."""
    if sys.stdout is None:
        return
    sys.stdout.write(text)
    sys.stdout.flush()


def kv(key, value):
    return "    %-13s %s" % (key, value)


def erase_size_for(n):
    return (n + XSPI_SECTOR_SIZE - 1) // XSPI_SECTOR_SIZE * XSPI_SECTOR_SIZE


def addr_to_offset(addr):
    if addr < FLASH_BASE or addr > FLASH_END:
        raise DfuError("Address 0x%08X outside external flash (0x%08X-0x%08X)"
                       % (addr, FLASH_BASE, FLASH_END))
    off = addr - FLASH_BASE
    if off % XSPI_SECTOR_SIZE:
        raise DfuError("Address 0x%08X is not sector (4 KB) aligned" % addr)
    return off


class DfuSession(object):
    """One serial connection to a bootloader."""

    def __init__(self, port, log=print):
        self.port = port
        self.log = log
        self.ser = None
        self.info = None          # probe_info() result: version, caps, serial

    # Transport

    def open(self):
        # Windows lists a new port a moment before it can be opened.
        deadline = time.time() + 3.0
        while True:
            try:
                self.ser = serial.Serial(self.port, BAUDRATE, timeout=30)
                return self
            except serial.SerialException as e:
                if time.time() >= deadline:
                    raise DfuError("Cannot open bootloader port %s: %s" % (self.port, e))
                time.sleep(0.2)

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _send(self, payload):
        self.ser.write(make_packet(payload))

    def _read_line(self, timeout=None):
        if timeout is not None:
            old = self.ser.timeout
            self.ser.timeout = timeout
            try:
                return self.ser.readline().decode(errors="ignore").strip()
            finally:
                self.ser.timeout = old
        return self.ser.readline().decode(errors="ignore").strip()

    def _wait_ack(self, what, bad_replies=(), timeout=None, quiet_msgs=False):
        """Reads lines until N6ACK and returns the others. Raises DfuError on a
        listed bad reply or on timeout."""
        seen = []
        while True:
            resp = self._read_line(timeout=timeout)
            if resp == "N6ACK":
                return seen
            if resp == "":
                raise DfuError("No response from bootloader during %s" % what)
            if resp in bad_replies:
                raise DfuError("Bootloader rejected %s: %s" % (what, resp))
            seen.append(resp)
            if not quiet_msgs:
                self.log(kv("device", resp))

    # v2 capability probe

    def probe_info(self):
        """Sends DFU_INFO. Returns the info dict, or None on a v1 bootloader,
        which only ACKs."""
        self.ser.reset_input_buffer()
        self._send(b"DFU_INFO")
        try:
            seen = self._wait_ack("DFU_INFO", timeout=3, quiet_msgs=True)
        except DfuError:
            return None  # no ACK either: treated as v1
        for line in seen:
            if line.startswith("N6INFO"):
                parts = line.split()
                info = {"version": 1, "caps": set(), "serial": None}
                if len(parts) >= 2:
                    try:
                        info["version"] = int(parts[1])
                    except ValueError:
                        pass
                toks = parts[2:]
                i = 0
                while i < len(toks):
                    if toks[i] == "UID" and i + 1 < len(toks):
                        info["serial"] = toks[i + 1]
                        i += 2
                    else:
                        info["caps"].add(toks[i])
                        i += 1
                self.info = info
                return info
        return None

    def has_cap(self, cap):
        return bool(self.info) and cap in self.info["caps"]

    def crc32(self, offset, length):
        """Device-side CRC32 of `length` flash bytes at `offset`, or None when
        unsupported or failed."""
        if not self.has_cap("CRC32"):
            return None
        self._send(b"DFU_CRC32" + struct.pack("<II", offset, length))
        try:
            seen = self._wait_ack("DFU_CRC32", timeout=30, quiet_msgs=True)
        except DfuError:
            return None
        for line in seen:
            if line.startswith("CRC32"):
                try:
                    return int(line.split()[1], 16) & 0xFFFFFFFF
                except (IndexError, ValueError):
                    return None
        return None

    # Image programming

    def program(self, data, offset, label, next_image=False, progress=True):
        """START, ERASE, DATA, then DFU_NEXT or DFU_END. next_image=True needs
        MULTI and keeps the session open; otherwise the device resets."""
        total = len(data)
        if total == 0:
            raise DfuError("Refusing to program an empty image (%s)" % label)

        if offset == 0:
            # A bootloader that locks its own flash opens it for one DFU START.
            # One without the lock only acknowledges.
            self._send(b"DFU_FSBL_UNLOCK")
            self._wait_ack("DFU_FSBL_UNLOCK", timeout=3, quiet_msgs=True)

        self._send(b"DFU START" + struct.pack("<I", offset))
        self._wait_ack("DFU START", bad_replies=("START_BAD_ADDR", "FSBL_LOCKED"),
                       quiet_msgs=True)

        erase = erase_size_for(total)
        raw(kv("erase", "%s bytes at 0x%08X ... " %
               ("{:,}".format(erase), FLASH_BASE + offset)))
        try:
            self._send(b"DFU_ERASE" + struct.pack("<I", erase))
            self._wait_ack("DFU_ERASE",
                           bad_replies=("ERASE_BAD_SIZE", "ERASE_FLASH_MODE", "DFU_REFUSED"),
                           quiet_msgs=True)
        except DfuError:
            raw("failed\n")
            raise
        raw("ok\n")

        self._send(b"DFU_DATA")
        self._wait_ack("DFU_DATA", bad_replies=("DATA_NO_ERASE", "DFU_REFUSED"),
                       quiet_msgs=True)

        BAR = 34
        written = 0
        since_16k = 0
        drawn = 0
        t0 = time.time()
        if progress:
            raw(kv("write", "["))
        try:
            for i in range(0, total, CHUNK_SIZE):
                chunk = data[i:i + CHUNK_SIZE]
                self.ser.write(make_packet(chunk))
                ack = self._read_line()
                if ack != "N6ACK":
                    raise DfuError("Lost sync while writing %s at byte %d (got %r).\n"
                                   "Re-run the upload." % (label, written, ack))
                written += len(chunk)
                since_16k += len(chunk)
                if since_16k >= 16384:
                    ack = self._read_line()
                    if ack != "N6ACK":
                        raise DfuError("Lost sync (16 KB ack) while writing %s (got %r)"
                                       % (label, ack))
                    since_16k = 0
                if progress:
                    want = BAR * written // total
                    if want > drawn:
                        raw("=" * (want - drawn))
                        drawn = want
        except DfuError:
            if progress:
                raw("\n")   # ends the partial line
            raise
        dt = time.time() - t0
        if progress:
            raw("]  %s bytes in %.1f s (%.0f KB/s)\n"
                % ("{:,}".format(total), dt, total / dt / 1024 if dt else 0))

        if next_image:
            self._send(b"DFU_NEXT")
            seen = self._wait_ack("DFU_NEXT", quiet_msgs=True)
            if not any(l.startswith("NEXT_OK") for l in seen):
                raise DfuError("Bootloader did not confirm DFU_NEXT "
                               "(image boundary); aborting to stay safe.")
        else:
            self._send(b"DFU_END")
            try:
                resp = self._read_line(timeout=5)
                if resp and resp != "DFU_DONE":
                    self.log(kv("device", resp))
            except Exception:
                pass  # the device resets here

    def reset_device(self):
        """v2: resets the device from the idle state."""
        self._send(b"DFU_RESET")
        try:
            self._read_line(timeout=3)
        except Exception:
            pass


# Bootloader entry

def _ports():
    return list(serial.tools.list_ports.comports())


def touch_1200(port, log):
    """1200-bps touch: the core watches for baud 1200 and a DTR drop."""
    try:
        s = serial.Serial()
        s.port = port
        s.baudrate = 1200
        s.dtr = True
        s.open()
        time.sleep(0.1)
        s.dtr = False
        s.close()
        log(kv("action", "1200-baud touch on %s" % port))
        return True
    except serial.SerialException:
        return False


def _send_b(port, log):
    try:
        s = serial.Serial(port, BAUDRATE, timeout=1, write_timeout=1)
        s.dtr = True
        s.rts = True
        time.sleep(0.1)
        s.reset_input_buffer()
        s.write(b"b\n")
        s.flush()
        time.sleep(0.3)
        s.close()
        log(kv("action", "asked app on %s to enter bootloader" % port))
        return True
    except serial.SerialException as e:
        if "denied" in str(e).lower() or "busy" in str(e).lower():
            log(kv("BLOCKED", "%s is held open by another program" % port))
            log(kv("", "(Serial Monitor / Neuro Studio / another terminal)"))
            log(kv("", "close it and upload again"))
            return False
        return True  # the port vanished: the board is rebooting
    except Exception:
        return False


def _first_bootloader(exclude):
    """The first bootloader port by identity, skipping the ports in exclude."""
    ports = _ports()
    for ident in BOOTLOADER_IDENTITIES:
        for p in ports:
            if (p.vid, p.pid) == ident and p.device not in exclude:
                return p.device
    return None


def _wait_bootloader(timeout_sec, prev_app_port, log, ignore=()):
    """Returns the bootloader's port, or None on timeout. Ports in ignore are
    other boards' bootloaders."""
    start = time.time()
    app_gone = False
    while (time.time() - start) < timeout_sec:
        # 0xCAFE:0x4013 is also an app identity: not trusted on the app's port.
        port = _first_bootloader(set(ignore) | {prev_app_port})
        if port:
            return port
        devices = {p.device for p in _ports()}
        if prev_app_port and prev_app_port not in devices:
            app_gone = True
        if app_gone or prev_app_port is None:
            for p in _ports():
                if is_bootloader_family(p) and p.device not in ignore:
                    return p.device
        time.sleep(0.3)
    # The port never left: the board was in the bootloader already.
    if prev_app_port:
        for p in _ports():
            if p.device == prev_app_port and is_bootloader_family(p):
                return p.device
    return None


def enter_bootloader(preferred_port, log=print, timeout_sec=12):
    """Gets the board into the bootloader. Returns its port, or None. The
    board on preferred_port wins over a bootloader already on another port."""
    ports = _ports()
    selected = None
    if preferred_port:
        for p in ports:
            if p.device == preferred_port:
                selected = (p.vid, p.pid)
                break
    if selected in BOOTLOADER_IDENTITIES[:2]:
        log(kv("bootloader", "already present on %s" % preferred_port))
        return preferred_port

    app = None
    ignore = ()
    if selected == APP_IDENTITIES[0]:
        # The selected app runs, so a bootloader elsewhere is another board's.
        app = preferred_port
        ignore = {p.device for p in ports if p.device != app and is_bootloader_family(p)}
    else:
        boot = find_bootloader_port()
        if boot:
            log(kv("bootloader", "already present on %s" % boot))
            return boot
        if selected in APP_IDENTITIES:
            app = preferred_port
    if app is None:
        app = find_app_port()
        if app and preferred_port and app != preferred_port:
            log(kv("port", "selected %s is not the Neuro N6 app; using %s"
                   % (preferred_port, app)))

    if app:
        # The touch works with every core sketch. The 'b' command needs the
        # debug console, and a sketch reading Serial gets it as input.
        if touch_1200(app, log):
            boot = _wait_bootloader(6, app, log, ignore)
            if boot:
                log(kv("bootloader", boot))
                return boot
        if app in {p.device for p in _ports()}:
            _send_b(app, log)
        boot = _wait_bootloader(timeout_sec, app, log, ignore)
        if boot:
            log(kv("bootloader", boot))
            return boot
    else:
        log(kv("app", "not found; waiting for a bootloader to appear"))
        log(kv("", "(double-tap the reset button if nothing happens)"))
        boot = _wait_bootloader(timeout_sec, preferred_port or None, log)
        if boot:
            log(kv("bootloader", boot))
            return boot
    return None


def bootloader_serial(port):
    """iSerial of the bootloader port, or None."""
    for p in _ports():
        if p.device == port:
            return p.serial_number or None
    return None


def wait_for_app(timeout_sec=10, strict=False):
    """Returns the app's port once it enumerates, or None on timeout. strict=True
    matches 0x1209:0x8008 only: 0xCAFE:0x4013 is also a bootloader identity."""
    from utils.device import APP_VID, APP_PID
    start = time.time()
    while (time.time() - start) < timeout_sec:
        if strict:
            for p in _ports():
                if (p.vid, p.pid) == (APP_VID, APP_PID):
                    return p.device
        else:
            port = find_app_port()
            if port:
                return port
        time.sleep(0.3)
    return None


def host_crc32(data):
    return zlib.crc32(data) & 0xFFFFFFFF
