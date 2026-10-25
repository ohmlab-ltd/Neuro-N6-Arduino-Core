# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
# Command-line uploader: writes one image.
# Usage: python neuro-n6-uploader-cli.py --port COMx --bin firmware.bin --address 0x90100000

import argparse
import os
import sys
import time
import struct
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
try:
    import serial
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), "pyserial"))
    import serial
import serial.tools.list_ports

from utils.pack import chunkate, pack, calculate_erase_size, make_packet

BAUDRATE = 115200
CHUNK_SIZE = 4096
XSPI_SECTOR_SIZE = 0x1000
FLASH_BASE = 0x90000000
FLASH_END = 0x91FFFFFF
DEFAULT_FLASH_OFFSET = 0x00100000
BAR_WIDTH = 30


def human_bytes(n):
    if n >= 1024 * 1024:
        return f"{n / (1024 * 1024):.2f} MB"
    if n >= 1024:
        return f"{n / 1024:.1f} KB"
    return f"{n} B"


def bar(done, total, width=BAR_WIDTH):
    total = max(1, total)
    done = max(0, min(done, total))
    filled = int(width * done / total)
    pct = done * 100.0 / total
    return f"[{'#' * filled}{'-' * (width - filled)}] {pct:5.1f}%"


def line():
    print("+------------------------------------------------------------+")


def banner(title):
    line()
    print("| Neuro N6 Upload                                           |")
    print(f"| {title[:58]:<58} |")
    line()


def step(idx, total, text):
    print(f"  [{idx}/{total}] {text}")

from utils.device import (APP_IDENTITIES, BOOTLOADER_IDENTITIES,
                          is_bootloader_family, find_app_port)
from utils.dfu import touch_1200


def _port_is_neuron6_app(port):
    """True if `port` enumerates as the app."""
    if not port:
        return False
    for p in serial.tools.list_ports.comports():
        if p.device == port and (p.vid, p.pid) in APP_IDENTITIES:
            return True
    return False


def _reboot_port(port):
    """Sends the 'b' reboot command to `port`."""
    try:
        ser = serial.Serial(port, 115200, timeout=1, write_timeout=1)
        ser.dtr = True
        ser.rts = True
        time.sleep(0.1)
        ser.reset_input_buffer()
        print(f"  Port      : {port}")
        print("  Action    : asking app to enter bootloader")
        ser.write(b'b\n')
        ser.flush()
        time.sleep(0.5)
        ser.close()
    except serial.SerialException:
        pass  # the port disappears as the device resets
    except Exception as e:
        print(f"Note: Could not send reboot command: {e}")

def enter_from_app(app_port, ignore=()):
    """Takes a running app into the bootloader with the 1200-bps touch, or the
    'b' command if it ignores the touch. Returns the bootloader's port or None;
    ports in ignore are other boards' bootloaders."""
    touch_1200(app_port, lambda text: print("  Action    : 1200-bps touch on %s" % app_port))
    boot = wait_for_bootloader_port(timeout_sec=4, prev_app_port=app_port, ignore=ignore)
    if boot is None and _port_is_neuron6_app(app_port):
        _reboot_port(app_port)
        boot = wait_for_bootloader_port(timeout_sec=10, prev_app_port=app_port, ignore=ignore)
    return boot

def parse_address(addr_str):
    """Returns the offset from the flash base, or None for a bad address."""
    try:
        addr = int(addr_str, 16)

        if addr < FLASH_BASE or addr > FLASH_END:
            print(f"Error: Address 0x{addr:08X} outside flash range (0x{FLASH_BASE:08X} - 0x{FLASH_END:08X})")
            return None

        offset = addr - FLASH_BASE
        if offset % XSPI_SECTOR_SIZE != 0:
            print(f"Error: Address must be sector-aligned (0x{XSPI_SECTOR_SIZE:X} bytes)")
            return None

        return offset
    except ValueError:
        print(f"Error: Invalid hex address format: {addr_str}")
        return None


def find_bootloader_port(exclude=()):
    """Port of a bootloader with a PID of its own, or None. 0xCAFE:0x4013 is
    left to wait_for_bootloader_port()."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if (port.vid, port.pid) in BOOTLOADER_IDENTITIES[:2] and port.device not in exclude:
            return port.device
    return None


def port_identity(port):
    """(vid, pid) of `port`, or None when it is not enumerated."""
    for p in serial.tools.list_ports.comports():
        if port and p.device == port:
            return (p.vid, p.pid)
    return None


def wait_for_bootloader_port(timeout_sec=10, prev_app_port=None, ignore=()):
    """Returns the bootloader's port, or None on timeout. Ports in ignore are
    other boards' bootloaders."""
    print("  Scan      : waiting for bootloader (double-tap reset if needed)")
    start = time.time()
    app_port_gone = False

    while (time.time() - start) < timeout_sec:
        current_devices = {p.device for p in serial.tools.list_ports.comports()}

        port = find_bootloader_port(ignore)
        if port:
            print(f"  Found     : bootloader on {port}")
            return port

        if prev_app_port and prev_app_port not in current_devices:
            app_port_gone = True

        # Once the app port has gone, any bootloader-family device is accepted.
        if app_port_gone:
            for p in serial.tools.list_ports.comports():
                if is_bootloader_family(p) and p.device not in ignore:
                    print(f"  Found     : legacy bootloader on {p.device}")
                    return p.device

        time.sleep(0.3)

    # The port never left: the board was in the bootloader already.
    if prev_app_port:
        for p in serial.tools.list_ports.comports():
            if p.device == prev_app_port and is_bootloader_family(p):
                print(f"  Found     : assuming {prev_app_port} is legacy bootloader")
                return p.device

    return None


def run_dfu(port, bin_path, flash_offset):
    if not os.path.exists(bin_path):
        print(f"ERROR: Binary file not found: {bin_path}")
        return False

    banner("DFU session")
    print(f"  Port      : {port}")
    print(f"  Firmware  : {bin_path}")
    print(f"  Address   : 0x{FLASH_BASE + flash_offset:08X}")
    print()

    # The timeout covers a flash erase.
    try:
        ser = serial.Serial(port, BAUDRATE, timeout=30)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open serial port {port}: {e}")
        return False

    with open(bin_path, "rb") as f:
        file_data = f.read()
    total_size = len(file_data)

    chunks, _ = chunkate(bin_path, CHUNK_SIZE)
    packets = pack(chunks)
    erase_size = calculate_erase_size(total_size, XSPI_SECTOR_SIZE)
    host_crc = zlib.crc32(file_data) & 0xFFFFFFFF

    print(f"  Size      : {total_size:,} bytes ({human_bytes(total_size)})")
    print(f"  Erase     : {erase_size:,} bytes ({human_bytes(erase_size)})")
    print(f"  CRC32     : 0x{host_crc:08X}")
    print()

    step(1, 4, "Handshake")
    print(f"  Progress  : {bar(1, 4)}")
    # "DFU START" + flash offset, uint32 LE
    start_payload = b"DFU START" + struct.pack("<I", flash_offset)
    ser.write(make_packet(start_payload))

    while True:
        resp = ser.readline().decode(errors="ignore").strip()
        if resp == "N6ACK":
            break
        elif resp == "START_BAD_ADDR":
            print("ERROR: DFU START failed - invalid address")
            ser.close()
            return False
        elif resp == "":
            print("ERROR: DFU START failed - no response")
            ser.close()
            return False
        else:
            print(f"  Device    : {resp}")

    print()
    step(2, 4, "Erase flash")
    print(f"  Progress  : {bar(2, 4)}")
    # "DFU_ERASE" + erase size, uint32 LE
    payload = b"DFU_ERASE" + struct.pack("<I", erase_size)
    ser.write(make_packet(payload))

    while True:
        resp = ser.readline().decode(errors="ignore").strip()
        if resp == "N6ACK":
            break
        elif resp == "ERASE_BAD_SIZE":
            print("ERROR: Erase failed - bad size")
            ser.close()
            return False
        elif resp == "":
            print("ERROR: Erase timeout - no response")
            ser.close()
            return False
        else:
            print(f"  Device    : {resp}")
    print("  Erase     : complete")

    print()
    step(3, 4, "Program firmware")
    print(f"  Progress  : {bar(0, total_size)}  0 / {total_size:,} bytes")
    ser.write(make_packet(b"DFU_DATA"))

    resp = ser.readline().decode(errors="ignore").strip()
    if resp != "N6ACK":
        print(f"ERROR: DFU_DATA failed (got {resp!r})")
        ser.close()
        return False

    bytes_written = 0
    bytes_since_16k = 0
    start_time = time.time()
    last_progress = -1

    for packet in packets:
        payload_len = len(packet) - 5
        ser.write(packet)
        bytes_written += payload_len
        bytes_since_16k += payload_len

        # One N6ACK per frame.
        ack = ser.readline().decode(errors="ignore").strip()
        if ack != "N6ACK":
            print(f"ERROR: Lost sync at byte {bytes_written:,} (expected ACK, got {ack!r})")
            ser.close()
            return False

        # One more N6ACK every 16 KB.
        if bytes_since_16k >= 16384:
            ack = ser.readline().decode(errors="ignore").strip()
            if ack != "N6ACK":
                print(f"ERROR: Lost sync at byte {bytes_written:,} (expected 16 KB ACK, got {ack!r})")
                ser.close()
                return False
            bytes_since_16k = 0

        pct = bytes_written * 100 // total_size
        if pct >= last_progress + 5 or bytes_written >= total_size:
            last_progress = pct
            print(f"  Progress  : {bar(bytes_written, total_size)}  {bytes_written:,} / {total_size:,} bytes")

    elapsed = time.time() - start_time
    speed = total_size / elapsed / 1024 if elapsed > 0 else 0
    print(f"  Write     : {total_size:,} bytes in {elapsed:.1f}s ({speed:.1f} KB/s)")

    print()
    step(4, 4, "Finalize and reset")
    print(f"  Progress  : {bar(4, 4)}")
    ser.write(make_packet(b"DFU_END"))

    # The device replies DFU_DONE and resets, possibly before the reply arrives.
    try:
        response = ser.readline().decode(errors="ignore").strip()
        if response == "DFU_DONE":
            print("  Device    : acknowledged; resetting")
    except Exception:
        pass

    ser.close()
    print()
    line()
    print("| Upload complete                                           |")
    line()
    print(f"  Wrote     : {total_size:,} bytes to flash")
    print("  Status    : done; the board should reappear shortly")

    # The Arduino IDE needs the output flushed and a pause before the exit.
    sys.stdout.flush()
    sys.stderr.flush()
    time.sleep(0.5)

    return True

def main():
    parser = argparse.ArgumentParser(description="Neuro N6 DFU Uploader (CLI)")
    parser.add_argument("--port", required=False, help="Serial port (e.g., COM3) - auto-detected if not specified")
    parser.add_argument("--bin", required=True, help="Path to binary file")
    parser.add_argument("--address", required=False, default=f"0x{FLASH_BASE + DEFAULT_FLASH_OFFSET:08X}",
                        help=f"Flash address in hex (default: 0x{FLASH_BASE + DEFAULT_FLASH_OFFSET:08X})")
    parser.add_argument("--no-reboot", action="store_true", help="Skip auto-reboot (device already in bootloader)")

    args = parser.parse_args()

    flash_offset = parse_address(args.address)
    if flash_offset is None:
        sys.exit(1)

    selected = port_identity(args.port)
    ignore = set()
    if selected in BOOTLOADER_IDENTITIES[:2]:
        boot = args.port
    elif selected == APP_IDENTITIES[0] and not args.no_reboot:
        # The selected app runs, so a bootloader elsewhere is another board's.
        boot = None
        ignore = {p.device for p in serial.tools.list_ports.comports()
                  if p.device != args.port and is_bootloader_family(p)}
    else:
        boot = find_bootloader_port()
    if boot is not None:
        banner("Bootloader already present")
        print(f"  Found     : {boot}")
        success = run_dfu(boot, args.bin, flash_offset)
        sys.exit(0 if success else 1)

    port = args.port

    if port:
        if not args.no_reboot:
            banner("Reboot to bootloader")
            if not _port_is_neuron6_app(port):
                app = find_app_port()
                if app:
                    print(f"  Port      : selected {port} is not the Neuro N6 app; using {app}")
                    port = app

            # The bootloader lands on a different COM port from the app's.
            port = enter_from_app(port, ignore)
            if port is None:
                print("ERROR: Bootloader port not found after reboot.")
                print("       Double-tap reset to enter bootloader mode, then upload again.")
                sys.exit(1)
    else:
        if not args.no_reboot:
            banner("Auto-detect board")
            app = find_app_port()
            if app is None:
                print("  App       : not found; board may already be in bootloader")
            else:
                print(f"  App       : Neuro N6 on {app}")
                port = enter_from_app(app)

        if port is None:
            port = find_bootloader_port()
        if port is None:
            port = wait_for_bootloader_port(timeout_sec=15)
        if port is None:
            print("ERROR: No device found.")
            print("       Double-tap reset to enter bootloader mode, then upload again.")
            sys.exit(1)

    success = run_dfu(port, args.bin, flash_offset)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
