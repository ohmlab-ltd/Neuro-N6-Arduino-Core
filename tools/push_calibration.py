#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Push a camera calibration to a Neuro N6 board as a persistent asset.

    python push_calibration.py --width 640 --height 480 \
        --fx 520.1 --fy 518.4 --cx 322.7 --cy 239.1 \
        --k1 -0.041 --k2 0.0032

--width and --height are the resolution the numbers were measured at. The
board resets when the transfer commits, and the FSBL programs the asset.
"""
import argparse, struct, sys, time, zlib

CAL_MAGIC   = 0x4C41434E   # 'NCAL'
CAL_VERSION = 1
AST0, AST1, AST2 = 0x30545341, 0x31545341, 0x32545341
OUT_EP, CHUNK = 0x01, 48


def build(a):
    return struct.pack("<IHHH4f5f", CAL_MAGIC, CAL_VERSION, a.width, a.height,
                       a.fx, a.fy, a.cx, a.cy, a.k1, a.k2, a.p1, a.p2, a.k3)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--width",  type=int, required=True)
    p.add_argument("--height", type=int, required=True)
    p.add_argument("--fx", type=float, required=True)
    p.add_argument("--fy", type=float, required=True)
    p.add_argument("--cx", type=float, required=True)
    p.add_argument("--cy", type=float, required=True)
    for k in ("k1", "k2", "p1", "p2", "k3"):
        p.add_argument("--" + k, type=float, default=0.0)
    p.add_argument("--vid", type=lambda x: int(x, 0), default=0x1209)
    p.add_argument("--pid", type=lambda x: int(x, 0), default=0x8008)
    a = p.parse_args()

    try:
        import usb.core, libusb_package
    except ImportError:
        sys.exit("needs pyusb and libusb-package:  pip install pyusb libusb-package")

    dev = usb.core.find(idVendor=a.vid, idProduct=a.pid,
                        backend=libusb_package.get_libusb1_backend())
    if dev is None:
        sys.exit("no Neuro N6 found (is it running an application, not the bootloader?)")

    payload = build(a)
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    print("calibration : %d bytes, crc 0x%08X" % (len(payload), crc))
    print("measured at : %dx%d" % (a.width, a.height))

    # Version 0: the device picks one newer than it has.
    dev.write(OUT_EP, struct.pack("<IIII", AST0, len(payload), crc, 0))
    time.sleep(0.05)
    for off in range(0, len(payload), CHUNK):
        part = payload[off:off + CHUNK]
        dev.write(OUT_EP, struct.pack("<IHB", AST1, off, len(part)) + part)
        time.sleep(0.03)
    dev.write(OUT_EP, struct.pack("<I", AST2))
    print("committed   : the board is resetting so the FSBL can program it")


if __name__ == "__main__":
    main()
