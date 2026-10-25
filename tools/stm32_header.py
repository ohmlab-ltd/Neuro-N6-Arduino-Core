#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Wrap a binary in the STM32 image header, version 2.3, without a key.

This is the header STM32_SigningTool_CLI writes for
    -nk -of 0x80000000 -t fsbl -hv 2.3 -align
with zeros in place of ST's random extension padding. The bootloader jumps to
offset 0x400 and reads no field.

Layout, offsets in bytes, every field little-endian:

    0x000  magic             "STM2"
    0x004  signature         96 bytes, zero: no key
    0x064  checksum          sum of payload bytes, modulo 2^32
    0x068  header version    0x00020300
    0x06C  image length      head padding plus padded payload
    0x070  entry point       payload word 1, the reset vector
    0x074  reserved          0
    0x078  load address      0xFFFFFFFF
    0x07C  reserved          0
    0x080  image version     0
    0x084  option flags      0x80000000
    0x088  extension length  0x1A0
    0x08C  binary type       0x10, FSBL
    0x090  padding           16 bytes, zero
    0x0A0  extension         type "ST" 0xFF 0xFF, length 0x1A0, then padding
    0x240  head padding      zero, up to 0x400
    0x400  payload           zero padded at the end to a multiple of 32

    python stm32_header.py app.bin app_Trusted.bin
    python stm32_header.py --describe app_Trusted.bin
"""
import struct
import sys

MAGIC = 0x324D5453
HEADER_VERSION = 0x00020300
PAYLOAD_OFFSET = 0x400
EXTENSION_OFFSET = 0xA0
EXTENSION_LENGTH = 0x1A0
EXTENSION_TYPE_PADDING = bytes([0x53, 0x54, 0xFF, 0xFF])
BINARY_TYPE_FSBL = 0x10
OPTION_FLAGS_NO_KEY = 0x80000000
LOAD_ADDRESS_NONE = 0xFFFFFFFF
PAYLOAD_ALIGN = 32
HEAD_PADDING = PAYLOAD_OFFSET - EXTENSION_OFFSET - EXTENSION_LENGTH


def pad_payload(payload):
    """Zero-pad to a multiple of 32 bytes."""
    rem = len(payload) % PAYLOAD_ALIGN
    return payload if not rem else payload + bytes(PAYLOAD_ALIGN - rem)


def build_header(payload, image_version=0, option_flags=OPTION_FLAGS_NO_KEY,
                 binary_type=BINARY_TYPE_FSBL):
    """The 1024-byte header for an already padded payload."""
    entry = struct.unpack_from("<I", payload, 4)[0] if len(payload) >= 8 else 0
    hdr = bytearray(PAYLOAD_OFFSET)
    struct.pack_into("<I", hdr, 0x000, MAGIC)
    struct.pack_into("<I", hdr, 0x064, sum(payload) & 0xFFFFFFFF)
    struct.pack_into("<I", hdr, 0x068, HEADER_VERSION)
    struct.pack_into("<I", hdr, 0x06C, HEAD_PADDING + len(payload))
    struct.pack_into("<I", hdr, 0x070, entry)
    struct.pack_into("<I", hdr, 0x078, LOAD_ADDRESS_NONE)
    struct.pack_into("<I", hdr, 0x080, image_version)
    struct.pack_into("<I", hdr, 0x084, option_flags)
    struct.pack_into("<I", hdr, 0x088, EXTENSION_LENGTH)
    struct.pack_into("<I", hdr, 0x08C, binary_type)
    hdr[EXTENSION_OFFSET:EXTENSION_OFFSET + 4] = EXTENSION_TYPE_PADDING
    struct.pack_into("<I", hdr, EXTENSION_OFFSET + 4, EXTENSION_LENGTH)
    return bytes(hdr)


def wrap(payload, **kw):
    """Header plus padded payload: the file the uploader flashes."""
    padded = pad_payload(payload)
    return build_header(padded, **kw) + padded


def describe(image):
    """Field table for a wrapped image."""
    u32 = lambda off: struct.unpack_from("<I", image, off)[0]
    payload = image[PAYLOAD_OFFSET:]
    ext_pad = image[EXTENSION_OFFSET + 8:EXTENSION_OFFSET + EXTENSION_LENGTH]
    return [
        ("magic", "%s (%s)" % (image[:4], "ok" if u32(0) == MAGIC else "BAD")),
        ("signature", "zero, no key" if not any(image[4:0x64]) else "present"),
        ("checksum", "0x%08X (%s)" % (u32(0x64), "ok" if u32(0x64) == sum(payload) & 0xFFFFFFFF else "MISMATCH")),
        ("header version", "0x%08X" % u32(0x68)),
        ("image length", "0x%X (%s)" % (u32(0x6C), "ok" if u32(0x6C) == HEAD_PADDING + len(payload) else "MISMATCH")),
        ("entry point", "0x%08X" % u32(0x70)),
        ("load address", "0x%08X" % u32(0x78)),
        ("image version", "%d" % u32(0x80)),
        ("option flags", "0x%08X" % u32(0x84)),
        ("binary type", "0x%02X" % u32(0x8C)),
        ("extension", "type %s, length 0x%X, padding %s" % (
            bytes(image[EXTENSION_OFFSET:EXTENSION_OFFSET + 4]), u32(EXTENSION_OFFSET + 4),
            "zero" if not any(ext_pad) else "random (ST tool)")),
        ("payload", "%d bytes at 0x%X" % (len(payload), PAYLOAD_OFFSET)),
    ]


def main(argv):
    if len(argv) == 3 and argv[1] == "--describe":
        for name, value in describe(open(argv[2], "rb").read()):
            print("  %-16s %s" % (name, value))
        return 0
    if len(argv) != 3:
        print("usage: stm32_header.py <in.bin> <out.bin> | --describe <image.bin>", file=sys.stderr)
        return 2
    with open(argv[1], "rb") as fh:
        payload = fh.read()
    with open(argv[2], "wb") as fh:
        fh.write(wrap(payload))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
