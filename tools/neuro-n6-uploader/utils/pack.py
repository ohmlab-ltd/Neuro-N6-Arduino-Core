# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
# DFU packet framing.
import struct
import sys

STX = 0x02
ETX = 0x03

def chunkate(path: str, chunk_size: int):
    try:
        with open(path, "rb") as f:
            data = f.read()

        total_size = len(data)

        chunks = [
            data[i:i + chunk_size]
            for i in range(0, total_size, chunk_size)
        ]

        return chunks, total_size

    except Exception as e:
        print(f"Unable to split binary into chunks: {e}")
        sys.exit(1)

# Packet: STX 0x02 | payload length, uint16 LE | payload | XOR of payload bytes | ETX 0x03
def make_packet(data: bytes):
    length = len(data)
    if length > 0xFFFF:
        raise ValueError(f"Payload too large for LEN16: {length} bytes")

    checksum = 0
    for b in data:
        checksum ^= b

    return bytes([STX]) + struct.pack("<H", length) + data + bytes([checksum, ETX])

def pack(chunks: list):
    return [make_packet(chunk) for chunk in chunks]

def calculate_erase_size(binary_size: int, sector_size: int):
    return ((binary_size + sector_size - 1) // sector_size) * sector_size
