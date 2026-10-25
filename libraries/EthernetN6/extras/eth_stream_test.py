#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Wired Ethernet stream check: prints the frame rate and saves the first frame.

Usage:
    python eth_stream_test.py                 # 192.168.20.1:5000
    python eth_stream_test.py 192.168.20.1 5000

The host's Ethernet NIC must be on 192.168.20.2/24.
"""
import socket, struct, sys, time

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.20.1"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 5000
MAGIC = 0x30564E44          # "DNV0" LE, the board's vnd_frame magic
HDR = struct.Struct("<5I")  # magic, frame_id, meta_len, payload_len, flags


def recvall(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("peer closed")
        buf += chunk
    return bytes(buf)


def main():
    print(f"connecting to {HOST}:{PORT} ...")
    s = socket.create_connection((HOST, PORT), timeout=10)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print("connected, waiting for frames (Ctrl-C to stop)")

    frames = 0
    bytes_rx = 0
    t0 = time.time()
    saved = False
    try:
        while True:
            magic, fid, meta_len, jpeg_len, flags = HDR.unpack(recvall(s, HDR.size))
            if magic != MAGIC:
                print(f"BAD MAGIC 0x{magic:08X}, stream desynced"); break
            meta = recvall(s, meta_len) if meta_len else b""
            jpeg = recvall(s, jpeg_len)
            frames += 1
            bytes_rx += HDR.size + meta_len + jpeg_len

            if not saved and jpeg[:2] == b"\xff\xd8" and jpeg[-2:] == b"\xff\xd9":
                with open("frame0.jpg", "wb") as f:
                    f.write(jpeg)
                print(f"saved frame0.jpg ({jpeg_len} bytes, valid SOI/EOI)")
                saved = True

            now = time.time()
            if now - t0 >= 1.0:
                mbps = (bytes_rx * 8) / (now - t0) / 1e6
                print(f"{frames} fps | {mbps:5.1f} Mbit/s | last jpeg {jpeg_len} B | meta {meta_len} B")
                frames = 0; bytes_rx = 0; t0 = now
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        s.close()


if __name__ == "__main__":
    main()
