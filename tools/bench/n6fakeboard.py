#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Stand-in for a board running extras/bench/WiFiTwoWay on the network link.

    python tools/bench/n6fakeboard.py &
    python tools/bench/n6bench.py wifitest --host 127.0.0.1 --soak 3
"""
import argparse
import select
import socket
import struct
import sys
import threading
import time

VND_MAGIC = 0x30564E44
HDR = struct.Struct("<IIIII")
CMD_MAX = 64
SUB_TTL = 3.0          # N6_HOST_SUB_TTL_MS
LOG_WINDOW = 2.0       # VND_DEBUG_LOG_TTL_MS
LOG_BYTES = 1280       # VND_DEBUG_LOG_STORE
STALL = 5.0            # N6_NET_STALL_MS
CROP = 64              # VND_VISION_CROP_DIM


def tlv(tag, value):
    return struct.pack("<BH", tag, len(value)) + value


def frame(fid, meta, payload=b""):
    return HDR.pack(VND_MAGIC, fid, len(meta), len(payload), 1 if payload else 0) + meta + payload


def jpeg_of(level):
    """A JPEG of a flat grey, or a blob with the right markers without Pillow."""
    try:
        from PIL import Image
        import io
        buf = io.BytesIO()
        Image.new("L", (160, 120), max(0, min(255, int(level)))).save(buf, "JPEG")
        return buf.getvalue()
    except ImportError:
        return b"\xff\xd8" + bytes(2048) + b"\xff\xd9"


class FakeBoard:
    def __init__(self, bind, port, name="neuron6-fake00"):
        self.name = name
        self.bind, self.port = bind, port
        self.level, self.flag = 50, 0
        self.auto_exposure, self.exposure_us = True, 8000
        self.sub_mask, self.sub_until = 0, 0.0
        self.log, self.log_next = [], 1        # (number, text, time)
        self.evidence = b""                    # last thumbnail TLV, kept after a lapse
        self.shown = {}
        self.fid = 0
        self.boot = ["[BOOT] reset: fake board", "[bench] WiFiTwoWay ready"]

    # The Terminal log
    def say(self, text):
        self.log.append((self.log_next, text, time.time()))
        self.log_next += 1

    def log_tlvs(self):
        now = time.time()
        window, used = [], 0
        for num, text, at in self.log:
            if now - at > LOG_WINDOW:
                continue
            used += 1 + len(text)
            if used > LOG_BYTES:
                break
            window.append((num, text))
        out = b""
        if self.log:
            out += tlv(0x17, self.log[-1][1].encode() + b"\r\n")
        first = window[0][0] if window else self.log_next
        rec = struct.pack("<IB", first, len(window))
        for _num, text in window:
            rec += bytes([len(text)]) + text.encode()
        out += tlv(0x71, rec)
        return out

    # Announces
    def descriptor(self):
        out = bytes([2])
        for pid, typ, name, lo, hi, cur in ((0, 0, b"level", 0, 100, self.level),
                                            (1, 2, b"flag", 0, 1, self.flag)):
            out += bytes([pid, typ, 0, len(name)]) + name + struct.pack("<iii", lo, hi, cur)
        return out

    def announce_frames(self):
        ip = socket.inet_aton(self.bind if self.bind != "0.0.0.0" else "127.0.0.1")
        for t, v in ((0x20, ip + struct.pack("<I", self.port)),
                     (0x64, bytes([0x07])),      # exposure, gain, white balance
                     (0x65, self.descriptor()),
                     (0x60, bytes([0x01]))):
            self.fid += 1
            yield frame(self.fid, tlv(t, v))

    def ip_frame(self):
        ip = socket.inet_aton(self.bind if self.bind != "0.0.0.0" else "127.0.0.1")
        return frame(0, tlv(0x20, ip + struct.pack("<I", self.port)) +
                     tlv(0x21, self.name.encode()))

    # Per-frame work
    def wanted(self):
        return bool(self.sub_mask & 1) and time.time() < self.sub_until

    def tick_slots(self):
        """The sketch's loop: the tune and gate lines, and the thumbnail."""
        state = (self.level, self.flag)
        if self.shown.get("tune") != state:
            self.shown["tune"] = state
            self.say("[tune] level=%d flag=%d" % state)
        gate = self.wanted()
        if self.shown.get("gate") != gate:
            self.shown["gate"] = gate
            self.say("[gate] crops wanted: %s" % ("yes" if gate else "no"))
        if gate:
            px = bytes([0x1F, 0x00]) * (CROP * CROP)      # RGB565 blue
            self.evidence = tlv(0x70, bytes([1, 0, CROP, CROP]) + px)

    def video_frame(self):
        self.fid += 1
        meta = self.log_tlvs() + tlv(0x61, bytes([1]) + struct.pack("<HHHH", 500, 500, 400, 400) +
                                     bytes([1, 0]) + struct.pack("<H", 5) + b"bench") + self.evidence
        level = 128 if self.auto_exposure else min(255, self.exposure_us / 250.0)
        return frame(self.fid, meta, jpeg_of(level))

    # Commands
    def command(self, cmd):
        magic = cmd[:4]
        if magic == b"TUN0" and len(cmd) >= 10:
            idx, _typ, value = cmd[4], cmd[5], struct.unpack_from("<i", cmd, 6)[0]
            if idx == 0:
                self.level = max(0, min(100, value))
            elif idx == 1:
                self.flag = 1 if value else 0
        elif magic == b"SUB0" and len(cmd) >= 8:
            self.sub_mask = struct.unpack_from("<I", cmd, 4)[0]
            self.sub_until = time.time() + SUB_TTL if self.sub_mask else 0.0
        elif magic == b"CAM0" and len(cmd) >= 12:
            flags, exposure, _gain, apply_mask = struct.unpack_from("<BIHB", cmd, 4)
            if apply_mask & 0x01:
                self.auto_exposure = bool(flags & 0x01)
            if apply_mask & 0x08:
                self.exposure_us = exposure
        # ISP0, the asset commands and REBOOT_DFU are ignored.

    # Serving one host
    def serve(self, conn):
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.setblocking(False)
        self.evidence = b""
        self.shown.clear()
        self.sub_mask, self.sub_until = 0, 0.0
        self.log = []
        self.say("[bootlog] --- pre-connect boot log ---")
        for line in self.boot:
            self.say(line)
        self.say("[bootlog] --- end of boot log, live from here ---")
        print("fakeboard: host connected")

        out, rx = bytearray(), bytearray()
        beats = 0
        t_frame = t_beat = t_ann = t_slots = 0.0
        progress = time.time()
        while True:
            now = time.time()
            if now - t_slots >= 0.2:
                t_slots = now
                self.tick_slots()
            if now - t_beat >= 1.0:
                t_beat = now
                beats += 1
                self.say("[beat] %d" % beats)
            if now - t_ann >= 1.0:
                t_ann = now
                for f in self.announce_frames():
                    out += f
            if now - t_frame >= 1 / 15.0:
                t_frame = now
                out += self.video_frame()
            if len(out) > 8 << 20:      # a host that reads nothing at all
                del out[:len(out) - (8 << 20)]

            try:
                readable, writable, _ = select.select([conn], [conn] if out else [], [], 0.02)
            except OSError:
                break
            if writable:
                try:
                    sent = conn.send(bytes(out[:1 << 16]))
                except BlockingIOError:
                    sent = 0
                except OSError:
                    break
                if sent:
                    del out[:sent]
                    progress = now
            elif out and now - progress > STALL:
                print("fakeboard: host stopped reading - dropped")
                break
            if readable:
                try:
                    data = conn.recv(4096)
                except BlockingIOError:
                    data = None
                except OSError:
                    break
                if data == b"":
                    print("fakeboard: host closed")
                    break
                if data:
                    rx += data
            while len(rx) >= 20:
                magic, _fid, mlen, plen, _flags = HDR.unpack_from(rx, 0)
                if magic != VND_MAGIC or mlen != 0 or plen > CMD_MAX:
                    print("fakeboard: host sent something other than a command - dropped")
                    conn.close()
                    return
                if len(rx) < 20 + plen:
                    break
                if plen:
                    self.command(bytes(rx[20:20 + plen]))
                del rx[:20 + plen]
        conn.close()

    def run_udp(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.bind, self.port))
        while True:
            data, src = s.recvfrom(1024)
            if len(data) >= 20 and struct.unpack_from("<I", data, 0)[0] == VND_MAGIC:
                s.sendto(self.ip_frame(), src)
                print("fakeboard: answered a probe from %s:%d" % src)

    def run(self):
        threading.Thread(target=self.run_udp, daemon=True).start()
        print("fakeboard: TCP and UDP on %s:%d" % (self.bind, self.port))
        while True:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.bind, self.port))
            srv.listen(1)
            conn, _peer = srv.accept()
            srv.close()          # one host at a time: others are refused
            try:
                self.serve(conn)
            except Exception as e:  # noqa: BLE001
                print("fakeboard: %s" % e)
                try:
                    conn.close()
                except OSError:
                    pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--name", default="neuron6-fake00", help="the name its discovery reply carries")
    args = ap.parse_args()
    try:
        FakeBoard(args.bind, args.port, args.name).run()
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
