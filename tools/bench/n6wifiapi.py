#!/usr/bin/env python3
"""Checks the WiFi API on a board running extras/bench/WiFiApi. Windows only.

    python tools/bench/n6wifiapi.py [--port PORT] [--phases pass,ap,sta,stored,link] [--json out.json]

pass    The board's own SoftAP password. This phase replaces it.
ap      The board hosts a WPA2 or WPA3 access point and this PC joins it.
        Other security is refused, and the host server waits for a stream.
sta     The board joins a Wi-Fi Direct group owner this PC hosts, or the
        2.4 GHz network named in N6_STA_SSID and N6_STA_KEY. No host server
        runs on it.
stored  WiFi settings stored over USB, then WiFi.begin() with no arguments.
link    The link key.

The firewall must let python.exe in on Public networks.
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT

import argparse
import hashlib
import json
import os
import re
import secrets
import socket
import string
import struct
import subprocess
import sys
import tempfile
import threading
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build_n6radio  # noqa: E402

IFACE = "WiFi"
TCP_ECHO, HTTP, UDP_SINK, BOARD_SRV, BOARD_UDP, MCAST_PORT = 5601, 5602, 5603, 5605, 5606, 5607
MCAST_GROUP = "239.255.0.7"
DNS_NAME, DNS_ANSWER = "bench.n6.test", "10.99.0.7"


def random_key(n=20):
    return "".join(secrets.choice(string.ascii_letters + string.digits) for _ in range(n))


def find_port():
    """The serial port of the one Neuro N6 running a sketch (USB 1209:8008)."""
    from serial.tools import list_ports
    ports = [p.device for p in list_ports.comports() if (p.vid, p.pid) == (0x1209, 0x8008)]
    if len(ports) != 1:
        raise SystemExit("%d Neuro N6 boards running a sketch found; name one with --port" % len(ports))
    return ports[0]


# The board's shell

class Shell:
    def __init__(self, port, log_path, prefix="[api]"):
        self.port = port
        self.prefix = prefix
        self.s = serial.Serial(port, 115200, timeout=0.05)
        self.log = open(log_path, "w", encoding="utf-8")
        self.lines = []
        self.buf = b""
        self.secrets = []

    def hide(self, text):
        text = re.sub(r"\bpass key=\S+", "pass key=<key>", text)
        text = re.sub(r'^(\[api\] net \d+ )ssid="[^"]*"', r"\1ssid=<hidden>", text)
        for k in self.secrets:
            text = text.replace(k, "<key>")
        return text

    def pump(self):
        data = self.s.read(8192)
        if not data:
            return
        self.buf += data
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            line = raw.decode("utf-8", errors="replace").rstrip("\r")
            self.lines.append(line)
            self.log.write("%.3f %s\n" % (time.time(), self.hide(line)))
        self.log.flush()

    def cmd(self, text, timeout=30.0):
        name = text.split()[0]
        self.log.write(">> %s\n" % self.hide(text))
        start = len(self.lines)
        self.s.write((text + "\n").encode())
        deadline = time.time() + timeout
        end = re.compile(r"^%s (ok|fail) %s\b" % (re.escape(self.prefix), re.escape(name)))
        while time.time() < deadline:
            self.pump()
            for i in range(start, len(self.lines)):
                m = end.match(self.lines[i])
                if m:
                    lead = self.prefix + " "
                    api = [l[len(lead):] for l in self.lines[start:i + 1]
                           if l.startswith(lead) and not l.startswith(lead + "beat ")]
                    return m.group(1) == "ok", api
            time.sleep(0.02)
        return False, ["timeout after %.0f s" % timeout]

    def wait_for(self, pattern, timeout, since=None):
        rx = re.compile(pattern)
        start = len(self.lines) if since is None else since
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.pump()
            for l in self.lines[start:]:
                if rx.search(l):
                    return l
            time.sleep(0.05)
        return None

    def mark(self):
        self.pump()
        return len(self.lines)

    def reopen(self, timeout=40.0):
        """Reopens the port after a board restart. It comes back under the same name."""
        try:
            self.s.close()
        except serial.SerialException:
            pass
        self.log.write("-- board restart --\n")
        deadline = time.time() + timeout
        while time.time() < deadline:
            time.sleep(1.0)
            try:
                self.s = serial.Serial(self.port, 115200, timeout=0.05)
                self.buf = b""
                return True
            except serial.SerialException:
                pass
        return False

    def close(self):
        self.s.close()
        self.log.close()


def field(api_lines, key):
    for l in api_lines:
        m = re.search(r"\b%s=(\"[^\"]*\"|\S+)" % re.escape(key), l)
        if m:
            return m.group(1).strip('"')
    return None


# This PC's WiFi

PROFILE = """<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>{name}</name>
  <SSIDConfig><SSID><name>{ssid}</name></SSID></SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>manual</connectionMode>
  <MSM><security>
    <authEncryption><authentication>{auth}</authentication><encryption>{enc}</encryption><useOneX>false</useOneX></authEncryption>
    {key}
  </security></MSM>
</WLANProfile>
"""


def netsh(*args):
    return subprocess.run(["netsh", "wlan"] + list(args), capture_output=True, text=True).stdout


def wlan_state():
    out = netsh("show", "interfaces")
    state = re.search(r"^\s*State\s*:\s*(\S+)", out, re.M)
    ssid = re.search(r"^\s*SSID\s*:\s*(.+)$", out, re.M)
    bssid = re.search(r"^\s*(?:AP )?BSSID\s*:\s*(\S+)", out, re.M)
    return (state.group(1) if state else "", ssid.group(1).strip() if ssid else "",
            bssid.group(1).upper() if bssid else "")


def wait_left(ssid, timeout=20):
    """Seconds until this PC stops reporting being joined to `ssid`, or None."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        state, cur, _ = wlan_state()
        if state != "connected" or cur != ssid:
            return round(time.time() - t0, 1)
        time.sleep(0.5)
    return None


def local_ip_towards(ip):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((ip, 9))
        a = s.getsockname()[0]
        s.close()
        return a
    except OSError:
        return None


class Join:
    """A temporary manual-connect profile for one network, removed on leave()."""

    def __init__(self, ssid, key, auth="WPA2PSK"):
        self.ssid, self.key, self.auth = ssid, key, auth
        self.name = "n6bench-" + ssid

    def join(self, board_ip, timeout=40):
        key = ('<sharedKey><keyType>passPhrase</keyType><protected>false</protected>'
               '<keyMaterial>%s</keyMaterial></sharedKey>' % self.key) if self.key else ""
        xml = PROFILE.format(name=self.name, ssid=self.ssid,
                             auth=self.auth if self.key else "open",
                             enc="AES" if self.key else "none", key=key)
        fd, path = tempfile.mkstemp(suffix=".xml")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                f.write(xml)
            netsh("add", "profile", "filename=%s" % path, "interface=%s" % IFACE, "user=current")
        finally:
            os.remove(path)
        deadline = time.time() + timeout
        while time.time() < deadline:
            netsh("connect", "name=%s" % self.name, "interface=%s" % IFACE)
            for _ in range(20):
                time.sleep(0.5)
                state, ssid, _ = wlan_state()
                mine = local_ip_towards(board_ip)
                if state == "connected" and ssid == self.ssid and mine and \
                        mine.rsplit(".", 1)[0] == board_ip.rsplit(".", 1)[0]:
                    return mine
        return None

    def leave(self):
        netsh("disconnect", "interface=%s" % IFACE)
        netsh("delete", "profile", "name=%s" % self.name, "interface=%s" % IFACE)


# Servers on this PC

class Servers:
    def __init__(self):
        self.udp_rx = []
        self.stop = False
        self.threads = []
        self.dns_sock = None
        self.ntp_error = None

    def start(self):
        for target in (self.tcp_echo, self.http, self.udp_sink, self.ntp):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self.threads.append(t)

    def dns_on(self, ip):
        """DNS answers only on this PC's address on the board's network."""
        if self.dns_sock is not None:
            if self.dns_sock.getsockname()[0] == ip:
                return True
            self.dns_sock.close()   # its thread ends on the error
            self.dns_sock = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind((ip, 53))
        except OSError as e:
            print("  (no DNS server here: %s)" % e)
            return False
        self.dns_sock = s
        threading.Thread(target=self.dns, args=(s,), daemon=True).start()
        return True

    def tcp_echo(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", TCP_ECHO))
        srv.listen(4)
        while not self.stop:
            c, _ = srv.accept()
            threading.Thread(target=self._echo, args=(c,), daemon=True).start()

    @staticmethod
    def _echo(c):
        try:
            while True:
                d = c.recv(65536)
                if not d:
                    break
                c.sendall(d)
        except OSError:
            pass
        c.close()

    def http(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", HTTP))
        srv.listen(4)
        body = bytes((i * 13) & 0xFF for i in range(5000))
        while not self.stop:
            c, _ = srv.accept()
            try:
                c.settimeout(5)
                req = b""
                while b"\r\n\r\n" not in req:
                    d = c.recv(4096)
                    if not d:
                        break
                    req += d
                c.sendall(b"HTTP/1.0 200 OK\r\nContent-Type: application/octet-stream\r\n"
                          b"Content-Length: %d\r\n\r\n" % len(body) + body)
            except OSError:
                pass
            c.close()

    def udp_sink(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("0.0.0.0", UDP_SINK))
        while not self.stop:
            d, a = s.recvfrom(2048)
            self.udp_rx.append((a, d))

    def ntp(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("0.0.0.0", 123))
        except OSError as e:
            self.ntp_error = "UDP port 123 on this PC is taken by another program (%s)" % e.strerror
            s.close()
            return
        while not self.stop:
            try:
                d, a = s.recvfrom(512)
            except OSError:
                # Windows raises here after an ICMP port unreachable for an earlier reply.
                continue
            if len(d) < 48 or (d[0] & 7) != 3:
                continue
            now = time.time() + 2208988800
            ts = struct.pack(">II", int(now), int((now % 1) * 2 ** 32))
            reply = bytes([0x24, 2, 6, 0xEC]) + b"\0" * 8 + b"LOCL" + ts + d[40:48] + ts + ts
            s.sendto(reply, a)

    @staticmethod
    def dns(s):
        while True:
            try:
                d, a = s.recvfrom(512)
            except OSError:
                return
            if len(d) < 17:
                continue
            p, labels = 12, []
            while p < len(d) and d[p]:
                labels.append(d[p + 1:p + 1 + d[p]].decode(errors="replace"))
                p += 1 + d[p]
            qname, question = ".".join(labels).lower(), d[12:p + 5]
            if qname == DNS_NAME and d[p + 1:p + 3] == b"\0\1":
                ans = b"\xc0\x0c\0\1\0\1\0\0\0\x3c\0\x04" + socket.inet_aton(DNS_ANSWER)
                s.sendto(d[:2] + b"\x81\x80\0\1\0\1\0\0\0\0" + question + ans, a)
            else:
                s.sendto(d[:2] + b"\x81\x83\0\1\0\0\0\0\0\0" + question, a)


# mDNS, asked directly

def _dns_name(data, off):
    """Reads a possibly compressed DNS name. Returns (name, offset after it)."""
    labels, jumped, end = [], False, off
    for _ in range(64):
        n = data[off]
        if n == 0:
            off += 1
            break
        if n & 0xC0 == 0xC0:
            if not jumped:
                end = off + 2
            jumped, off = True, ((n & 0x3F) << 8) | data[off + 1]
            continue
        labels.append(data[off + 1:off + 1 + n].decode(errors="replace"))
        off += 1 + n
    return ".".join(labels), (end if jumped else off)


def mdns_query(name, from_ip, qtype=1, timeout=2.0):
    """One-shot mDNS query from this PC's address `from_ip`. Returns (name, type,
    value) tuples: value is the address for A, the target for PTR, (port,
    target) for SRV."""
    qid = secrets.randbelow(65536)
    q = struct.pack(">HHHHHH", qid, 0, 1, 0, 0, 0)
    for label in name.split("."):
        q += bytes([len(label)]) + label.encode()
    q += b"\0" + struct.pack(">HH", qtype, 1)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((from_ip, 0))
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(from_ip))
    s.settimeout(timeout)
    answers = []
    try:
        s.sendto(q, ("224.0.0.251", 5353))
        data, _ = s.recvfrom(2048)
        qd, an = struct.unpack(">HH", data[4:8])
        off = 12
        for _ in range(qd):
            _, off = _dns_name(data, off)
            off += 4
        for _ in range(an):
            rname, off = _dns_name(data, off)
            rtype, _, _, rdlen = struct.unpack(">HHIH", data[off:off + 10])
            off += 10
            rd = data[off:off + rdlen]
            if rtype == 1:
                value = socket.inet_ntoa(rd)
            elif rtype == 12:
                value = _dns_name(data, off)[0]
            elif rtype == 33:
                value = (struct.unpack(">H", rd[4:6])[0], _dns_name(data, off + 6)[0])
            else:
                value = rd
            answers.append((rname, rtype, value))
            off += rdlen
    except (socket.timeout, OSError, struct.error, IndexError):
        pass
    finally:
        s.close()
    return answers


def discovery_checks(sh, rep, board_ip, pc_ip, name, label):
    import n6bench
    details = {}
    bcast = board_ip.rsplit(".", 1)[0] + ".255"
    n6bench.discover(5000, bcast, 2.0, details)
    got = details.get(board_ip, (None, False))
    rep.check("%s: the discovery reply carries the board's name" % label, got[0] == name, got)

    a = [v for (_, t, v) in mdns_query(name + ".local", pc_ip) if t == 1]
    rep.check("%s: mDNS answers %s.local" % (label, name), a == [board_ip], a)

    ptr = mdns_query("_neuron6._tcp.local", pc_ip, qtype=12)
    srv = [v for (_, t, v) in ptr if t == 33]
    rep.check("%s: DNS-SD lists the host server as _neuron6._tcp" % label,
              any(v == "%s._neuron6._tcp.local" % name for (_, t, v) in ptr if t == 12) and
              srv and srv[0][0] == 5000, [(t, v) for (_, t, v) in ptr])

    try:
        win = socket.getaddrinfo(name + ".local", 5000, socket.AF_INET)[0][4][0]
    except OSError as e:
        win = str(e)
    rep.check("%s: Windows itself resolves %s.local" % (label, name), win == board_ip, win)


def server_answers(board_ip, pc_ip, name):
    """Which parts of the host server answer: TCP 5000, discovery on UDP 5000
    and mDNS."""
    import n6bench
    try:
        socket.create_connection((board_ip, 5000), timeout=3).close()
        tcp = True
    except OSError:
        tcp = False
    details = {}
    n6bench.discover(5000, board_ip, 2.0, details)
    mdns = [v for (_, t, v) in mdns_query(name + ".local", pc_ip) if t == 1]
    return {"tcp": tcp, "udp": board_ip in details, "mdns": bool(mdns)}


def board_password(sh):
    """The board's own password, storing it first when there is none."""
    import n6bench
    for _ in range(2):
        ok, api = sh.cmd("pass", 20)
        if ok and "ok pass storing" in api:
            sh.s.close()
            if not (n6bench.wait_reset() and sh.reopen()):
                return None
            continue
        key = field(api, "key") if ok else None
        if key:
            sh.secrets.append(key)
        return key
    return None


def short_hash(key):
    return hashlib.sha256(key.encode()).hexdigest()[:8] if key else None


def shell_restart(sh, cmd):
    """A shell command that restarts the board, then the port reopened."""
    import n6bench
    ok = sh.cmd(cmd, 20)[0]
    sh.s.close()
    return n6bench.wait_reset() and sh.reopen() and ok


# Report

class Report:
    def __init__(self):
        self.rows = []

    def check(self, name, ok, detail=""):
        self.rows.append({"check": name, "result": "PASS" if ok else "FAIL", "detail": str(detail)})
        print("%s  %s%s" % ("PASS" if ok else "FAIL", name, ("  -- " + str(detail)) if detail else ""))
        return ok

    def info(self, name, detail):
        self.rows.append({"check": name, "result": "INFO", "detail": str(detail)})
        print("INFO  %s  -- %s" % (name, detail))

    def finish(self, path):
        failed = [r for r in self.rows if r["result"] == "FAIL"]
        passed = [r for r in self.rows if r["result"] == "PASS"]
        print("\n%d passed, %d failed" % (len(passed), len(failed)))
        if path:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.rows, f, indent=1)
        return 1 if failed else 0


# Checks shared by the ap and sta phases

def client_checks(sh, rep, pc_ip, board_ip, label):
    ok, api = sh.cmd("tcp %s %d 200000" % (pc_ip, TCP_ECHO), 60)
    rep.check("%s: WiFiClient echoes 200 kB through a server here" % label, ok,
              "; ".join(a for a in api if a.startswith("tcp ")))

    ok, api = sh.cmd("get %s %d /n6" % (pc_ip, HTTP), 30)
    rep.check("%s: WiFiClient HTTP GET" % label, ok and field(api, "body") == "5000",
              "; ".join(a for a in api if a.startswith("get ")))

    n0 = len(servers.udp_rx)
    ok, _ = sh.cmd("usend %s %d from-the-board" % (pc_ip, UDP_SINK))
    time.sleep(1.0)
    got = [d for a, d in servers.udp_rx[n0:] if a[0] == board_ip]
    rep.check("%s: WiFiUDP sends a datagram here" % label, ok and b"from-the-board" in got, got[:1])

    n0 = len(servers.udp_rx)
    ok, _ = sh.cmd("usend 255.255.255.255 %d board-broadcast" % UDP_SINK)
    time.sleep(1.0)
    got = [d for a, d in servers.udp_rx[n0:] if a[0] == board_ip]
    rep.check("%s: WiFiUDP broadcasts to 255.255.255.255" % label, ok and b"board-broadcast" in got, got[:1])

    ok, _ = sh.cmd("srv %d" % BOARD_SRV)
    good = False
    detail = "server did not start"
    if ok:
        try:
            c = socket.create_connection((board_ip, BOARD_SRV), timeout=5)
            c.settimeout(5)
            data = bytes((i * 31) & 0xFF for i in range(20000))
            back = b""
            for i in range(0, len(data), 1000):
                c.sendall(data[i:i + 1000])
                while len(back) < i + 1000:
                    d = c.recv(4096)
                    if not d:
                        break
                    back += d
            c.close()
            good, detail = back == data, "%d of %d bytes echoed" % (len(back), len(data))
        except OSError as e:
            detail = str(e)
    rep.check("%s: WiFiServer echoes 20 kB from a client here" % label, good, detail)

    good, detail = False, "server did not start"
    if ok:
        try:
            c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            c.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2048)
            c.settimeout(5)
            c.connect((board_ip, BOARD_SRV))
            c.setblocking(False)
            sent, t0 = 0, time.time()
            while time.time() - t0 < 3.0:          # never read, so the echo backs up
                try:
                    sent += c.send(bytes(1000))
                except BlockingIOError:
                    time.sleep(0.01)
            c.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
            c.close()                              # a reset, not a close
            t1 = time.time()
            good, _ = sh.cmd("stat", 5)
            detail = ("%d bytes sent and none read, then a reset; the shell answered in %.1f s"
                      % (sent, time.time() - t1)) if good else "the shell stopped answering after the reset"
        except OSError as e:
            detail = str(e)
    rep.check("%s: a client's reset while the echo is held up leaves the board answering" % label, good, detail)

    ok, _ = sh.cmd("udp %d" % BOARD_UDP)
    good, detail = False, "no socket"
    if ok:
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        u.settimeout(2)
        sizes, echoed = (1, 100, 1000, 1472), 0
        for n in sizes:
            payload = bytes((i + n) & 0xFF for i in range(n))
            for _ in range(3):
                u.sendto(payload, (board_ip, BOARD_UDP))
                try:
                    d, _ = u.recvfrom(2048)
                    if d == payload:
                        echoed += 1
                        break
                except socket.timeout:
                    pass
        u.close()
        good, detail = echoed == len(sizes), "%d of %d sizes (1..1472 bytes) echoed" % (echoed, len(sizes))
    rep.check("%s: WiFiUDP echoes datagrams from here" % label, good, detail)

    ok, _ = sh.cmd("mcast %s %d" % (MCAST_GROUP, MCAST_PORT))
    mark = sh.mark()
    m = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    m.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(pc_ip))
    m.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
    line = None
    for _ in range(5):
        m.sendto(b"n6-multicast", (MCAST_GROUP, MCAST_PORT))
        line = sh.wait_for(r"\[api\] mcast .*n6-multicast", 1.0, since=mark)
        if line:
            break
    m.close()
    rep.check("%s: WiFiUDP joins a multicast group" % label, ok and line is not None, line or "nothing heard")

    if servers.ntp_error:
        rep.info("%s: getTime from an NTP server here" % label, "not testable here: " + servers.ntp_error)
    else:
        ok, api = sh.cmd("time %s" % pc_ip, 20)
        epoch = int(field(api, "epoch") or 0)
        rep.check("%s: getTime from an NTP server here" % label, ok and abs(epoch - time.time()) < 3,
                  "board %d, PC %d" % (epoch, int(time.time())))

    ok, api = sh.cmd("resolve 10.1.2.3")
    rep.check("%s: hostByName takes a dotted quad" % label, ok and "-> 10.1.2.3" in " ".join(api), api[:1])

    if servers.dns_on(pc_ip):
        sh.cmd("dns %s" % pc_ip)
        ok, api = sh.cmd("resolve %s" % DNS_NAME, 20)
        rep.check("%s: hostByName asks the DNS server setDNS named" % label,
                  ok and ("-> %s" % DNS_ANSWER) in " ".join(api), api[:1])
        ok, api = sh.cmd("resolve nothing.n6.test", 20)
        rep.check("%s: hostByName reports a name that does not exist" % label, not ok, api[:1])


# Phases

PASSWORD = re.compile(r"[a-kmnp-z2-9]{4}(-[a-kmnp-z2-9]{4}){3}")


def phase_pass(sh, rep):
    import n6bench

    def appass():
        ok, api = sh.cmd("asset", 20)
        return field(api, "appass") if ok else None

    ok = shell_restart(sh, "pclear")
    rep.check("pass: removing the board's password restarts the board", ok and appass() == "no")
    k1 = board_password(sh)
    rep.check("pass: devicePassword() stores a new one, restarting the board once",
              k1 is not None and appass() == "yes", short_hash(k1))
    rep.check("pass: four groups of four from 32 letters and digits",
              k1 is not None and PASSWORD.fullmatch(k1) is not None, "%d characters" % len(k1 or ""))
    ok = shell_restart(sh, "reboot")
    k = board_password(sh)
    rep.check("pass: the same after a restart", ok and k1 is not None and k == k1,
              "%s, then %s" % (short_hash(k1), short_hash(k)))
    ok = shell_restart(sh, "pclear")
    k2 = board_password(sh)
    rep.check("pass: a different one once removed, as it is random, not derived from the chip",
              ok and k2 is not None and k2 != k1, "%s, then %s" % (short_hash(k1), short_hash(k2)))
    sh.s.close()
    ok = n6bench.store_record(n6bench.REC_WIFI, b"")
    ok = sh.reopen() and ok
    k = board_password(sh)
    rep.check("pass: storing WiFi settings over USB keeps it", ok and k2 is not None and k == k2,
              "%s, then %s" % (short_hash(k2), short_hash(k)))

    ssid = "N6PASS-%04d" % secrets.randbelow(10000)
    ok, api = sh.cmd("ap %s -" % ssid, 60)
    j = Join(ssid, k2 or "")
    try:
        pc_ip = j.join("192.168.10.1") if ok and k2 else None
        rep.check("pass: beginAP(ssid) hosts with it and this PC joins", pc_ip is not None, pc_ip)
    finally:
        j.leave()
        wait_left(ssid)
    sh.cmd("disc")


def phase_ap(sh, rep):
    ssid, key = "N6API-%04d" % secrets.randbelow(10000), random_key()
    sh.secrets.append(key)
    sh.cmd("cfg 0.0.0.0")
    ok, api = sh.cmd("ap %s %s 6" % (ssid, key), 60)
    rep.check("ap: beginAP comes up", ok, "; ".join(api[-2:]))
    if not ok:
        return
    board_ip = "192.168.10.1"
    j = Join(ssid, key)
    try:
        pc_ip = j.join(board_ip)
        rep.check("ap: this PC joins with the passphrase", pc_ip is not None, pc_ip)
        if not pc_ip:
            return

        _, _, pc_bssid = wlan_state()
        ok, api = sh.cmd("stat")
        st = {k: field(api, k) for k in ("status", "ip", "mask", "gw", "ssid", "ch", "bssid", "mac", "host")}
        rep.check("ap: status is WL_AP_CONNECTED with this PC joined", st["status"] == "8", st["status"])
        rep.check("ap: localIP, subnetMask, gatewayIP", st["ip"] == board_ip and st["mask"] == "255.255.255.0"
                  and st["gw"] == board_ip, "%s/%s gw %s" % (st["ip"], st["mask"], st["gw"]))
        rep.check("ap: SSID and channel", st["ssid"] == ssid and st["ch"] == "6", "%s ch %s" % (st["ssid"], st["ch"]))
        rep.check("ap: BSSID is the access point this PC joined", (st["bssid"] or "").upper() == pc_bssid,
                  "board %s, PC sees %s" % (st["bssid"], pc_bssid))
        rep.check("ap: macAddress is the access point's", st["mac"] == st["bssid"], st["mac"])
        rep.check("ap: default host name neuron6-xxxxxx", re.fullmatch(r"neuron6-[0-9a-f]{6}", st["host"] or "") is not None,
                  st["host"])

        off = server_answers(board_ip, pc_ip, st["host"])
        rep.check("ap: no host server before a sketch streams", not any(off.values()), off)
        ok, api = sh.cmd("wstart 1", 30)
        rep.check("ap: MayaW476_Start() starts the host server on the access point", ok, field(api, "rc"))

        client_checks(sh, rep, pc_ip, board_ip, "ap")
        discovery_checks(sh, rep, board_ip, pc_ip, st["host"], "ap")

        ok, api = sh.cmd("scan", 40)
        n = int(field(api, "n") or -1)
        nets = [field([a], "ssid") for a in api if a.startswith("net ")]
        visible = set(re.findall(r"^SSID \d+ : (.+)$", netsh("show", "networks"), re.M))
        rep.check("ap: scanNetworks while hosting", ok and n >= 1,
                  "%d networks; %d also seen by this PC" % (n, len(visible & set(nets))))
        if n >= 1:
            rep.check("ap: scan results carry RSSI, channel, encryption and BSSID",
                      all(re.search(r"rssi=-\d+ enc=\d+ ch=\d+ bssid=[0-9A-F:]{17}", a) for a in api if a.startswith("net ")),
                      re.sub(r'ssid="[^"]*"', "ssid=<hidden>", next((a for a in api if a.startswith("net ")), "")))
    finally:
        j.leave()

    ssid2, key2 = "N6API-%04d" % secrets.randbelow(10000), random_key()
    sh.secrets.append(key2)
    sh.cmd("disc")
    sh.cmd("cfg 192.168.23.1 192.168.23.1 192.168.23.1 255.255.255.0")
    sh.cmd("host n6bench-renamed")
    ok, api = sh.cmd("ap %s %s 1" % (ssid2, key2), 60)
    j2 = Join(ssid2, key2)
    try:
        pc_ip = j2.join("192.168.23.1") if ok else None
        rep.check("ap: config() moves the access point to 192.168.23.1", pc_ip is not None, pc_ip)
        if pc_ip:
            ok, api = sh.cmd("stat")
            rep.check("ap: ...and it hosts on channel 1", field(api, "ch") == "1", field(api, "ch"))
            a = [v for (_, t, v) in mdns_query("n6bench-renamed.local", pc_ip) if t == 1]
            rep.check("ap: setHostname() renames the board over mDNS", a == ["192.168.23.1"], a)
    finally:
        j2.leave()
    sh.cmd("cfg 0.0.0.0")
    sh.cmd("host -")

    ssid3, key3 = "N6API-%04d" % secrets.randbelow(10000), random_key()
    sh.secrets.append(key3)
    ok, _ = sh.cmd("ap %s %s 6" % (ssid3, key3), 60)
    j3 = Join(ssid3, key3)
    try:
        pc_ip = j3.join("192.168.10.1") if ok else None
        if pc_ip:
            ok, api = sh.cmd("disc")
            gone = wait_left(ssid3)
            rep.check("ap: disconnect() stops the access point", field(api, "status") == "6" and gone is not None,
                      "board status %s, this PC dropped after %s s" % (field(api, "status"), gone))
            ok, api = sh.cmd("ap %s %s 6" % (ssid3, key3), 60)
            pc_ip = j3.join("192.168.10.1") if ok else None
            rep.check("ap: beginAP again after disconnect()", pc_ip is not None, pc_ip)
            ok, api = sh.cmd("end", 20)
            gone = wait_left(ssid3)
            ok2, api2 = sh.cmd("stat")
            rep.check("ap: end() turns the radio off", gone is not None and field(api2, "ps") == "2",
                      "this PC dropped after %s s, board ps=%s status=%s" % (gone, field(api2, "ps"), field(api2, "status")))
            ok, api = sh.cmd("ap %s %s 6" % (ssid3, key3), 60)
            pc_ip = j3.join("192.168.10.1") if ok else None
            ok2, api2 = sh.cmd("stat")
            rep.check("ap: beginAP after end() wakes the radio", pc_ip is not None and field(api2, "ps") == "0",
                      "%s, ps=%s" % (pc_ip, field(api2, "ps")))
    finally:
        j3.leave()
    sh.cmd("disc")

    for sec, name in ((0, "open"), (1, "WEP"), (2, "WPA"), (4, "WPA/WPA2")):
        mark = sh.mark()
        ok, api = sh.cmd("ap N6REF-%04d %s 6 %d" % (secrets.randbelow(10000), random_key(), sec), 30)
        said = sh.wait_for(r"beginAP\(\): only WPA2 and WPA3", 3, since=mark)
        rep.check("ap: beginAP() refuses %s" % name, not ok and field(api, "status") == "9" and said is not None,
                  said)
    mark = sh.mark()
    ok, api = sh.cmd("ap N6REF-%04d short77 6" % secrets.randbelow(10000), 30)
    said = sh.wait_for(r"beginAP\(\): the passphrase must be 8 to 63", 3, since=mark)
    rep.check("ap: beginAP() refuses a passphrase under 8 characters", not ok and said is not None, said)

    for sec, name, auths in ((5, "WPA3", ("WPA3SAE",)), (6, "WPA2/WPA3", ("WPA2PSK", "WPA3SAE"))):
        ssid4, key4 = "N6WPA3-%04d" % secrets.randbelow(10000), random_key()
        sh.secrets.append(key4)
        ok, api = sh.cmd("ap %s %s 6 %d" % (ssid4, key4, sec), 60)
        rep.check("ap: beginAP() hosts %s" % name, ok, "; ".join(api[-2:]))
        for auth in auths:
            j4 = Join(ssid4, key4, auth)
            try:
                pc_ip = j4.join("192.168.10.1") if ok else None
                rep.check("ap: this PC joins the %s access point as %s" % (name, auth), pc_ip is not None, pc_ip)
            finally:
                j4.leave()
                wait_left(ssid4)
        sh.cmd("disc")


class GroupOwner:
    """This PC as a WPA2 access point: n6radio wfd-ap, for as long as it runs."""

    def __init__(self, ssid, key):
        self.ssid, self.key, self.p = ssid, key, None

    def start(self):
        exe = build_n6radio.build()
        self.p = subprocess.Popen([exe, "wfd-ap", "--ssid", self.ssid, "--key", self.key, "--seconds", "1800"],
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.time() + 20
        while time.time() < deadline:
            line = self.p.stdout.readline()
            if "WFD ready" in line:
                return True
            if not line and self.p.poll() is not None:
                return False
        return False

    def stop(self):
        if self.p and self.p.poll() is None:
            self.p.terminate()
            try:
                self.p.wait(5)
            except subprocess.TimeoutExpired:
                self.p.kill()
        self.p = None


def ping_ms(ip, count=10):
    out = subprocess.run(["ping", "-n", str(count), "-w", "2000", ip], capture_output=True, text=True).stdout
    times = [int(t) for t in re.findall(r"time[=<](\d+)ms", out)]
    return sorted(times)[len(times) // 2] if times else None


def phase_sta(sh, rep):
    ext_ssid, ext_key = os.environ.get("N6_STA_SSID"), os.environ.get("N6_STA_KEY")
    ssid, key = (ext_ssid, ext_key) if ext_ssid else ("N6STA-%04d" % secrets.randbelow(10000), random_key())
    sh.secrets.append(key)
    sh.cmd("disc")
    sh.cmd("cfg 0.0.0.0")

    for sec, name in ((0, "open"), (1, "WEP"), (2, "WPA"), (4, "WPA/WPA2")):
        mark = sh.mark()
        ok, api = sh.cmd("sta N6REF-%04d %s 1500 %d" % (secrets.randbelow(10000), random_key(), sec), 30)
        said = sh.wait_for(r"begin\(\): only WPA2 and WPA3", 3, since=mark)
        rep.check("sta: begin() refuses %s" % name, not ok and field(api, "status") == "4" and said is not None,
                  said)
    mark = sh.mark()
    ok, api = sh.cmd("sta N6REF-%04d - 1500" % secrets.randbelow(10000), 30)
    said = sh.wait_for(r"begin\(\): a passphrase is needed", 3, since=mark)
    rep.check("sta: begin(ssid) with no passphrase is refused", not ok and said is not None, said)

    t0 = time.time()
    ok, api = sh.cmd("sta N6NoSuchNet-%04d %s" % (secrets.randbelow(10000), random_key()), 40)
    status = field(api, "status")
    while status != "1" and time.time() - t0 < 80:   # the driver rescans for 16 to 25 s
        time.sleep(1.0)
        status = field(sh.cmd("stat", 10)[1], "status")
    rep.check("sta: WL_NO_SSID_AVAIL for a network nobody hosts", status == "1",
              "after %.0f s" % (time.time() - t0))
    sh.cmd("disc")

    sh.cmd("sta N6NoSuchNet-%04d %s 1500" % (secrets.randbelow(10000), random_key()), 30)
    sh.cmd("sta N6NoSuchNet-%04d %s 1500" % (secrets.randbelow(10000), random_key()), 30)
    ok1, api1 = sh.cmd("scan", 60)
    sh.cmd("disc", 30)
    ok2, api2 = sh.cmd("scan", 60)
    rep.check("sta: begin() again or disconnect() mid-join leaves scanning working", ok1 and ok2,
              "%s, %s" % (field(api1, "n"), field(api2, "n")))

    # Windows opens a group owner on the channel its WiFi already uses, so this
    # PC joins the board's own access point on channel 1 first.
    go = GroupOwner(ssid, key) if not ext_ssid else None
    steer = None
    if go is not None:
        steer_ssid, steer_key = "N6STEER-%04d" % secrets.randbelow(10000), random_key()
        sh.secrets.append(steer_key)
        sh.cmd("ap %s %s 1" % (steer_ssid, steer_key), 60)
        steer = Join(steer_ssid, steer_key)
        started = steer.join("192.168.10.1") is not None and go.start()
        if not rep.check("sta: this PC hosts a Wi-Fi Direct group owner on 2.4 GHz", started, ssid):
            steer.leave()
            go.stop()
            return
    def joined(timeout=60):
        """begin() returns after its timeout. The station keeper goes on trying."""
        ok, api = sh.cmd("sta %s %s" % (ssid, key), 60)
        t0 = time.time()
        while not ok and time.time() - t0 < timeout:
            time.sleep(2.0)
            _, api = sh.cmd("stat", 10)
            ok = field(api, "status") == "3"
        return ok, field(api, "ip"), time.time() - t0

    try:
        ok, board_ip, extra = joined()
        if steer is not None:
            steer.leave()
        rep.check("sta: begin() joins it and gets a DHCP lease", ok,
                  "%s%s" % (board_ip, ", on the station keeper's retry" if extra > 1 else ""))
        if not ok:
            return
        pc_ip = local_ip_towards(board_ip)
        ok, api = sh.cmd("stat")
        st = {k: field(api, k) for k in ("status", "ip", "mask", "gw", "dns", "rssi", "ch", "bssid", "mac", "host")}
        rep.check("sta: status WL_CONNECTED, gateway is this PC", st["status"] == "3" and st["gw"] == pc_ip,
                  "%s gw %s mask %s dns %s" % (st["ip"], st["gw"], st["mask"], st["dns"]))
        rep.check("sta: RSSI is a real reading", st["rssi"] is not None and -95 < int(st["rssi"]) < -5, st["rssi"])
        rep.check("sta: channel and BSSID", st["ch"] not in (None, "0") and st["bssid"] not in (None, "00:00:00:00:00:00")
                  and st["bssid"] != st["mac"], "ch %s bssid %s (board %s)" % (st["ch"], st["bssid"], st["mac"]))

        off = server_answers(board_ip, pc_ip, st["host"])
        rep.check("sta: no host server: TCP 5000, discovery and mDNS closed", not any(off.values()), off)
        for mode, name in ((2, "TRANSPORT_WIFI_STA"), (1, "TRANSPORT_WIFI_SOFTAP")):
            mark = sh.mark()
            ok, api = sh.cmd("wstart %d" % mode, 30)
            said = sh.wait_for(r"runs on (the board's|its) own access point only", 3, since=mark)
            rep.check("sta: MayaW476_Start(%s) is refused while joined" % name,
                      not ok and field(api, "rc") == "-3" and said is not None, said)
        off = server_answers(board_ip, pc_ip, st["host"])
        rep.check("sta: ...and the host server stays closed", not any(off.values()), off)

        client_checks(sh, rep, pc_ip, board_ip, "sta")

        base = ping_ms(board_ip, 20)
        ok, _ = sh.cmd("lp 1")
        time.sleep(3)
        _, api = sh.cmd("stat")
        slept = ping_ms(board_ip, 20)
        rep.check("sta: lowPowerMode() puts the station in IEEE power save",
                  ok and field(api, "ps") in ("1", "3") and slept is not None and base is not None and slept > base,
                  "ps=%s, median ping %s ms awake, %s ms in power save" % (field(api, "ps"), base, slept))
        ok, _ = sh.cmd("lp 0")
        time.sleep(2)
        _, api = sh.cmd("stat")
        rep.check("sta: noLowPowerMode() wakes it", ok and field(api, "ps") == "0", "ps=%s, ping %s ms"
                  % (field(api, "ps"), ping_ms(board_ip, 10)))

        prefix = board_ip.rsplit(".", 1)[0]
        fixed = prefix + ".77"
        sh.cmd("disc")
        sh.cmd("cfg %s %s %s 255.255.255.0" % (fixed, pc_ip, pc_ip))
        ok, ip, _ = joined()
        rep.check("sta: config() gives the station a fixed address", ok and ip == fixed
                  and ping_ms(fixed, 4) is not None, ip)
        sh.cmd("cfg 0.0.0.0")

        if go is not None:
            mark = sh.mark()
            go.stop()
            lost = sh.wait_for(r"\[api\] beat status=5", 40, since=mark)
            rep.check("sta: WL_CONNECTION_LOST when the network goes", lost is not None, lost or "status never 5")
            go = GroupOwner(ssid, key)
            go.start()
            mark = sh.mark()
            back = sh.wait_for(r"\[api\] beat status=3", 60, since=mark)
            if back is not None:
                rep.check("sta: rejoins by itself when it returns", True, back)
            else:
                _, api = sh.cmd("scan", 60)
                seen = any(field([a], "ssid") == ssid for a in api if a.startswith("net "))
                if seen:
                    rep.check("sta: rejoins by itself when it returns", False, "in range, not rejoined in 60 s")
                else:
                    rep.info("sta: rejoins by itself when it returns",
                             "not testable here: the restarted group owner came back off 2.4 GHz")
        sh.cmd("disc")
    finally:
        if go is not None:
            go.stop()


servers = Servers()


def phase_stored(sh, rep):
    import n6bench

    def restarted(send):
        """Closes the port, writes through `send`, and reopens it after the restart."""
        sh.s.close()
        ok = send()
        return sh.reopen() and ok

    def store(payload):
        return restarted(lambda: n6bench.store_record(n6bench.REC_WIFI, payload))

    def asset():
        ok, api = sh.cmd("asset", 20)
        return {k: field(api, k) for k in ("user", "text", "wifi", "fsbl")} if ok else {}

    tag = "%04d" % secrets.randbelow(10000)
    ok = store(b"")
    _, api = sh.cmd("stored", 20)
    rep.check("stored: removing the settings restarts the board", ok and field(api, "mode") == "0",
              "mode %s" % field(api, "mode"))
    ok, api = sh.cmd("sbegin", 60)
    rep.check("stored: begin() with nothing stored starts nothing", not ok and field(api, "status") == "1",
              "status %s (1 is WL_NO_SSID_AVAIL)" % field(api, "status"))

    ssid, key, host = "N6STORE-" + tag, random_key(), "n6-stored-" + tag
    sh.secrets.append(key)
    ok = store(n6bench.wifi_record(2, ssid, key.encode(), None, 6, host))
    _, api = sh.cmd("stored", 20)
    rep.check("stored: the board restarts with the new settings", ok and field(api, "mode") == "2" and
              field(api, "ssid") == ssid and field(api, "hostname") == host,
              "mode %s, %s, hostname %s" % (field(api, "mode"), field(api, "ssid"), field(api, "hostname")))
    rep.check("stored: the bootloader reports the write", asset().get("fsbl") == "written")

    ok, api = sh.cmd("sbegin", 90)
    rep.check("stored: begin() hosts the stored network", ok and field(api, "ip") == "192.168.10.1",
              "status %s, %s" % (field(api, "status"), field(api, "ssid")))
    sh.cmd("wstart 1", 30)
    j = Join(ssid, key)
    try:
        pc_ip = j.join("192.168.10.1") if ok else None
        rep.check("stored: this PC joins it with the stored passphrase", pc_ip is not None, pc_ip)
        if pc_ip:
            a = [v for (_, t, v) in mdns_query(host + ".local", pc_ip) if t == 1]
            rep.check("stored: the stored hostname answers over mDNS", a == ["192.168.10.1"], a)
            details = {}
            n6bench.discover(5000, "192.168.10.255", 2.0, details)
            got = details.get("192.168.10.1", (None, False))
            rep.check("stored: discovery names the board by the stored hostname", got[0] == host, got)
    finally:
        j.leave()
        wait_left(ssid)

    ok = shell_restart(sh, "awrite n6-user-asset-" + tag)
    a = asset()
    rep.check("stored: a user asset written beside the settings", ok and a.get("user") == "yes" and
              a.get("text") == "n6-user-asset-" + tag and a.get("wifi") == "yes", a)
    _, api = sh.cmd("stored", 20)
    rep.check("stored: ...and the settings are still there", field(api, "ssid") == ssid)

    nope = "N6NOPE-" + tag
    ok = store(n6bench.wifi_record(1, nope, random_key().encode()))
    a = asset()
    rep.check("stored: new settings keep the user asset", ok and a.get("user") == "yes" and
              a.get("text") == "n6-user-asset-" + tag and a.get("wifi") == "yes", a)
    t0 = time.time()
    ok, api = sh.cmd("sbegin", 90)
    status = field(api, "status")
    while not ok and status != "1" and time.time() - t0 < 80:   # the driver rescans for 16 to 25 s
        time.sleep(1.0)
        status = field(sh.cmd("stat", 10)[1], "status")
    rep.check("stored: joining a network that is not there ends in WL_NO_SSID_AVAIL", not ok and status == "1",
              "status %s after %.0f s" % (status, time.time() - t0))

    ok = store(n6bench.wifi_record(2, ssid, b"", None, 6))
    pw = board_password(sh)
    ok2, api = sh.cmd("sbegin", 90)
    j = Join(ssid, pw or "")
    try:
        pc_ip = j.join("192.168.10.1") if ok and ok2 and pw else None
        rep.check("stored: hosting with no stored key uses the board's own password", pc_ip is not None, pc_ip)
    finally:
        j.leave()
        wait_left(ssid)
    sh.cmd("disc")

    for mode, rec, why in ((2, n6bench.wifi_record(2, ssid, b"", 0, 6), r"only WPA2 and WPA3"),
                           (1, n6bench.wifi_record(1, nope, random_key().encode(), 4), r"only WPA2 and WPA3"),
                           (1, n6bench.wifi_record(1, nope, b""), r"a passphrase is needed")):
        ok = store(rec)
        mark = sh.mark()
        ok2, api = sh.cmd("sbegin", 60)
        said = sh.wait_for(r"stored on this board: " + why, 3, since=mark)
        rep.check("stored: %s refused" % ("hosting open" if mode == 2 else
                                          "joining WPA/WPA2" if why.startswith("only") else "joining with no key"),
                  ok and not ok2 and said is not None, said)

    ok = store(b"")
    a = asset()
    _, api = sh.cmd("stored", 20)
    rep.check("stored: removing the settings keeps the user asset", ok and a.get("user") == "yes" and
              a.get("wifi") == "no" and field(api, "mode") == "0", a)
    ok = shell_restart(sh, "aclear")
    a = asset()
    rep.check("stored: the user asset removed as well", ok and a.get("user") == "no" and a.get("wifi") == "no", a)


def phase_link(sh, rep):
    import n6bench
    board = "192.168.10.1"

    def keyed():
        d = {}
        n6bench.discover(5000, "192.168.10.255", 2.0, d)
        return d.get(board, (None, None))[1]

    def raw_challenge():
        """Connects without answering. Returns (socket, bytes received, connect time)."""
        s = socket.create_connection((board, 5000), timeout=5)
        t0 = time.time()
        s.settimeout(1.5)
        got = b""
        try:
            while len(got) < 39:
                chunk = s.recv(4096)
                if not chunk:
                    break
                got += chunk
        except socket.timeout:
            pass
        return s, got, t0

    def closed_within(s, t0, seconds):
        """Seconds from connecting until the board closed `s`. None if it
        stayed open or sent data."""
        s.settimeout(0.5)
        while time.time() - t0 < seconds:
            try:
                data = s.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return time.time() - t0
            if not data:
                return time.time() - t0
            return None
        return None

    def streams(link_key):
        """Connects as a host. Returns (challenged, attached, the board's console line)."""
        mark = sh.mark()
        try:
            t = n6bench.TcpLink(board, link_key=link_key)
        except (OSError, ConnectionError) as e:
            return False, False, str(e)
        said = sh.wait_for(r"Neuro host connected|a Neuro host was refused", 6, since=mark)
        t.close()
        sh.wait_for(r"listening again|was refused", 6, since=mark)   # the slot is free again
        return t.challenged, said is not None and "connected" in said, said

    tag = "%04d" % secrets.randbelow(10000)
    ssid, key, lk = "N6LINK-" + tag, random_key(), random_key(24)
    sh.secrets += [key, lk]
    sh.cmd("key -")
    ok, api = sh.cmd("ap %s %s" % (ssid, key), 60)
    sh.cmd("wstart 1", 30)
    j = Join(ssid, key)
    try:
        pc_ip = j.join(board) if ok else None
        if not rep.check("link: this PC joins the board's access point", pc_ip is not None, pc_ip):
            return
        rep.check("link: without a key, discovery says none is needed", keyed() is False)
        challenged, got, why = streams(b"")
        rep.check("link: without a key, a host is attached at once", not challenged and got, why)

        rep.check("link: setLinkKey() refuses a key under 8 characters", not sh.cmd("key short")[0])
        rep.check("link: setLinkKey() takes a key", sh.cmd("key " + lk)[0])
        rep.check("link: discovery says a key is needed", keyed() is True)

        s, got, t0 = raw_challenge()
        ch = n6bench.read_challenge(got)
        rep.check("link: a host is challenged first, with nothing else", ch is not None and len(got) == 39,
                  "%d bytes" % len(got))
        took = closed_within(s, t0, 14)
        s.close()
        rep.check("link: ...and dropped when it does not answer", took is not None and 9 <= took <= 12,
                  "closed after %s s" % ("%.1f" % took if took is not None else "never"))

        mark = sh.mark()
        s, got, t0 = raw_challenge()
        ch = n6bench.read_challenge(got)
        s.sendall(n6bench.frame_cmd(n6bench.link_answer(b"not the key at all", ch or b"")))
        took = closed_within(s, t0, 5)
        s.close()
        rep.check("link: a wrong key is refused", ch is not None and took is not None,
                  "closed after %.1f s" % took if took is not None else "stayed open")

        s, got, t0 = raw_challenge()
        s.sendall(n6bench.frame_cmd(n6bench.sub0(1)))
        took = closed_within(s, t0, 5)
        s.close()
        rep.check("link: a command instead of the answer is refused", took is not None,
                  "closed after %.1f s" % took if took is not None else "stayed open")
        rep.check("link: the board says why", sh.wait_for(r"refused: wrong link key", 3, since=mark) is not None)

        challenged, got, why = streams(lk.encode())
        rep.check("link: the right key is attached", challenged and got, why)

        sh.cmd("key -")
        challenged, got, why = streams(b"")
        rep.check("link: setLinkKey(\"\") turns it off", keyed() is False and not challenged and got, why)
    finally:
        j.leave()
        wait_left(ssid)

    lk2 = random_key(32)
    sh.secrets.append(lk2)
    sh.s.close()
    ok = n6bench.store_record(n6bench.REC_WIFI, n6bench.wifi_record(2, ssid, key.encode(), None, 6, "",
                                                                    lk2.encode()))
    ok = sh.reopen() and ok
    ok2, api = sh.cmd("sbegin", 90)
    sh.cmd("wstart 1", 30)
    j = Join(ssid, key)
    try:
        pc_ip = j.join(board) if ok and ok2 else None
        rep.check("link: stored settings with a link key come back up", pc_ip is not None, pc_ip)
        if pc_ip:
            rep.check("link: the stored key is asked for", keyed() is True)
            challenged, got, why = streams(lk.encode())
            rep.check("link: the old key is refused", challenged and not got, why)
            challenged, got, why = streams(lk2.encode())
            rep.check("link: the stored key is attached", challenged and got, why)
    finally:
        j.leave()
        wait_left(ssid)
        sh.s.close()
        n6bench.store_record(n6bench.REC_WIFI, b"")
        sh.reopen()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="default: the one Neuro N6 running a sketch")
    ap.add_argument("--phases", default="ap,sta")
    ap.add_argument("--json")
    ap.add_argument("--log", default=os.path.join(tempfile.gettempdir(), "n6wifiapi.log"))
    args = ap.parse_args()

    servers.start()
    sh = Shell(args.port or find_port(), args.log)
    rep = Report()
    try:
        ok, api = sh.cmd("stat", 20)
        if not rep.check("the board answers the shell", ok, api[:1]):
            return rep.finish(args.json)
        for ph in args.phases.split(","):
            {"pass": phase_pass, "ap": phase_ap, "sta": phase_sta, "stored": phase_stored,
             "link": phase_link}[ph.strip()](sh, rep)
    finally:
        sh.close()
        print("board console: %s" % args.log)
    return rep.finish(args.json)


if __name__ == "__main__":
    sys.exit(main())
