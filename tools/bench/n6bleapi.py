#!/usr/bin/env python3
"""Checks BLE on a board running extras/bench/BleBench. Windows only.

    python tools/bench/n6bleapi.py [--port PORT] [--json out.json]
"""

# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT

import argparse
import os
import re
import secrets
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build_n6radio  # noqa: E402
import n6wifiapi as w  # noqa: E402

CTRL = "19b10001-e8f2-537e-4f6c-d104768a1214"
COUNT = "19b10002-e8f2-537e-4f6c-d104768a1214"
SECRET = "19b10003-e8f2-537e-4f6c-d104768a1214"
PC_SERVICE = "6e360001-b5a3-f393-e0a9-e50e24dcca9e"
# Windows' own service, advertised connectably once a peer is bonded.
WINDOWS_BONDED_SERVICE = "3e1d50cd-7e3e-427d-8e1c-b78aa87fe624"

RADIO = None


def radio(*args, timeout=60):
    r = subprocess.run([RADIO] + list(args), capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=timeout)
    return r.returncode, r.stdout.splitlines()


def radio_bg(*args):
    return subprocess.Popen([RADIO] + list(args), stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")


def find(lines, word):
    return [l for l in lines if l.startswith(word)]


def main():
    global RADIO
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="default: the one Neuro N6 running a sketch")
    ap.add_argument("--json")
    ap.add_argument("--log", default=os.path.join(tempfile.gettempdir(), "n6bleapi.log"))
    args = ap.parse_args()
    RADIO = build_n6radio.build()

    sh = w.Shell(args.port or w.find_port(), args.log, prefix="[ble]")
    rep = w.Report()
    name = "N6BLE-%04d" % secrets.randbelow(10000)
    addr = None
    try:
        ok, api = sh.cmd("stat", 20)
        if not rep.check("the board answers the shell", ok, api[:1]):
            return rep.finish(args.json)

        mark = sh.mark()
        ok, api = sh.cmd("adv %s" % name, 60)
        addr = (w.field(api, "address") or "").upper()
        rep.check("BLE.begin() and advertising", ok and addr != "", "address %s" % addr)
        boot = "\n".join(sh.lines[mark:])
        _, api = sh.cmd("stat")
        rep.check("BLE starts with no WiFi network", "SoftAP up" not in boot and "STA connecting" not in boot
                  and "wlan_init OK" in boot and w.field(api, "wifi") == "0",
                  "firmware loaded, WiFi status %s" % w.field(api, "wifi"))

        rc, out = radio("scan", "--name", name, "--first", "--seconds", "15")
        seen = [l for l in out if l.startswith("ADV")]
        rep.check("this PC hears the board advertising", rc == 0 and any(addr in l for l in seen), seen[:1])

        mark = sh.mark()
        rc, out = radio("gatt", addr, "--list", "--read", CTRL, "--write", CTRL + "=5a", "--read", COUNT, timeout=90)
        wrote = sh.wait_for(r"\[ble\] ctrl written 0x5A", 5, since=mark)
        rep.check("peripheral: connect, discover, read and write", rc == 0 and wrote is not None,
                  "; ".join(find(out, "CONNECTED") + find(out, "WRITE")))

        mark = sh.mark()
        sh.s.write(b"busy 25\n")
        time.sleep(1.0)
        t0 = time.time()
        rc, out = radio("gatt", addr, "--read", CTRL, "--write", CTRL + "=a5", "--read", COUNT, timeout=90)
        took = time.time() - t0
        wrote = sh.wait_for(r"\[ble\] ctrl written 0xA5", 5, since=mark)
        done = sh.wait_for(r"\[ble\] ok busy", 40, since=mark)
        written_first = wrote is not None and done is not None and \
            sh.lines.index(wrote, mark) < sh.lines.index(done, mark)
        rep.check("serviced in the background: answers while loop() is blocked", rc == 0 and written_first,
                  "read+write took %.1f s during a 25 s delay()" % took)

        sh.cmd("notify 1")
        rc, out = radio("gatt", addr, "--notify", COUNT, "--seconds", "6", timeout=60)
        n = int((find(out, "NOTIFIED") or ["count=0"])[0].split("count=")[1])
        sh.cmd("notify 0")
        rep.check("peripheral: notifications to a subscriber", rc == 0 and n >= 8, "%d in 6 s" % n)

        rc, out = radio("gatt", addr, "--read", SECRET, timeout=60)
        refused = [l for l in out if l.startswith("READ")]
        rep.check("pairing: an encrypted value refuses a stranger", rc != 0, refused)
        mark = sh.mark()
        rc, out = radio("pair", addr, timeout=90)
        paired = rc == 0
        bonded = sh.wait_for(r"\[ble\] bond ltk stored", 10, since=mark)
        rep.check("pairing: this PC pairs (numeric comparison, confirmed on both sides)", paired and bonded is not None,
                  "; ".join(out[-2:]))
        rc, out = radio("gatt", addr, "--read", SECRET, timeout=60)
        rep.check("pairing: ...and then reads it", rc == 0, [l for l in out if l.startswith("READ")])

        # Connecting to Windows' bonded service reaches the PC's whole GATT
        # database, the bench server's service included.
        sh.cmd("stopadv")
        sh.cmd("disc", 20)
        srv = radio_bg("server", "--seconds", "50")
        time.sleep(3)
        ok, api = sh.cmd("central %s" % WINDOWS_BONDED_SERVICE, 60)
        srv.terminate()
        served = srv.communicate(timeout=10)[0]
        if "fail central not found" in api:
            # ArduinoBLE scans with the legacy commands only.
            rep.info("central: through the bond, connect to this PC",
                     "not testable here: the board's scan does not hear the service Windows advertises for a bonded peer")
        else:
            rep.check("central: through the bond, connects to this PC, reads and writes its value",
                      ok and 'read "n6-pc-ok"' in " ".join(api) and 'text="board-was-here"' in served,
                      "; ".join(a for a in api if a.startswith(("central", "fail central"))))
        radio("unpair", addr, timeout=60)

        sh.cmd("stopadv")
        adv = radio_bg("advertise", "--seconds", "20", "--mfg", "4e3642454e4348")
        time.sleep(2)
        ok, api = sh.cmd("scan 10", 40)
        heard = [a for a in api if "mfg=ffff4e3642454e4348" in a]
        adv.wait(30)
        rep.check("central: the board scans and hears this PC", ok and heard != [],
                  heard[:1] or "%s devices, none of them this PC" % w.field(api, "heard"))

        srv = radio_bg("server", "--seconds", "45")
        time.sleep(3)
        ok, api = sh.cmd("central %s" % PC_SERVICE, 60)
        srv.terminate()
        served = srv.communicate(timeout=10)[0]
        if "fail central not found" in api:
            rep.info("central: connect to this PC's GATT server",
                     "not testable here: this PC advertises it extended-only; needs a legacy peripheral")
        else:
            rep.check("central: connects to this PC, reads and writes its value",
                      ok and 'read "n6-pc-ok"' in " ".join(api) and 'text="board-was-here"' in served,
                      "; ".join(a for a in api if a.startswith("central")))

        ok, api = sh.cmd("scan 5", 30)
        rep.check("central: a plain scan after a filtered one hears everything", ok and int(w.field(api, "heard") or 0) > 0,
                  "%s reports" % w.field(api, "heard"))

        ssid, key = "N6BLEAP-%04d" % secrets.randbelow(10000), w.random_key()
        sh.secrets.append(key)
        ok, api = sh.cmd("wifiap %s %s" % (ssid, key), 60)
        j = w.Join(ssid, key)
        try:
            pc_ip = j.join("192.168.10.1") if ok else None
            rep.check("WiFi starts after a BLE-only start", pc_ip is not None, pc_ip)
            sh.cmd("adv %s" % name)
            rc, out = radio("gatt", addr, "--read", CTRL, timeout=60)
            rep.check("...and BLE still answers with WiFi up", rc == 0, find(out, "READ"))
        finally:
            j.leave()
        ok, api = sh.cmd("wifiend", 30)
        rc, out = radio("gatt", addr, "--read", CTRL, timeout=60)
        rep.check("WiFi.end() leaves BLE answering", ok and rc == 0,
                  "; ".join(api[:1] + find(out, "READ")))
        ok, _ = sh.cmd("end")
        rc, out = radio("scan", "--name", name, "--first", "--seconds", "8")
        heard = any(addr in l for l in out if l.startswith("ADV"))
        rep.check("BLE.end() stops advertising", ok and not heard,
                  "this PC %s the board" % ("still hears" if heard else "no longer hears"))

        mark = sh.mark()
        ok, api = sh.cmd("adv %s" % name, 60)
        rc, out = radio("scan", "--name", name, "--first", "--seconds", "15")
        heard = any(addr in l for l in out if l.startswith("ADV"))
        lost = [l for l in sh.lines[mark:] if "HCI bytes lost" in l]
        rep.check("BLE.begin() again after BLE.end()", ok and heard and not lost,
                  lost[:1] or "advertising again, heard by this PC")
        ok, _ = sh.cmd("end")
        rep.check("BLE.end()", ok)
    finally:
        if addr:
            radio("unpair", addr, timeout=60)
        sh.close()
        print("board console: %s" % args.log)
    return rep.finish(args.json)


if __name__ == "__main__":
    sys.exit(main())
