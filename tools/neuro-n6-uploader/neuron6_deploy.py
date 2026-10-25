#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Upload tool for the Arduino IDE: writes model weights and the application.

    neuron6_deploy.py --port COMx --bin <app_Trusted.bin> --address 0x90100000
                      --manifest <build>/neuron6_models.json
                      --model-upload changed|always

    changed  skips weights the board already holds
    always   rewrites them
"""

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
try:
    import serial
except ImportError:
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), "pyserial"))
    import serial

from utils import dfu, fsbl
from utils.dfu import DfuError, DfuSession
from utils.dfu import human, raw, kv as kvfmt

WIDTH = 64


def log(msg=""):
    print(msg, flush=True)


def kv(key, value):
    log(kvfmt(key, value))


def header(title):
    log("=" * WIDTH)
    log("  NEURO N6 %s" % title.upper())
    log("=" * WIDTH)


def rule():
    log("=" * WIDTH)


def group(title):
    log()
    log("  %s" % title)


def die(msg):
    log()
    log("=" * WIDTH)
    log("  UPLOAD FAILED")
    log("=" * WIDTH)
    for l in str(msg).splitlines():
        log("  " + l)
    log("=" * WIDTH)
    sys.exit(1)


# Host-side weights cache, keyed by board serial and flash address

def cache_path():
    if sys.platform.startswith("win"):
        base = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~")
        d = os.path.join(base, "NeuroN6")
    else:
        d = os.path.join(os.path.expanduser("~"), ".neuron6")
    try:
        os.makedirs(d, exist_ok=True)
    except OSError:
        pass
    return os.path.join(d, "board_weights_cache.json")


def cache_load():
    try:
        with open(cache_path(), "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def cache_save(cache):
    try:
        with open(cache_path(), "w", encoding="utf-8") as f:
            json.dump(cache, f, indent=1)
    except OSError:
        pass


def cache_get(cache, serial_key, address):
    return cache.get(serial_key or "unknown", {}).get(address)


def cache_put(cache, serial_key, address, sha256, crc32):
    cache.setdefault(serial_key or "unknown", {})[address] = {
        "sha256": sha256, "crc32": crc32,
        "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
    }


def load_manifest(path):
    if not path or not os.path.isfile(path):
        return []
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        models = data.get("models", [])
    except (OSError, ValueError) as e:
        die("Model manifest is unreadable (%s):\n  %s\n"
            "Rebuild the sketch (Verify) and upload again." % (e, path))
    for m in models:
        if not os.path.isfile(m["weights_bin"]):
            die("Cached weights for model '%s' are missing:\n  %s\n"
                "The model cache was cleaned since the last build.\n"
                "Rebuild the sketch (Verify) and upload again."
                % (m["name"], m["weights_bin"]))
    return models


def reenter_bootloader(log_fn):
    """v1: gets back into the bootloader after the DFU_END reset."""
    time.sleep(1.5)  # lets the reset and re-enumeration settle
    app = dfu.wait_for_app(timeout_sec=8)
    port = dfu.enter_bootloader(app, log=log_fn)
    if port is None:
        die("Could not re-enter the bootloader for the next image.\n"
            "Double-tap the reset button and upload again.")
    return port


def program_with_verify(ses, data, offset, label):
    """Programs one image in a MULTI session, verifies it by device CRC32 and
    retries once."""
    want = dfu.host_crc32(data)
    attempts = 2
    for attempt in range(1, attempts + 1):
        ses.program(data, offset, label, next_image=True)
        raw(kvfmt("verify", "device CRC32 ... "))
        got = ses.crc32(offset, len(data))
        if got is None:
            log("not supported by this bootloader; skipped")
            return
        if got == want:
            log("0x%08X ok" % got)
            return
        log("MISMATCH (device 0x%08X, host 0x%08X)" % (got, want))
        if attempt < attempts:
            kv("retry", "rewriting %s" % label)
    die("%s failed flash verification after retry.\n"
        "The flash contents do NOT match the uploaded file - do not trust\n"
        "this deployment. Check the USB connection and try again." % label)


def main():
    ap = argparse.ArgumentParser(description="Neuro N6 weights+app deploy")
    ap.add_argument("--port", default="")
    ap.add_argument("--bin", required=True, help="application _Trusted.bin")
    ap.add_argument("--address", default="0x90100000")
    ap.add_argument("--manifest", default="")
    ap.add_argument("--model-upload", default="changed",
                    choices=None)  # validated below: the IDE can pass a raw token
    args = ap.parse_args()

    mode = args.model_upload.strip().lower()
    if mode not in ("changed", "always"):
        mode = "changed"  # an unexpanded {build.model_upload_mode}

    if not os.path.isfile(args.bin):
        die("Application binary not found:\n  %s\n"
            "Build the sketch first (Verify), then upload." % args.bin)
    try:
        app_offset = dfu.addr_to_offset(int(args.address, 16))
    except (ValueError, DfuError) as e:
        die("Bad application address %r: %s" % (args.address, e))

    models = load_manifest(args.manifest)
    with open(args.bin, "rb") as f:
        app_data = f.read()

    header("deploy")
    group("PLAN")
    for m in models:
        kv("weights", "%-28s %9s   0x%08X" % (m["name"][:28], human(m["size"]),
                                              int(m["address"], 16)))
    kv("application", "%-28s %9s   0x%08X" % (os.path.basename(args.bin)[:28],
                                              human(len(app_data)),
                                              dfu.FLASH_BASE + app_offset))
    if models:
        kv("policy", "upload weights %s" %
           ("only when changed" if mode == "changed"
            else "every time (Tools > Model weights)"))

    group("CONNECT")
    port = dfu.enter_bootloader(args.port or None, log=log)
    if port is None:
        die("No Neuro N6 bootloader found.\n"
            "Plug the board in and double-tap the reset button, then upload again.")

    ses = DfuSession(port, log=log)
    try:
        ses.open()
        info = ses.probe_info()
        v2 = bool(info)
        if len(app_data) > 0x100000 and not (info and "LOAD1536K" in info["caps"]):
            die("This application is %d bytes, but the board's bootloader loads at most 1 MB "
                "into RAM (FSBL v2.3.1 or older) and would truncate it. Update the bootloader "
                "to %s or newer, or shrink the sketch."
                % (len(app_data), fsbl.text(fsbl.MINIMUM)))
        if v2:
            caps = ", ".join(sorted(info["caps"])) or "-"
            kv("bootloader", "%s  |  FSBL v%d (%s)" % (port, info["version"], caps))
        else:
            kv("bootloader", "%s  |  legacy FSBL v1 (no CRC verify, "
               "one image per entry)" % port)
        release = fsbl.identify(ses) if v2 else None
        if release is not None:
            kv("", "release %s" % fsbl.text(release))
        # app_offset 0: this upload is the bootloader itself.
        if app_offset != 0 and (not v2 or (release is not None and release < fsbl.MINIMUM)):
            kv("", "older than %s, which this core needs: see" % fsbl.text(fsbl.MINIMUM))
            kv("", "Upload in the README for how to update it")
        serial_key = (info.get("serial") if info else None) or \
            dfu.bootloader_serial(port) or "unknown"

        cache = cache_load()

        uploads = []   # (kind, name, data, offset, sha, crc)
        for m in models:
            addr = int(m["address"], 16)
            offset = dfu.addr_to_offset(addr)
            with open(m["weights_bin"], "rb") as f:
                data = f.read()
            want_crc = int(m["crc32"], 16)
            skip = False
            why = ""
            if mode == "changed":
                if v2 and ses.has_cap("CRC32"):
                    got = ses.crc32(offset, len(data))
                    if got == want_crc:
                        skip, why = True, "flash CRC matches (device-verified)"
                    elif got is None:
                        why = "device CRC query failed; uploading"
                else:
                    entry = cache_get(cache, serial_key, m["address"])
                    if entry and entry.get("sha256") == m["sha256"]:
                        skip, why = True, "unchanged since last upload (host record)"
            if skip:
                group("WEIGHTS %s" % m["name"])
                kv("status", "skipped - %s" % why)
            else:
                if why:
                    group("WEIGHTS %s" % m["name"])
                    kv("status", why)
                uploads.append(("weights", m["name"], data, offset,
                                m["sha256"], m["crc32"]))

        total_images = len(uploads) + 1
        img_no = 0
        for kind, name, data, offset, sha, crc in uploads:
            img_no += 1
            group("WEIGHTS %s  (%d/%d)" % (name, img_no, total_images - 1))
            if v2 and ses.has_cap("MULTI"):
                program_with_verify(ses, data, offset, "weights '%s'" % name)
            else:
                # v1: each image ends with a device reset.
                ses.program(data, offset, "weights '%s'" % name, next_image=False)
                ses.close()
                port = reenter_bootloader(log)
                ses = DfuSession(port, log=log).open()
                if v2:
                    ses.probe_info()
            cache_put(cache, serial_key, "0x%08X" % (dfu.FLASH_BASE + offset),
                      sha, crc)
            cache_save(cache)

        group("APPLICATION %s" % os.path.basename(args.bin))
        if v2 and ses.has_cap("MULTI"):
            program_with_verify(ses, app_data, app_offset, "application")
            app_verified = True
            ses.reset_device()
        else:
            ses.program(app_data, app_offset, "application", next_image=False)
            app_verified = False
    except DfuError as e:
        die(e)
    except serial.SerialException as e:
        die("Serial link lost during upload: %s\n"
            "Check the USB cable and try again." % e)
    finally:
        ses.close()

    group("SUMMARY")
    if models:
        kv("weights", "%d uploaded, %d skipped (already on board)"
           % (len(uploads), len(models) - len(uploads)))
    kv("application", "uploaded%s" % (" and verified" if app_verified else ""))
    app = dfu.wait_for_app(timeout_sec=8, strict=True)
    kv("board", "running on %s" % app if app
       else "rebooting into the sketch")
    log()
    rule()
    sys.stdout.flush()
    time.sleep(0.4)


if __name__ == "__main__":
    main()
