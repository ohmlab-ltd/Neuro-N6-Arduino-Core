#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Fail if hand-written source contains non-ASCII, a BOM, CRLF or a NUL.

    python tools/check_ascii.py            # check, exit 1 on any finding
    python tools/check_ascii.py --list     # list findings only
"""
import argparse
import os
import sys
import unicodedata

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

EXTS = (".c", ".h", ".cpp", ".hpp", ".ino", ".py", ".S", ".s", ".ld", ".txt", ".md",
        ".properties")

# Third-party trees, as listed in THIRD-PARTY-NOTICES.md.
VENDORED = (
    "cores/NeuroN6/STM32N6xx_HAL_Driver/", "cores/NeuroN6/CMSIS/", "cores/NeuroN6/class/",
    "cores/NeuroN6/device/", "cores/NeuroN6/portable/", "cores/NeuroN6/common/",
    "cores/NeuroN6/host/", "cores/NeuroN6/typec/", "cores/NeuroN6/FreeRTOS/",
    "cores/NeuroN6/AI_Runtime/", "cores/NeuroN6/ST/", "cores/NeuroN6/api/",
    "cores/NeuroN6/osal/", "cores/NeuroN6/Third_Party/", "cores/NeuroN6/lib_vision_models_pp/",
    "libraries/ArduinoBLE/", "libraries/Quirc/", "libraries/N6lwIP/",
    "libraries/EthernetN6/src/stm32n6xx_hal_eth",
    "libraries/MayaW476/src/wifi_nxp/", "libraries/MayaW476/src/osa/",
    "libraries/VD66GY/extras/", "libraries/LSM6DSL/extras/",
    "libraries/PostProcess/src/Fonts/",
    "libraries/Microphone/src/stm32n6xx_hal_mdf", "libraries/Speaker/src/stm32n6xx_hal_sai",
    "libraries/OV5640/src/ov5640_reg.c", "libraries/OV5640/src/ov5640_reg.h",
    "libraries/OV5640/src/ov5640_af_firmware.h", "libraries/VD66GY/reference/",
    "cores/NeuroN6/partition_stm32n657xx.h", "cores/NeuroN6/Print.h", "cores/NeuroN6/WString.h",
    "cores/NeuroN6/Stream.h", "cores/NeuroN6/Printable.h", "cores/NeuroN6/RingBuffer.h",
    "cores/NeuroN6/WCharacter.h",
    "tools/pyserial/",
)

OHM_LAB_IN_VENDORED = (
    "cores/NeuroN6/api/Common.h", "cores/NeuroN6/api/itoa.c",
    "libraries/ArduinoBLE/src/utility/HCIMayaW476Transport.cpp",
)


def vendored(rel):
    if any(rel.startswith(o) for o in OHM_LAB_IN_VENDORED):
        return False
    return any(rel.startswith(v) for v in VENDORED)


def walk():
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames
                       if not d.startswith(".")
                       and d not in ("__pycache__", "build", "dist", "node_modules")]
        for fn in filenames:
            if not fn.endswith(EXTS):
                continue
            full = os.path.join(dirpath, fn)
            rel = os.path.relpath(full, ROOT).replace("\\", "/")
            if vendored(rel):
                continue
            yield rel, full


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    findings = []
    for rel, full in walk():
        raw = open(full, "rb").read()
        if raw.startswith(b"\xef\xbb\xbf"):
            findings.append((rel, 0, "UTF-8 BOM"))
        if b"\r\n" in raw:
            findings.append((rel, 0, "CRLF line endings"))
        if b"\x00" in raw:
            findings.append((rel, 0, "embedded NUL byte"))
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            findings.append((rel, 0, "not valid UTF-8: %s" % exc))
            continue
        for n, line in enumerate(text.split("\n"), 1):
            for ch in line:
                if ord(ch) > 126:
                    try:
                        name = unicodedata.name(ch)
                    except ValueError:
                        name = "U+%04X" % ord(ch)
                    findings.append((rel, n, "non-ASCII %s (U+%04X)" % (name, ord(ch))))
                    break

    if not findings:
        print("check_ascii: clean")
        return 0
    byfile = {}
    for rel, n, what in findings:
        byfile.setdefault(rel, []).append((n, what))
    print("check_ascii: %d findings in %d files" % (len(findings), len(byfile)))
    for rel in sorted(byfile):
        for n, what in byfile[rel][:5]:
            print("  %s:%d: %s" % (rel, n, what))
        if len(byfile[rel]) > 5:
            print("  %s: ... and %d more" % (rel, len(byfile[rel]) - 5))
    return 0 if args.list else 1


if __name__ == "__main__":
    sys.exit(main())
