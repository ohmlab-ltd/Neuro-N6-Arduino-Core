#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Fail if memcpy, memset or memmove in a linked image is a byte loop.

cores/NeuroN6/n6_string.c defines them. One that calls itself also fails.

    python tools/check_string_codegen.py build/Sketch.ino.elf
    python tools/check_string_codegen.py --objdump /path/to/objdump image.elf
"""
import argparse
import os
import re
import subprocess
import sys

ROUTINES = ("memcpy", "memset", "memmove")

# memmove hands the ascending case to memcpy.
ALLOWED_CALLS = {"memmove": {"memcpy"}}

LABEL_RE = re.compile(r"^[0-9a-f]+ <([A-Za-z_][A-Za-z0-9_.$]*)>:")
INSN_RE = re.compile(r"^\s*[0-9a-f]+:\s+(?:[0-9a-f]{4}\s)+\s*([a-z][a-z0-9.]*)(?:\s+(.*))?$")
CALL_RE = re.compile(r"<([A-Za-z_][A-Za-z0-9_.$]*)>$")

BYTE_ACCESS = ("ldrb", "strb", "ldrsb", "strsb")


def find_objdump(explicit):
    if explicit:
        return explicit
    env = os.environ.get("OBJDUMP")
    if env:
        return env
    for name in ("arm-none-eabi-objdump", "arm-none-eabi-objdump.exe"):
        for d in os.environ.get("PATH", "").split(os.pathsep):
            p = os.path.join(d, name)
            if os.path.isfile(p):
                return p
    return None


def disassemble(objdump, elf):
    out = subprocess.run([objdump, "-d", elf], stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE)
    if out.returncode != 0:
        sys.stderr.write(out.stderr.decode("utf-8", "replace"))
        sys.exit("objdump failed on " + elf)
    return out.stdout.decode("utf-8", "replace").splitlines()


def split_functions(lines):
    """Map each function name to its instruction lines."""
    bodies = {}
    cur = None
    for line in lines:
        m = LABEL_RE.match(line)
        if m is not None:
            cur = m.group(1)
            bodies[cur] = []
        elif cur is not None:
            bodies[cur].append(line)
    return bodies


def check(name, body):
    problems = []
    wide = 0
    for line in body:
        m = INSN_RE.match(line)
        if m is None:
            continue
        mnemonic, operands = m.group(1), (m.group(2) or "")
        base = mnemonic.split(".")[0]

        if ("ldr" in base or "str" in base or base in ("ldm", "stm", "ldc", "stc")
                or base.startswith("vld") or base.startswith("vst")):
            if base not in BYTE_ACCESS:
                wide += 1

        # A bare <name> is a branch or call. An internal branch is <name+0xNN>.
        call = CALL_RE.search(operands.strip())
        if call is not None:
            target = call.group(1)
            if target == name:
                problems.append("calls itself: " + line.strip())
            elif (target in ROUTINES
                  and target not in ALLOWED_CALLS.get(name, set())):
                problems.append("calls " + target + ": " + line.strip())

    if wide == 0:
        problems.append("no access wider than a byte, so this is a byte loop")
    return problems


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("elf")
    ap.add_argument("--objdump", default=None)
    args = ap.parse_args()

    objdump = find_objdump(args.objdump)
    if objdump is None:
        sys.exit("no objdump: pass --objdump or set OBJDUMP")
    if not os.path.isfile(args.elf):
        sys.exit("no such file: " + args.elf)

    bodies = split_functions(disassemble(objdump, args.elf))

    failures = 0
    for name in ROUTINES:
        body = bodies.get(name)
        if body is None:
            print("MISSING %-8s not in the image" % name)
            continue
        problems = check(name, body)
        if problems:
            failures += 1
            for p in problems:
                print("FAIL    %-8s %s" % (name, p))
        else:
            print("ok      %-8s %d instructions" % (name, len(body)))

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
