#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Preflight check: the compiler exists and is GCC 11 or later.

Usage: check_toolchain.py <compiler-command-or-path>

Exits 1 when the compiler is missing, too old, or reports a version that
cannot be parsed.
"""
import os
import shutil
import subprocess
import sys

MIN_MAJOR = 11          # the linker script uses (READONLY)
TESTED = "13.3.rel1"


def fail(msg, fix=None):
    # Non-verbose builds show a hook's stderr, not its stdout.
    err = sys.stderr
    print("+------------------------------------------------------------+", file=err)
    print("| Neuro N6 Toolchain                                        |", file=err)
    print("| FAILED                                                     |", file=err)
    print("+------------------------------------------------------------+", file=err)
    print("  Problem   : %s" % msg, file=err)
    print(file=err)
    if fix:
        print("  Fix       : %s" % fix, file=err)
    else:
        print("  Fix       : install Arm GNU Toolchain %s or any GCC >= %d" % (TESTED, MIN_MAJOR), file=err)
        print("              and make sure arm-none-eabi-gcc is on PATH.", file=err)
        print("  Download  : https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads", file=err)
        print("  Also OK   : STM32CubeIDE 1.13+ bundled toolchain.", file=err)
    print("+------------------------------------------------------------+", file=err)
    sys.exit(1)


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "arm-none-eabi-gcc"
    if "{runtime.tools" in cmd:
        fail("The compiler tool is not installed. Install the core through Boards "
             "Manager, or on a manual clone add  compiler.path=  to platform.local.txt "
             "to use the arm-none-eabi-gcc on PATH.")
    resolved = shutil.which(cmd)
    if resolved is None:
        if os.path.dirname(cmd):
            fail("'%s' does not exist." % cmd, "reinstall the core in Boards Manager.")
        fail("'%s' was not found on PATH." % cmd)

    try:
        out = subprocess.run(
            [resolved, "-dumpversion"], capture_output=True, text=True, timeout=30
        )
    except Exception as exc:
        fail("could not run '%s -dumpversion': %s" % (resolved, exc))

    version = out.stdout.strip()
    if out.returncode != 0 or not version:
        fail("'%s -dumpversion' returned no version (rc=%d)." % (resolved, out.returncode))

    try:
        major = int(version.split(".")[0])
    except ValueError:
        fail("could not parse GCC version %r from %s." % (version, resolved))

    if major < MIN_MAJOR:
        fail(
            "found arm-none-eabi-gcc %s at %s, but this core requires GCC >= %d\n"
            "(the linker script uses the (READONLY) section keyword; %s is the\n"
            "tested release)." % (version, resolved, MIN_MAJOR, TESTED)
        )

    print("+------------------------------------------------------------+")
    print("| Neuro N6 Toolchain                                        |")
    print("+------------------------------------------------------------+")
    print("  Compiler  : arm-none-eabi-gcc %s" % version)
    print("  Path      : %s" % resolved)
    print("  Status    : OK")
    print("+------------------------------------------------------------+")
    sys.exit(0)


if __name__ == "__main__":
    main()
