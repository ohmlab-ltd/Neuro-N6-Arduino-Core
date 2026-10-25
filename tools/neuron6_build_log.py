#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Build-stage log lines for the Neuro N6 core.

Usage: neuron6_build_log.py <phase> [args...] <verbosity>
"""

import datetime as _dt
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from neuron6_style import header, kv, out


def main():
    argv = sys.argv[1:]
    verbosity = "minimal"
    if argv and argv[-1] in ("minimal", "verbose"):
        verbosity = argv[-1]
        argv = argv[:-1]

    phase = argv[0] if argv else "start"
    project = argv[1] if len(argv) > 1 else ""
    opt = argv[3] if len(argv) > 3 else ""

    if phase == "start":
        header("build")
        kv("sketch", project or "(unnamed)")
        detail = "Ohm Lab Neuro N6"
        if opt:
            detail += "  (optimize %s)" % opt
        kv("board", detail)
        if verbosity == "verbose":
            kv("started", _dt.datetime.now().strftime("%H:%M:%S"))
    elif verbosity != "verbose":
        return  # minimal prints only the start header
    elif phase == "preflight-ok":
        kv("toolchain", "ok - compiling core, libraries and sketch")
    elif phase == "signing":
        kv("link", "ok - signing trusted image")
    elif phase == "done":
        kv("sign", "ok - firmware ready to upload")
    else:
        out("  [build] %s" % " ".join(argv))


if __name__ == "__main__":
    main()
