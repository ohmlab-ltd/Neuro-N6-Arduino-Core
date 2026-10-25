#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Post-build step: wrap the application in the STM32 image header.

tools/stm32_header.py writes the header. With SIGNING_TOOL set to "auto" or
to the executable's path, ST's STM32_SigningTool_CLI writes it.

Usage: sign_bin.py <build_path> <project_name>
"""

import glob as globmod
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stm32_header


def _line():
    print("+------------------------------------------------------------+")


def _bar(done, total, width=30):
    total = max(1, total)
    done = max(0, min(done, total))
    filled = int(width * done / total)
    return "[" + ("#" * filled) + ("-" * (width - filled)) + "]"


_SIGNING_TOOL_SEARCH_PATHS_WINDOWS = [
    os.path.join(os.environ.get("PROGRAMFILES", os.path.join("C:" + os.sep, "Program Files")),
                 "STMicroelectronics", "STM32Cube", "STM32CubeProgrammer", "bin",
                 "STM32_SigningTool_CLI.exe"),
    os.path.join("D:" + os.sep, "ST", "STM32CubeProgrammer", "bin", "STM32_SigningTool_CLI.exe"),
    os.path.join("C:" + os.sep, "ST", "STM32CubeProgrammer", "bin", "STM32_SigningTool_CLI.exe"),
]

_SIGNING_TOOL_SEARCH_PATHS_UNIX = [
    "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_SigningTool_CLI",
    os.path.expanduser("~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_SigningTool_CLI"),
    "/usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_SigningTool_CLI",
]

_SIGNING_TOOL_SEARCH_PATHS = (
    _SIGNING_TOOL_SEARCH_PATHS_WINDOWS if os.name == "nt"
    else _SIGNING_TOOL_SEARCH_PATHS_UNIX
)

_CUBEIDE_GLOB_WINDOWS = os.path.join(
    os.environ.get("PROGRAMFILES", os.path.join("C:" + os.sep, "Program Files")),
    "STMicroelectronics", "STM32Cube", "STM32CubeIDE*", "**",
    "STM32_SigningTool_CLI.exe")

_CUBEIDE_GLOB_MACOS = "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macos64_*/tools/bin/STM32_SigningTool_CLI"

_CUBEIDE_GLOB = _CUBEIDE_GLOB_WINDOWS if os.name == "nt" else _CUBEIDE_GLOB_MACOS


def find_signing_tool():
    """Locate STM32_SigningTool_CLI, returning the path or None."""
    for p in _SIGNING_TOOL_SEARCH_PATHS:
        if os.path.isfile(p):
            return p
    for p in globmod.glob(_CUBEIDE_GLOB, recursive=True):
        if os.path.isfile(p):
            return p
    import shutil
    return shutil.which("STM32_SigningTool_CLI") or shutil.which("STM32_SigningTool_CLI.exe")


def _header_banner(in_bin, out_bin, method):
    _line()
    print("| Neuro N6 Signing                                          |")
    _line()
    print("  Input     : %s" % os.path.basename(in_bin))
    print("  Output    : %s" % os.path.basename(out_bin))
    print("  Method    : %s" % method)


def wrap_with_python(in_bin, out_bin):
    """Write the header with stm32_header.py."""
    with open(in_bin, "rb") as fh:
        payload = fh.read()
    image = stm32_header.wrap(payload)
    with open(out_bin, "wb") as fh:
        fh.write(image)
    fields = dict(stm32_header.describe(image))
    _header_banner(in_bin, out_bin, "STM32 header v2.3, no key (tools/stm32_header.py)")
    print("  Entry     : %s" % fields["entry point"])
    print("  Length    : %s" % fields["image length"])
    print("  Checksum  : %s" % fields["checksum"])
    print("  %s  image ready" % _bar(2, 2))
    print("  Note      : set SIGNING_TOOL=auto to use ST's tool instead")
    _line()
    return 0


def wrap_with_st_tool(in_bin, out_bin, choice):
    """Run STM32_SigningTool_CLI, found by path or by search when choice is auto."""
    tool = choice if os.path.isfile(choice) else None
    if tool is None and choice.lower() in ("auto", "st"):
        tool = find_signing_tool()
    if not tool:
        print("ERROR: SIGNING_TOOL is set but STM32_SigningTool_CLI was not found.", file=sys.stderr)
        print("       Install STM32CubeProgrammer, set SIGNING_TOOL to the executable,", file=sys.stderr)
        print("       or unset SIGNING_TOOL to use the built-in header writer.", file=sys.stderr)
        return 4
    build_path = os.path.dirname(in_bin)
    cmd = [
        tool,
        "-bin", os.path.basename(in_bin),
        "-nk",
        "-of", "0x80000000",
        "-t", "fsbl",
        "-o", os.path.basename(out_bin),
        "-hv", "2.3",
        "-dump", os.path.basename(out_bin),
        "-align",
    ]
    _header_banner(in_bin, out_bin, "STM32_SigningTool_CLI, no key")
    print("  Tool      : %s" % tool)
    print("  %s  packaging trusted image" % _bar(1, 2))
    print("  Detail    : STM32 signing tool output follows")
    print()
    try:
        p = subprocess.run(cmd, cwd=build_path, input="y\n", text=True, check=False)
    except FileNotFoundError:
        print("ERROR: signing tool not found: %s" % tool, file=sys.stderr)
        return 4
    if p.returncode == 0:
        print()
        print("  %s  signed image ready" % _bar(2, 2))
        print("  Note      : unset SIGNING_TOOL to use the built-in header writer")
        _line()
    return p.returncode


def main():
    if len(sys.argv) < 3:
        print("Usage: sign_bin.py <build_path> <project_name>", file=sys.stderr)
        return 2
    build_path, project_name = sys.argv[1], sys.argv[2]
    in_bin = os.path.join(build_path, project_name + ".bin")
    out_bin = os.path.join(build_path, project_name + "_Trusted.bin")
    if not os.path.isfile(in_bin):
        print("ERROR: input bin not found: %s" % in_bin, file=sys.stderr)
        return 3
    choice = os.environ.get("SIGNING_TOOL", "").strip()
    if choice:
        return wrap_with_st_tool(in_bin, out_bin, choice)
    return wrap_with_python(in_bin, out_bin)


if __name__ == "__main__":
    raise SystemExit(main())
