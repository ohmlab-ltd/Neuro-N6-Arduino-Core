#!/usr/bin/env python3
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Memory-use summary for a build.

The section table goes to stdout for recipe.size.regex. The summary goes
to stderr.

Usage: neuron6_size.py <arm-none-eabi-size> <elf> [max_flash] [verbosity]
"""

import os
import sys
import subprocess

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from neuron6_style import bar, group, human, item, out, rule

# Sections of system/linker/STM32N657I0HXQ_LRUN.ld. The loaded image holds
# .data's initial values and .ext_psram_init's load copy.
FLASH_SECTIONS = {
    ".isr_vector", ".text", ".rodata", ".ARM.extab", ".ARM",
    ".preinit_array", ".init_array", ".fini_array", ".data", ".noncacheable",
    ".gnu.sgstubs", ".ext_psram_init",
}
RAM_SECTIONS = {".data", ".noncacheable", ".bss", ".axisram_nc", "._user_heap_stack"}
PSRAM_SECTIONS = {".ext_psram_init", ".ext_psram"}


def _parse_size_a(text):
    sizes = {}
    for line in text.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].startswith("."):
            try:
                sizes[parts[0]] = int(parts[1], 0)
            except ValueError:
                pass
    return sizes


def main():
    if len(sys.argv) < 3:
        print("Usage: neuron6_size.py <arm-none-eabi-size> <elf> [max_flash] [verbosity]",
              file=sys.stderr)
        return 2

    size_cmd = sys.argv[1]
    elf = sys.argv[2]
    max_flash = int(sys.argv[3], 0) if len(sys.argv) > 3 and sys.argv[3] else 0
    verbosity = sys.argv[4] if len(sys.argv) > 4 else "minimal"

    proc = subprocess.run([size_cmd, "-A", elf], text=True, capture_output=True)
    if proc.stdout:
        print(proc.stdout.rstrip())
    if proc.returncode != 0:
        if proc.stderr:
            print(proc.stderr.rstrip(), file=sys.stderr)
        return proc.returncode

    sections = _parse_size_a(proc.stdout)
    flash = sum(v for k, v in sections.items() if k in FLASH_SECTIONS)
    ram = sum(v for k, v in sections.items() if k in RAM_SECTIONS)
    psram = sum(v for k, v in sections.items() if k in PSRAM_SECTIONS)

    group("MEMORY")
    if max_flash:
        item("flash", "%s  %s of %s" % (bar(flash / max_flash), human(flash),
                                        human(max_flash)))
    else:
        item("flash", human(flash))
    item("ram", "%s (axisram data + bss)" % human(ram))
    if psram:
        item("psram", human(psram))
    if verbosity == "verbose":
        other = sum(sections.values()) - flash - ram - psram
        if other > 0:
            item("other", human(other))
        item("image", os.path.basename(elf))
    out()
    rule()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
