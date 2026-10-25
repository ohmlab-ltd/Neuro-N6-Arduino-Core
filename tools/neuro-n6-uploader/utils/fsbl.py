# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Identifies the bootloader release by the CRC32 of its image in flash.

Each RELEASES row is the version, size in bytes and zlib CRC32 of one
Neuro-N6-FSBL-vX.Y.Z-Trusted.bin.
"""

RELEASES = (
    ((2, 6, 2), 83744, 0x3C092073),
    ((2, 6, 1), 83744, 0xA1CBFD65),
    ((2, 6, 0), 83680, 0x2E77445C),
    ((2, 5, 0), 84672, 0x933535FB),
    ((2, 4, 0), 82944, 0x5A5F9417),
    ((2, 3, 1), 82944, 0xBA47754D),
    ((2, 2, 2), 82048, 0xDBC5FE2B),
    ((2, 2, 0), 81984, 0xFCDB2624),
    ((2, 1, 0), 80960, 0x0C0B0A2A),
    ((2, 0, 2), 79136, 0x306B8D2B),
    ((2, 0, 1), 79136, 0x674E7585),
)

# The oldest bootloader this core supports.
MINIMUM = (2, 6, 1)


def identify(session):
    """The (major, minor, patch) of the bootloader in flash, or None when it
    is not in RELEASES or the bootloader has no CRC32 capability."""
    if not session.has_cap("CRC32"):
        return None
    crc_of = {}                              # one query per distinct size
    for version, size, crc in RELEASES:
        if size not in crc_of:
            crc_of[size] = session.crc32(0, size)
        if crc_of[size] == crc:
            return version
    return None


def text(version):
    return "v%d.%d.%d" % version
