# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
# USB identities and port discovery.
import time
import serial.tools.list_ports

VID = 0x1209                # pid.codes VID

APP_VID = VID
APP_PID = 0x8008            # running application

BOOTLOADER_VID = VID
BOOTLOADER_PID = 0x8009     # DFU bootloader, protocol v2

LEGACY_VID = 0xCAFE
LEGACY_APP_PID = 0x4013         # app, and the v1 DFU bootloader
LEGACY_BOOTLOADER_PID = 0x4014

# Match lists, preferred identity first.
APP_IDENTITIES = (
    (APP_VID, APP_PID),
    (LEGACY_VID, LEGACY_APP_PID),
)
BOOTLOADER_IDENTITIES = (
    (BOOTLOADER_VID, BOOTLOADER_PID),
    (LEGACY_VID, LEGACY_BOOTLOADER_PID),
    (LEGACY_VID, LEGACY_APP_PID),      # also an app identity
)


def _find_port_multi(identities):
    for vid, pid in identities:
        for p in serial.tools.list_ports.comports():
            if p.vid == vid and p.pid == pid:
                return p.device
    return None


def find_app_port():
    """Port of a running application, or None. 0xCAFE:0x4013 can also be a
    bootloader."""
    return _find_port_multi(APP_IDENTITIES)


def find_bootloader_port():
    """Port of a DFU bootloader, or None. 0xCAFE:0x4013 is matched last: it can
    also be an app."""
    return _find_port_multi(BOOTLOADER_IDENTITIES)


def is_bootloader_family(p):
    """True for a bootloader identity or anything on LEGACY_VID."""
    return ((p.vid, p.pid) in BOOTLOADER_IDENTITIES) or (p.vid == LEGACY_VID)


def wait_for_port_gone(port, timeout_sec=5.0, poll=0.05):
    """Returns True once `port` is gone, False on timeout."""
    if port is None:
        return True
    start = time.time()
    while (time.time() - start) < timeout_sec:
        if port not in [p.device for p in serial.tools.list_ports.comports()]:
            return True
        time.sleep(poll)
    return False


def wait_for_bootloader_port(timeout_sec=10.0, poll=0.1):
    """Returns the bootloader's port once it enumerates, or None on timeout."""
    start = time.time()
    while (time.time() - start) < timeout_sec:
        port = find_bootloader_port()
        if port:
            return port
        time.sleep(poll)
    return None
