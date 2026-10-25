# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
# Falls back to the pyserial copy in tools/pyserial when none is installed.
import os
import sys

try:
    import serial  # noqa: F401
except ImportError:
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "pyserial"))
