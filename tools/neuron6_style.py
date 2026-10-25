# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
"""Shared terminal styling for the Neuro N6 build tools, in ASCII only.

Output goes to stderr: the Arduino IDE hides a build command's stdout unless
its "verbose compile" preference is on.
"""

import sys

WIDTH = 64

# No ANSI codes: the Arduino IDE's output panel prints them as text.


def out(msg=""):
    print(msg, file=sys.stderr, flush=True)


def header(title):
    out("=" * WIDTH)
    out("  NEURO N6 %s" % title.upper())
    out("=" * WIDTH)


def rule():
    out("=" * WIDTH)


def group(title):
    out()
    out("  %s" % title)


def kv(key, value):
    """Top-level row."""
    out("  %-13s %s" % (key, value))


def item(key, value):
    """Row inside a group."""
    out("    %-13s %s" % (key, value))


def human(n):
    if n >= 1024 * 1024:
        return "%.2f MB" % (n / (1024.0 * 1024.0))
    if n >= 1024:
        return "%.1f KB" % (n / 1024.0)
    return "%d B" % n


def bar(fraction, width=34):
    """Fraction as a bar and a percentage."""
    fraction = max(0.0, min(1.0, fraction))
    filled = int(width * fraction)
    if 0 < filled < width:
        body = "=" * (filled - 1) + ">" + "." * (width - filled)
    elif filled >= width:
        body = "=" * width
    else:
        body = ">" + "." * (width - 1) if fraction > 0 else "." * width
    return "[%s] %5.1f%%" % (body, fraction * 100.0)
