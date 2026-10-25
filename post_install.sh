#!/usr/bin/env bash
# Copyright (c) 2026 Ohm Lab Ltd.
# SPDX-License-Identifier: MIT
# Linux only. Installs the udev rule for the Neuro N6. Boards Manager runs it
# after an install. On a clone:  sudo ./post_install.sh
set -u

SELF="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
RULES="$(dirname "$SELF")/extras/udev/99-neuron6.rules"
DEST=/etc/udev/rules.d/99-neuron6.rules

if [ "$(uname -s)" != "Linux" ]; then
  exit 0
fi
if [ ! -f "$RULES" ]; then
  echo "post_install.sh: $RULES not found" >&2
  exit 1
fi

if [ "$(id -u)" -ne 0 ]; then
  if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && command -v pkexec >/dev/null 2>&1; then
    exec pkexec "$SELF" "$@"
  fi
  if command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; then
    exec sudo -n "$SELF" "$@"
  fi
  echo "post_install.sh: root is needed to write $DEST. Run:  sudo $SELF" >&2
  exit 0
fi

install -m 0644 "$RULES" "$DEST"
if command -v udevadm >/dev/null 2>&1; then
  udevadm control --reload-rules 2>/dev/null || true
  udevadm trigger --subsystem-match=usb --subsystem-match=tty 2>/dev/null || true
fi
echo "post_install.sh: installed $DEST"
