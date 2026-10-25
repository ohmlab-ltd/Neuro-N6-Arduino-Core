/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* Stand-in for Wire.h, for test_mmc5603nj.cpp. MIT, Ohm Lab Ltd. */
#pragma once
#include <stdint.h>
#include <stddef.h>
class TwoWire {
public:
  void    beginTransmission(uint8_t addr);
  size_t  write(uint8_t b);
  size_t  write(const uint8_t *p, size_t n);
  uint8_t endTransmission(bool stop = true);
  uint8_t requestFrom(uint8_t addr, uint8_t n);
  int     read(void);
};
extern TwoWire Wire;
