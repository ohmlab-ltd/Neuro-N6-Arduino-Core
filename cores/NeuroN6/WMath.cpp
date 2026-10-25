/**
 ******************************************************************************
 * @file    WMath.cpp
 * @brief   Arduino random()/randomSeed()/map().
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include <stdlib.h>
#include <stdint.h>

void randomSeed(unsigned long seed)
{
  if (seed != 0) {
    srand((unsigned int)seed);
  }
}

long random(long howbig)
{
  if (howbig <= 0) {
    return 0;
  }
  return rand() % howbig;
}

long random(long howsmall, long howbig)
{
  if (howsmall >= howbig) {
    return howsmall;
  }
  long diff = howbig - howsmall;
  return random(diff) + howsmall;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  long in_range = in_max - in_min;
  if (in_range == 0) {
    return out_min;
  }
  return (x - in_min) * (out_max - out_min) / in_range + out_min;
}
