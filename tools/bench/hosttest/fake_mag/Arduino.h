/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* Stand-in for Arduino.h, for test_mmc5603nj.cpp. MIT, Ohm Lab Ltd. */
#pragma once
#include <stdint.h>
#include <stddef.h>
uint32_t millis(void);
void delay(uint32_t ms);
