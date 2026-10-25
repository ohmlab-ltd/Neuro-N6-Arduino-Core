/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* Stand-in for the kernel, for test_tick.c. MIT, Ohm Lab Ltd. */
#pragma once
#include <stdint.h>
typedef uint32_t TickType_t;
typedef long BaseType_t;
#define pdTRUE  1
#define pdFALSE 0
#define configTICK_RATE_HZ ((TickType_t)1000)
