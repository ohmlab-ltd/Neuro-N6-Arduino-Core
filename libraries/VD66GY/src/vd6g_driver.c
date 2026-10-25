/*
 * vd6g_driver.c: compiles the ST vd6g sensor driver as one translation unit.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include <string.h>

/* The driver asserts on paths reachable in normal operation, and a failing assert parks
 * the calling task. NDEBUG must be defined before vd6g.c includes <assert.h>. */
#define NDEBUG 1

#include "../extras/st-driver/sensors/vd6g/vd6g.c"
