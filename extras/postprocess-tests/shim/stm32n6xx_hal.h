/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* Host stand-in: vnd_meta.c uses these to guard the Terminal log. */
#ifndef STM32N6XX_HAL_H_SHIM
#define STM32N6XX_HAL_H_SHIM

#include <stdint.h>

static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __set_PRIMASK(uint32_t v) { (void)v; }
static inline void __disable_irq(void) { }
static inline void __enable_irq(void) { }

#endif
