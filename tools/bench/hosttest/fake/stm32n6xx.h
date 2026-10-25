/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* Stand-in for the device header, for test_asset.c. MIT, Ohm Lab Ltd. */
#pragma once
#include <stdint.h>
#include <stdlib.h>
typedef struct { volatile uint32_t BKP0R; volatile uint32_t rest[64]; } TAMP_TypeDef;
extern TAMP_TypeDef g_tamp;
#define TAMP (&g_tamp)
typedef struct { volatile uint32_t DBPCR; } PWR_TypeDef;
extern PWR_TypeDef g_pwr;
#define PWR (&g_pwr)
#define PWR_DBPCR_DBP (1U << 0)
#define __DSB() do { } while (0)
/* A reset ends the call that asked for it: the test jumps back. */
void fake_system_reset(void);
static void NVIC_SystemReset(void) { fake_system_reset(); }
/* The D-cache over the mapped flash. */
#define __DCACHE_PRESENT 1U
void SCB_InvalidateDCache_by_Addr(volatile void *addr, int32_t dsize);
