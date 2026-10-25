/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* Stand-in for the kernel, for test_tick.c. MIT, Ohm Lab Ltd. */
#pragma once
#define taskSCHEDULER_SUSPENDED   0
#define taskSCHEDULER_NOT_STARTED 1
#define taskSCHEDULER_RUNNING     2
extern int g_sched;
extern uint32_t g_kticks;
extern int g_in_isr;
static inline BaseType_t xTaskGetSchedulerState(void) { return g_sched; }
static inline TickType_t xTaskGetTickCount(void) { return g_kticks; }
static inline TickType_t xTaskGetTickCountFromISR(void) { return g_kticks; }
static inline BaseType_t xPortIsInsideInterrupt(void) { return g_in_isr ? pdTRUE : pdFALSE; }
static inline void vTaskDelay(TickType_t t) { (void)t; }
#define taskYIELD() do { } while (0)
