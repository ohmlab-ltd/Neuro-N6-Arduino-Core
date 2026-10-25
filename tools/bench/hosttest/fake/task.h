/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* Stand-in for the device header, for test_asset.c. MIT, Ohm Lab Ltd. */
#pragma once
#define taskSCHEDULER_SUSPENDED   0
#define taskSCHEDULER_NOT_STARTED 1
#define taskSCHEDULER_RUNNING     2
extern int g_susp;
static int xTaskGetSchedulerState(void) { return g_susp ? taskSCHEDULER_SUSPENDED : taskSCHEDULER_RUNNING; }
static void vTaskSuspendAll(void) { g_susp++; }
static int xTaskResumeAll(void) { g_susp--; return 0; }
