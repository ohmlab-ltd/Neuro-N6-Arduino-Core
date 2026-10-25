/**
 ******************************************************************************
 * @file    newlib_lock_glue.c
 * @brief   newlib to FreeRTOS glue: the malloc locks.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include <reent.h>
#include "FreeRTOS.h"
#include "task.h"

void __malloc_lock(struct _reent *reent)
{
  (void)reent;
  configASSERT(xPortIsInsideInterrupt() == pdFALSE);   /* no malloc in ISRs */
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    vTaskSuspendAll();
  }
}

void __malloc_unlock(struct _reent *reent)
{
  (void)reent;
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    (void)xTaskResumeAll();
  }
}
