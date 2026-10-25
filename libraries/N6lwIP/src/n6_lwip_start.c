/**
 ******************************************************************************
 * @file    n6_lwip_start.c
 * @brief   One tcpip_init() for every network library.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#include "N6lwip.h"
#include "lwip/tcpip.h"
#include "FreeRTOS.h"
#include "task.h"

/* 0 not started, 1 starting, 2 running */
static volatile int s_tcpip_state = 0;

static void tcpip_ready(void *arg)
{
  (void)arg;
  s_tcpip_state = 2;
}

int n6_lwip_tcpip_start(void)
{
  vTaskSuspendAll();
  const int first = (s_tcpip_state == 0);
  if (first) s_tcpip_state = 1;
  (void)xTaskResumeAll();

  if (first) tcpip_init(tcpip_ready, NULL);

  const TickType_t t0 = xTaskGetTickCount();
  while ((s_tcpip_state != 2) && ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(2000))) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return (s_tcpip_state == 2) ? 0 : -1;
}
