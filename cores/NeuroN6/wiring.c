/**
 ******************************************************************************
 * @file    wiring.c
 * @brief   Arduino timing primitives.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 ******************************************************************************
 */

#include "stm32n6xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

extern uint32_t SystemCoreClock;

uint32_t millis(void) { return HAL_GetTick(); }

void delay(uint32_t ms)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    /* In 64 bits: pdMS_TO_TICKS() wraps past 4,294,967 ms. */
    vTaskDelay((TickType_t)(((uint64_t)ms * configTICK_RATE_HZ) / 1000u));
  } else {
    HAL_Delay(ms);
  }
}

void yield(void)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    taskYIELD();
  }
}

uint32_t micros(void)
{
  const uint32_t load = SysTick->LOAD + 1u;          /* SysTick cycles per ms */
  /* SysTick counts the kernel tick only once the scheduler runs. */
  if ((load <= 1u) || (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)) {
    return HAL_GetTick() * 1000u;
  }
  uint32_t ms, val, ms2, p1, p2;
  do {
    ms  = HAL_GetTick();
    p1  = SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;
    val = SysTick->VAL;
    p2  = SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;
    ms2 = HAL_GetTick();
  } while ((ms != ms2) || (p1 != p2));

  /* PENDSTSET: SysTick has reloaded and the tick is not yet counted. */
  if (p2 != 0u) ms++;
  uint32_t elapsed = load - val;                     /* cycles into this ms */
  if (elapsed >= load) elapsed = load - 1u;          /* guard a VAL==0 sample */
  const uint32_t sub_us = (uint32_t)(((uint64_t)elapsed * 1000u) / load);
  return ms * 1000u + sub_us;
}

void delayMicroseconds(uint32_t us)
{
  /* DWT is enabled in NeuroN6_Setup(). Waits in chunks: the 32-bit counter
   * compare is wrap-safe only below 2^32 cycles. */
  const uint32_t cyc_per_us = SystemCoreClock / 1000000u;
  uint64_t cycles = (uint64_t)us * (uint64_t)cyc_per_us;
  while (cycles > 0u) {
    const uint32_t chunk = (cycles > 0x40000000u) ? 0x40000000u : (uint32_t)cycles;
    const uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < chunk) {}
    cycles -= chunk;
  }
}

/* FreeRTOS idle hook. The NeuroN6-Powersave library overrides it. */
__attribute__((weak)) void vApplicationIdleHook(void) {}
