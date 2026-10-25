/* Host test of HAL_GetTick() and micros() across the scheduler's start,
 * cores/NeuroN6/stm32n6xx_hal_timebase_tim.c and wiring.c. From this folder:
 *
 *   gcc -std=c11 -Wall -Ifake_tick test_tick.c -o /tmp/test_tick
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <stdint.h>
#include "stm32n6xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

volatile uint32_t uwTick;
uint32_t SystemCoreClock = 400000000u;
SysTick_Type g_systick;
SCB_Type g_scb;
DWT_Type g_dwt;
int g_sched = taskSCHEDULER_NOT_STARTED;
uint32_t g_kticks;
int g_in_isr;
void HAL_Delay(uint32_t ms) { (void)ms; }

#include "../../../cores/NeuroN6/stm32n6xx_hal_timebase_tim.c"
#include "../../../cores/NeuroN6/wiring.c"

static int fails, checks;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

#define CYC_PER_MS 400000u

/* Cycles since the scheduler started. SysTick reloads every CYC_PER_MS, and
 * its interrupt is taken `latency` cycles later; TIM16 counts uwTick on with
 * its own phase. */
static uint64_t g_e;
static uint32_t g_u0, g_phase, g_latency;

static void place(uint64_t e)
{
  const uint64_t reloads = e / CYC_PER_MS;
  const uint64_t taken = (e >= g_latency) ? (e - g_latency) / CYC_PER_MS : 0u;
  g_e = e;
  g_systick.VAL = (CYC_PER_MS - 1u) - (uint32_t)(e % CYC_PER_MS);
  g_kticks = (uint32_t)taken;
  g_scb.ICSR = (reloads > taken) ? SCB_ICSR_PENDSTSET_Msk : 0u;
  uwTick = g_u0 + (uint32_t)((e + g_phase) / CYC_PER_MS);
}

int main(void)
{
  uint32_t last_ms, last_us;

  /* 1. before the scheduler: uwTick, whole milliseconds */
  uwTick = 1234u;
  CHECK(HAL_GetTick() == 1234u);
  CHECK(millis() == 1234u);
  CHECK(micros() == 1234000u);

  /* 2. a SysTick left loaded and stopped before the scheduler adds nothing */
  g_systick.LOAD = CYC_PER_MS - 1u;
  g_systick.VAL  = 123u;
  CHECK(micros() == 1234000u);
  last_ms = HAL_GetTick();
  last_us = micros();

  /* 3. the scheduler starts: the kernel tick from 0, uwTick counting on */
  g_u0 = 1234u;
  g_phase = 150000u;
  g_latency = 30000u;
  g_sched = taskSCHEDULER_RUNNING;
  place(0u);
  CHECK(HAL_GetTick() >= last_ms);
  CHECK(micros() >= last_us);

  /* 4. five seconds in steps that land everywhere in a millisecond, with the
   *    interrupt late: never backwards, and the time since boot */
  {
    int back_ms = 0, back_us = 0, off = 0;
    for (uint64_t e = 0u; e < 5000ull * CYC_PER_MS; e += 7919u) {
      place(e);
      const uint32_t ms = HAL_GetTick(), us = micros();
      if ((int32_t)(ms - last_ms) < 0) back_ms++;
      if ((int32_t)(us - last_us) < 0) back_us++;
      const int32_t lag = (int32_t)(1234000u + (uint32_t)(e / 400u) - us);
      if (lag < -1000 || lag > 1000) off++;
      last_ms = ms;
      last_us = us;
    }
    CHECK(back_ms == 0);
    CHECK(back_us == 0);
    CHECK(off == 0);
  }

  /* 5. an interrupt reads the same time */
  place(5000ull * CYC_PER_MS + 40000u);
  {
    const uint32_t task = HAL_GetTick();
    g_in_isr = 1;
    CHECK(HAL_GetTick() == task);
    g_in_isr = 0;
    CHECK(task == 1234u + 5000u);
  }

  printf("%d checks, %d failed\n", checks, fails);
  return fails != 0;
}
