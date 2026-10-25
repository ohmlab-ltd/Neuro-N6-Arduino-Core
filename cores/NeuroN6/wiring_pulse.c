/**
 ******************************************************************************
 * @file    wiring_pulse.c
 * @brief   pulseIn()/pulseInLong() via the DWT cycle counter.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "wiring_digital.h"
#include "wiring_pulse.h"

extern uint32_t SystemCoreClock;

#define PULSE_MAX_TIMEOUT_US 5000000UL  /* < 2^32 cycles at 800 MHz */

uint32_t pulseIn(uint32_t pin, uint32_t state, uint32_t timeout_us)
{
  const PinDescription *d = pin_desc(pin);
  if (d == NULL) { return 0; }

  if (timeout_us > PULSE_MAX_TIMEOUT_US) { timeout_us = PULSE_MAX_TIMEOUT_US; }

  uint32_t cycles_per_us  = SystemCoreClock / 1000000u;
  uint32_t timeout_cycles = timeout_us * cycles_per_us;
  uint32_t want           = (state != LOW) ? d->mask : 0u;

  uint32_t start = DWT->CYCCNT;

  /* Skips a pulse already in progress. One timeout covers all three waits. */
  while ((d->port->IDR & d->mask) == want) {
    if ((DWT->CYCCNT - start) > timeout_cycles) { return 0; }
  }
  while ((d->port->IDR & d->mask) != want) {
    if ((DWT->CYCCNT - start) > timeout_cycles) { return 0; }
  }
  uint32_t pulse_start = DWT->CYCCNT;
  while ((d->port->IDR & d->mask) == want) {
    if ((DWT->CYCCNT - start) > timeout_cycles) { return 0; }
  }
  return (DWT->CYCCNT - pulse_start) / cycles_per_us;
}

uint32_t pulseInLong(uint32_t pin, uint32_t state, uint32_t timeout_us)
{
  return pulseIn(pin, state, timeout_us);
}
