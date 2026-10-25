/**
 ******************************************************************************
 * @file    wiring_pulse.h
 * @brief   Arduino pulseIn()/pulseInLong().
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef WIRING_PULSE_H
#define WIRING_PULSE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Returns the pulse width in microseconds, or 0 on timeout. Busy-waits. */
uint32_t pulseIn(uint32_t pin, uint32_t state, uint32_t timeout_us);
uint32_t pulseInLong(uint32_t pin, uint32_t state, uint32_t timeout_us);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
inline uint32_t pulseIn(uint32_t pin, uint32_t state) { return pulseIn(pin, state, 1000000UL); }
inline uint32_t pulseInLong(uint32_t pin, uint32_t state) { return pulseInLong(pin, state, 1000000UL); }
#endif

#endif /* WIRING_PULSE_H */
