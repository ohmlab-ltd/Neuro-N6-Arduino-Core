/**
 ******************************************************************************
 * @file    api/Common.h
 * @brief   Ohm Lab's stand-in for ArduinoCore-API's Common.h.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef NEURON6_API_COMMON_SHIM_H
#define NEURON6_API_COMMON_SHIM_H

#include <stdint.h>

/* Same signatures as wiring.c and Arduino.h: uint32_t, not unsigned long. */
#ifdef __cplusplus
extern "C" {
#endif

uint32_t millis(void);
uint32_t micros(void);
void     delay(uint32_t ms);
void     delayMicroseconds(uint32_t us);

#ifdef __cplusplus
}
#endif

/* min, max and the other helpers. */
#include "wiring_constants.h"

#endif /* NEURON6_API_COMMON_SHIM_H */
