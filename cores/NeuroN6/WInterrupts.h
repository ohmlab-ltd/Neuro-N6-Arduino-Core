/**
 ******************************************************************************
 * @file    WInterrupts.h
 * @brief   Arduino attachInterrupt()/detachInterrupt() over STM32N6 EXTI.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef WINTERRUPTS_H
#define WINTERRUPTS_H

#include <stdint.h>
#include "wiring_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/* mode is RISING, FALLING or CHANGE. EXTI line N serves pin N of one port at
 * a time: a second attach on the same line is rejected. */
void attachInterrupt(uint32_t pin, voidFuncPtr callback, uint32_t mode);
void detachInterrupt(uint32_t pin);

#ifdef __cplusplus
}
#endif

#endif /* WINTERRUPTS_H */
