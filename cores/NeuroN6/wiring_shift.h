/**
 ******************************************************************************
 * @file    wiring_shift.h
 * @brief   Arduino shiftIn()/shiftOut().
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef WIRING_SHIFT_H
#define WIRING_SHIFT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t shiftIn(uint32_t dataPin, uint32_t clockPin, uint32_t bitOrder);
void    shiftOut(uint32_t dataPin, uint32_t clockPin, uint32_t bitOrder, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /* WIRING_SHIFT_H */
