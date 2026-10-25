/**
 ******************************************************************************
 * @file    wiring_shift.c
 * @brief   shiftIn() and shiftOut(), bit-banged with no added delays.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "wiring_digital.h"
#include "wiring_shift.h"

uint8_t shiftIn(uint32_t dataPin, uint32_t clockPin, uint32_t bitOrder)
{
  uint8_t value = 0;

  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(clockPin, HIGH);
    if (bitOrder == LSBFIRST) {
      value |= (uint8_t)(digitalRead(dataPin) << i);
    } else {
      value = (uint8_t)((value << 1) | digitalRead(dataPin));
    }
    digitalWrite(clockPin, LOW);
  }
  return value;
}

void shiftOut(uint32_t dataPin, uint32_t clockPin, uint32_t bitOrder, uint8_t val)
{
  for (uint8_t i = 0; i < 8; i++) {
    if (bitOrder == LSBFIRST) {
      digitalWrite(dataPin, (val & (1u << i)) ? HIGH : LOW);
    } else {
      digitalWrite(dataPin, (val & (1u << (7 - i))) ? HIGH : LOW);
    }
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
  }
}
