/**
 ******************************************************************************
 * @file    wiring_analog.h
 * @brief   Arduino analogRead / analogWrite / tone for the Neuro N6.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef WIRING_ANALOG_H
#define WIRING_ANALOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* VREF+ = 1.8 V. */
int  analogRead(uint32_t pin);
void analogReadResolution(int bits);        /* 8..16, default 10 */

void analogWrite(uint32_t pin, int value);
void analogWriteResolution(int bits);       /* 4..16, default 8 */
void analogWriteFrequency(uint32_t hz);     /* default 1000 */

void tone(uint32_t pin, unsigned int frequency, unsigned long duration_ms);
void noTone(uint32_t pin);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
inline void tone(uint32_t pin, unsigned int frequency) { tone(pin, frequency, 0UL); }
#endif

#endif /* WIRING_ANALOG_H */
