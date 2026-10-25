/**
 ******************************************************************************
 * @file    wiring_constants.h
 * @brief   Arduino-canonical constants and helper macros for the Neuro N6.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef WIRING_CONSTANTS_H
#define WIRING_CONSTANTS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LOW  0x0
#define HIGH 0x1

/* Pin modes, with the ArduinoCore-API values. */
#define INPUT           0x0
#define OUTPUT          0x1
#define INPUT_PULLUP    0x2
#define INPUT_PULLDOWN  0x3
#define OUTPUT_OPENDRAIN 0x4

/* attachInterrupt edge modes, with the ArduinoCore-API values. EXTI has no
 * level interrupts, so attachInterrupt() rejects LOW and HIGH. */
#define CHANGE  2
#define FALLING 3
#define RISING  4

#define LSBFIRST 0
#define MSBFIRST 1

/* Print bases, kept in step with the defines in HardwareSerial.h. */
#ifndef DEC
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#endif

/* In C, min, max and abs are macros, so an argument may be evaluated twice.
 * C++ gets min and max as templates (below) and abs from <stdlib.h>, so the
 * standard library headers still compile after Arduino.h. */
#ifndef __cplusplus
#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef abs
#define abs(x) ((x) > 0 ? (x) : -(x))
#endif
#endif
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif
/* No round() macro: it would break the declaration in newlib's <math.h>. */
#ifndef radians
#define radians(deg) ((deg) * DEG_TO_RAD)
#endif
#ifndef degrees
#define degrees(rad) ((rad) * RAD_TO_DEG)
#endif
#ifndef sq
#define sq(x) ((x) * (x))
#endif

#define PI          3.1415926535897932384626433832795
#define HALF_PI     1.5707963267948966192313216916398
#define TWO_PI      6.283185307179586476925286766559
#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG  57.295779513082320876798154814105
#define EULER       2.718281828459045235360287471352

#define lowByte(w)  ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define bitRead(value, bit)  (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)   ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define bit(b) (1UL << (b))

/* PRIMASK. The HAL tick and the IWDG feed stall while interrupts are masked. */
#define interrupts()   __enable_irq()
#define noInterrupts() __disable_irq()

typedef bool boolean;
typedef uint8_t byte;
typedef uint16_t word;
typedef void (*voidFuncPtr)(void);

#ifdef __cplusplus
}

#include <stdlib.h>

/* <stdlib.h> has no unsigned abs(). An unsigned value is its own absolute
 * value, as the C macro gives. */
constexpr unsigned int abs(unsigned int x) { return x; }
constexpr unsigned long abs(unsigned long x) { return x; }
constexpr unsigned long long abs(unsigned long long x) { return x; }

/* As in ArduinoCore-API's Common.h. */
#ifndef min
template<class T, class L>
constexpr auto min(const T& a, const L& b) -> decltype((b < a) ? b : a)
{
  return (b < a) ? b : a;
}
#endif

#ifndef max
template<class T, class L>
constexpr auto max(const T& a, const L& b) -> decltype((b < a) ? b : a)
{
  return (a < b) ? b : a;
}
#endif
#endif /* __cplusplus */

#endif /* WIRING_CONSTANTS_H */
