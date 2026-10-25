/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* Host stand-in for arm_math_types.h. vnd_meta.h tests the guard name. */
#ifndef _ARM_MATH_TYPES_H_
#define _ARM_MATH_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#include <math.h>

typedef float  float32_t;
typedef double float64_t;
typedef int8_t q7_t;
typedef int16_t q15_t;
typedef int32_t q31_t;

#ifndef Q7_MIN
#define Q7_MIN   ((q7_t)0x80)
#define Q15_MIN  ((q15_t)0x8000)
#define Q31_MIN  ((q31_t)0x80000000)
#define Q7_MAX   ((q7_t)0x7F)
#define Q15_MAX  ((q15_t)0x7FFF)
#define Q31_MAX  ((q31_t)0x7FFFFFFF)
#endif

#endif /* _ARM_MATH_TYPES_H_ */
