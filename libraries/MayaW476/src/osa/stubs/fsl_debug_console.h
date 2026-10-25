/**
  ******************************************************************************
  * @file    fsl_debug_console.h
  * @brief   Stand-in for NXP's fsl_debug_console.h, mapped to newlib stdio.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#ifndef FSL_DEBUG_CONSOLE_H_STM32N6_STUB
#define FSL_DEBUG_CONSOLE_H_STM32N6_STUB

#include <stdio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PRINTF
#define PRINTF(...)        ((void)printf(__VA_ARGS__))
#endif

#ifndef DbgConsole_Printf
#define DbgConsole_Printf  printf
#endif

#ifndef DbgConsole_Scanf
#define DbgConsole_Scanf   scanf
#endif

#ifndef DbgConsole_Putchar
#define DbgConsole_Putchar putchar
#endif

#ifndef DbgConsole_Getchar
#define DbgConsole_Getchar getchar
#endif

#ifndef SCANF
#define SCANF(...)         ((void)scanf(__VA_ARGS__))
#endif

#ifndef PUTCHAR
#define PUTCHAR(c)         ((void)putchar(c))
#endif

#ifndef GETCHAR
#define GETCHAR()          getchar()
#endif

static inline int DbgConsole_Init(void *uartBase, uint32_t baudRate, uint8_t device, uint32_t clkSrcFreq)
{
    (void)uartBase; (void)baudRate; (void)device; (void)clkSrcFreq;
    return 0;
}

static inline int DbgConsole_Deinit(void) { return 0; }

#ifdef __cplusplus
}
#endif

#endif /* FSL_DEBUG_CONSOLE_H_STM32N6_STUB */
