/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* Stand-in for the HAL, for test_tick.c. MIT, Ohm Lab Ltd. */
#pragma once
#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef struct { uint32_t Period, Prescaler, ClockDivision, CounterMode; } TIM_Base_InitTypeDef;
typedef struct { void *Instance; TIM_Base_InitTypeDef Init; } TIM_HandleTypeDef;

#define TIM16                         ((void *)0x16)
#define TIM16_IRQn                    133
#define TIM_COUNTERMODE_UP            0u
#define TIM_IT_UPDATE                 1u
#define __HAL_RCC_TIM16_CLK_ENABLE()  do { } while (0)
#define __HAL_TIM_DISABLE_IT(h, it)   do { (void)(h); (void)(it); } while (0)
#define __HAL_TIM_ENABLE_IT(h, it)    do { (void)(h); (void)(it); } while (0)

static inline void HAL_NVIC_SetPriority(int irq, uint32_t p, uint32_t s) { (void)irq; (void)p; (void)s; }
static inline void HAL_NVIC_EnableIRQ(int irq) { (void)irq; }
static inline uint32_t HAL_RCCEx_GetTIMGFreq(void) { return 400000000u; }
static inline HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *h) { (void)h; return HAL_OK; }
static inline HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h) { (void)h; return HAL_OK; }

extern volatile uint32_t uwTick;
static inline void HAL_IncTick(void) { uwTick++; }
void HAL_Delay(uint32_t ms);

static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __set_PRIMASK(uint32_t v) { (void)v; }
static inline void __disable_irq(void) { }

typedef struct { volatile uint32_t CTRL, LOAD, VAL; } SysTick_Type;
typedef struct { volatile uint32_t ICSR; } SCB_Type;
typedef struct { volatile uint32_t CYCCNT; } DWT_Type;
extern SysTick_Type g_systick;
extern SCB_Type g_scb;
extern DWT_Type g_dwt;
#define SysTick  (&g_systick)
#define SCB      (&g_scb)
#define DWT      (&g_dwt)
#define SCB_ICSR_PENDSTSET_Msk (1u << 26)
