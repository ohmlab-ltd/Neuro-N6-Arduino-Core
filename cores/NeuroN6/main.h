/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025-2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  *
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32n6xx_hal.h"

void Error_Handler(void);

#define GREEN_LED_Pin       GPIO_PIN_6
#define GREEN_LED_GPIO_Port GPIOD
#define BLUE_LED_Pin        GPIO_PIN_1
#define BLUE_LED_GPIO_Port  GPIOC
#define RED_LED_Pin         GPIO_PIN_11
#define RED_LED_GPIO_Port   GPIOE

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
