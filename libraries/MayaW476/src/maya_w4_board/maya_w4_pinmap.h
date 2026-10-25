/**
  ******************************************************************************
  * @file    maya_w4_pinmap.h
  * @brief   Pin mapping between the STM32N6 and the MAYA-W476 module.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#ifndef MAYA_W4_PINMAP_H
#define MAYA_W4_PINMAP_H

#include "main.h"
#include "stm32n6xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Control lines. A -D flag in a board's build.extra_flags overrides one. */
#ifndef PDn_pin_MAYA_Pin
#define PDn_pin_MAYA_Pin                GPIO_PIN_5      /* PB5, power-down, active low */
#define PDn_pin_MAYA_GPIO_Port          GPIOB
#endif
#ifndef WL_WAKE_IN_MAYA_Pin
#define WL_WAKE_IN_MAYA_Pin             GPIO_PIN_0      /* PG0, host to module WLAN wake, output */
#define WL_WAKE_IN_MAYA_GPIO_Port       GPIOG
#endif
#ifndef WL_WAKE_OUT_MAYA_Pin
#define WL_WAKE_OUT_MAYA_Pin            GPIO_PIN_7      /* PB7, module to host WLAN wake, input */
#define WL_WAKE_OUT_MAYA_GPIO_Port      GPIOB
#endif
#ifndef NB_WAKE_IN_MAYA_Pin
#define NB_WAKE_IN_MAYA_Pin             GPIO_PIN_5      /* PE5, 802.15.4 wake-in, reserved */
#define NB_WAKE_IN_MAYA_GPIO_Port       GPIOE
#endif
#ifndef NB_WAKE_OUT_MAYA_Pin
#define NB_WAKE_OUT_MAYA_Pin            GPIO_PIN_10     /* PE10, 802.15.4 wake-out, reserved */
#define NB_WAKE_OUT_MAYA_GPIO_Port      GPIOE
#endif

/** PDn. Low powers the module down and loses all its state. */
#define MAYA_PDn_PORT                   PDn_pin_MAYA_GPIO_Port
#define MAYA_PDn_PIN                    PDn_pin_MAYA_Pin

/** SDMMC1 in SDIO mode: the Wi-Fi data path. */
extern SDIO_HandleTypeDef hsdio1;
#define MAYA_WIFI_SDIO_HANDLE           (&hsdio1)

static inline void MAYA_W4_AssertPowerDown(void)
{
    HAL_GPIO_WritePin(MAYA_PDn_PORT, MAYA_PDn_PIN, GPIO_PIN_RESET);
}

static inline void MAYA_W4_ReleasePowerDown(void)
{
    HAL_GPIO_WritePin(MAYA_PDn_PORT, MAYA_PDn_PIN, GPIO_PIN_SET);
}

#ifdef __cplusplus
}
#endif

#endif /* MAYA_W4_PINMAP_H */
