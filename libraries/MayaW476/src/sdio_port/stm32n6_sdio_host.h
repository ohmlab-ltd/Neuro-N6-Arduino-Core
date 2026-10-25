/**
  ******************************************************************************
  * @file    stm32n6_sdio_host.h
  * @brief   STM32N6 SDIO host port for the wifi_nxp driver and the IW610.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#ifndef STM32N6_SDIO_HOST_H
#define STM32N6_SDIO_HOST_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32n6xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Must be called before sdio_drv_init(). */
void STM32N6_SDIO_BindHandle(SDIO_HandleTypeDef *hsdio);

/** The SDMMC interrupt vector must call this. */
void STM32N6_SDIO_IRQHandler(void);

/* The sdio_drv_* API is declared in wifi_nxp's mlan_sdio_api.h. */

#ifdef __cplusplus
}
#endif

#endif /* STM32N6_SDIO_HOST_H */
