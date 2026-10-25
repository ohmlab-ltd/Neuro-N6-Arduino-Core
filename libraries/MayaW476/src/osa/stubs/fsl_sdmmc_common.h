/**
  ******************************************************************************
  * @file    fsl_sdmmc_common.h
  * @brief   Stand-in for NXP's SDMMC stack header: the names wifi_nxp uses.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#ifndef FSL_SDMMC_COMMON_H_STM32N6_STUB
#define FSL_SDMMC_COMMON_H_STM32N6_STUB

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SDIO block size in bytes. */
#ifndef FSL_SDMMC_DEFAULT_BLOCK_SIZE
#define FSL_SDMMC_DEFAULT_BLOCK_SIZE  (512U)
#endif

/* Opaque. wifi_nxp holds pointers to these and never dereferences them. */
typedef struct _sdio_card     sdio_card_t;
typedef struct _sdmmc_host    sdmmchost_t;

typedef enum
{
    kSDIO_FunctionNum0 = 0U,
    kSDIO_FunctionNum1 = 1U,
    kSDIO_FunctionNum2 = 2U,
    kSDIO_FunctionNum3 = 3U,
    kSDIO_FunctionNum4 = 4U,
    kSDIO_FunctionNum5 = 5U,
    kSDIO_FunctionNum6 = 6U,
    kSDIO_FunctionNum7 = 7U,
} sdio_func_num_t;

/* The MAYA-W4 runs at 1.8 V. */
typedef enum
{
    kSDMMC_OperationVoltage_None = 0U,
    kSDMMC_OperationVoltage180V  = 1U,
    kSDMMC_OperationVoltage300V  = 2U,
    kSDMMC_OperationVoltage330V  = 3U,
} sdmmc_operation_voltage_t;

#ifdef __cplusplus
}
#endif

#endif /* FSL_SDMMC_COMMON_H_STM32N6_STUB */
