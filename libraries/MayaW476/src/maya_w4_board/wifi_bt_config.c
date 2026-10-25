/**
  ******************************************************************************
  * @file    wifi_bt_config.c
  * @brief   MAYA-W476 bring-up: PDn power sequencing and the first SDIO probe.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#include "wifi_bt_config.h"
#include "maya_w4_pinmap.h"
#include "stm32n6_sdio_host.h"

#include "cmsis_os2.h"

/* Power sequencing. */

void MAYA_W4_PowerCycle(void)
{
    /* Datasheet minima: 1 us low and about 20 ms after release. */
    MAYA_W4_AssertPowerDown();
    osDelay(500U);
    MAYA_W4_ReleasePowerDown();
    osDelay(1000U);
}

/* First SDIO probe. */

#define CCCR_SDIO_REVISION_OFFSET   0x00U

maya_w4_status_t MAYA_W4_FirstLightSDIO(uint8_t *out_sdio_revision)
{
    STM32N6_SDIO_BindHandle(MAYA_WIFI_SDIO_HANDLE);

    MAYA_W4_PowerCycle();

    extern int sdio_drv_init(void (*cd_int)(int));
    extern int sdio_drv_creg_read(int addr, int fn, uint32_t *resp);

    if (sdio_drv_init(NULL) != 0 /* WM_SUCCESS */) {
        return MAYA_W4_ERR_SDIO_INIT;
    }

    uint32_t resp = 0U;
    if (sdio_drv_creg_read(CCCR_SDIO_REVISION_OFFSET, 0 /* fn0 */, &resp) != 1) {
        return MAYA_W4_ERR_SDIO_CMD52;
    }

    if (out_sdio_revision != NULL) {
        *out_sdio_revision = (uint8_t)(resp & 0xFFU);
    }
    return MAYA_W4_OK;
}
