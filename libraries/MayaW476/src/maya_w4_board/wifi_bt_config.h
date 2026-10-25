/**
  ******************************************************************************
  * @file    wifi_bt_config.h
  * @brief   MAYA-W476 board bring-up helpers.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#ifndef WIFI_BT_CONFIG_H
#define WIFI_BT_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MAYA_W4_OK            = 0,
    MAYA_W4_ERR_SDIO_INIT = -2,  /* enumeration failed */
    MAYA_W4_ERR_SDIO_CMD52 = -3, /* the CCCR read failed */
} maya_w4_status_t;

/** Pulses PDn low and waits. No SDIO traffic until it returns. */
void MAYA_W4_PowerCycle(void);

/** Power-cycles the module, enumerates it and reads the CCCR SDIO revision
 *  into out_sdio_revision, which may be NULL. The IW610 reports 0x04. */
maya_w4_status_t MAYA_W4_FirstLightSDIO(uint8_t *out_sdio_revision);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_BT_CONFIG_H */
