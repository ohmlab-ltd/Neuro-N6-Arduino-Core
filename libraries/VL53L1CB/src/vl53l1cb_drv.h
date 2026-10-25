/**
 ******************************************************************************
 * @file    vl53l1cb_drv.h
 * @brief   VL53L1CB register access and single-zone ranging, using the
 *          sensor's own (non-histogram) ranging.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#ifndef VL53L1CB_DRV_H
#define VL53L1CB_DRV_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L1CB_DRV_I2C_ADDR   0x52u     /* 8-bit form of 0x29 */
#define VL53L1CB_DRV_MODEL_ID   0xEACCu   /* model 0xEA, module type 0xCC */

/* Soft reset, load the configuration and calibrate VHV once: long distance
 * mode, 100 ms timing budget, a 0.125 MCPS signal minimum and a 15 mm sigma
 * limit. 0 on success. */
int32_t  vl53l1cb_drv_init(void);

uint16_t vl53l1cb_drv_model_id(void);       /* 0 when the sensor does not answer */

/* A size x size SPAD region centred on (row, col) of the 16x16 array, rows
 * counted from the bottom. size is 4 to 16. */
int32_t  vl53l1cb_drv_set_roi(uint8_t row, uint8_t col, uint8_t size);

/* Starts timed ranging. The first result is ready after one timing budget;
 * call vl53l1cb_drv_stop() after reading it to measure a single zone. */
int32_t  vl53l1cb_drv_start(void);
int32_t  vl53l1cb_drv_data_ready(void);     /* 1 ready, 0 not yet, < 0 error */

/* Reads the result and clears the interrupt. *valid is 1 only when the
 * sensor reports a good range. */
int32_t  vl53l1cb_drv_read(uint16_t *mm, uint8_t *valid);

void     vl53l1cb_drv_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* VL53L1CB_DRV_H */
