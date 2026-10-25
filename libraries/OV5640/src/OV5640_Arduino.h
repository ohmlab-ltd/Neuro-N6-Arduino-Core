/**
  ******************************************************************************
  * @file    OV5640_Arduino.h
  * @brief   Arduino wrapper for the OV5640 camera driver.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#ifndef OV5640_ARDUINO_H
#define OV5640_ARDUINO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    VGA     = 0x03,  /* 640x480 */
    WVGA    = 0x04,  /* 800x480 */
    XGA     = 0x05,  /* 1024x768 */
    HD720   = 0x06,  /* 1280x720 */
    FHD1080 = 0x07,  /* 1920x1080 */
    QSXGA   = 0x08   /* 2592x1944 */
} CameraResolution;

typedef enum {
    MIRROR_FLIP_NONE = 0x00,
    FLIP             = 0x01,
    MIRROR           = 0x02,
    MIRROR_FLIP      = 0x03
} CameraMirrorFlip;

/**
  * @brief  Resets, identifies and starts the OV5640.
  * @retval 0 on success; -1 bus IO, -2 ID read, -3 ID mismatch, -4 init, -5 flip, -6 start
  */
int32_t ov5640_init(CameraResolution resolution, CameraMirrorFlip mirror_flip);

void ov5640_write(uint16_t reg, uint8_t val);

uint8_t ov5640_read(uint16_t reg);

void ov5640_stream_on(void);

void ov5640_stream_off(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "OV5640_Sensor.h"
#endif

#endif /* OV5640_ARDUINO_H */
