/**
 ******************************************************************************
 * @file    VL53L1CB_Arduino.h
 * @brief   VL53L1CB time-of-flight sensor as a 4x4 grid.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#ifndef VL53L1CB_ARDUINO_H
#define VL53L1CB_ARDUINO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define VL53L1CB_OK                      0
#define VL53L1CB_ERROR                   (-1)
#define VL53L1CB_DEVICE_ADDRESS          0x52U   /* 8-bit form of 0x29 */

#define VL53L1CB_FRAME_ROWS              4U
#define VL53L1CB_FRAME_COLS              4U
#define VL53L1CB_FRAME_ZONE_COUNT        (VL53L1CB_FRAME_ROWS * VL53L1CB_FRAME_COLS)
#define VL53L1CB_FRAME_MIN_ROI_SPADS     4U
#define VL53L1CB_FRAME_TIMING_BUDGET_MS  100U

int32_t VL53L1CB_Setup4x4FrameCapture(void);
int32_t VL53L1CB_CaptureNextFrameZoneOnce(void);
void vnd_meta_build_tof(void);

uint16_t VL53L1CB_WhoAmI(void);   /* 0xEACC, or 0 when the sensor does not answer */
void     VL53L1CB_GetFrameMm(uint16_t out[VL53L1CB_FRAME_ROWS][VL53L1CB_FRAME_COLS]);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <Arduino.h>
#include <Wire.h>

/* Zone distances in millimetres, row-major. 0 is no target. */
typedef struct {
  uint16_t mm[VL53L1CB_FRAME_ROWS][VL53L1CB_FRAME_COLS];
} tofFrame;

class VL53L1CB {
  public:
    /* addr and theWire are ignored: the driver uses the on-board I2C bus. */
    bool     begin(uint8_t addr = VL53L1CB_DEVICE_ADDRESS, TwoWire *theWire = &Wire);
    uint16_t whoami();   /* 0xEACC, or 0 before begin() */
    tofFrame read();     /* steps the sweep and returns the grid. A zone takes 50 to 100 ms */
    void     sendNS();   /* steps the sweep and streams the grid to Neuro Studio */
  private:
    TwoWire *_wire = &Wire;
    uint8_t  _addr = VL53L1CB_DEVICE_ADDRESS;
};

#endif /* __cplusplus */

#endif /* VL53L1CB_ARDUINO_H */
