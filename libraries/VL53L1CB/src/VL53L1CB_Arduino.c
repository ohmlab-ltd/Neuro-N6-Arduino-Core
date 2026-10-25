/**
 ******************************************************************************
 * @file    VL53L1CB_Arduino.c
 * @brief   The 4x4 sweep: one zone per measurement, without blocking.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#include "VL53L1CB_Arduino.h"
#include "vl53l1cb_drv.h"
#include "stm32n6xx_hal.h"
#include "../../PostProcess/src/vnd_meta.h"
#include <string.h>

#define STATE_IDLE      0U
#define STATE_WAIT      1U
/* A measurement that is not ready by then is dropped. */
#define ZONE_TIMEOUT_MS (3U * VL53L1CB_FRAME_TIMING_BUDGET_MS + 50U)

static volatile uint8_t  s_initialized = 0U;
static volatile uint32_t s_state = STATE_IDLE;
static volatile uint32_t s_zone = 0U;
static uint32_t          s_started_ms = 0U;
static volatile uint16_t s_frame_mm[VL53L1CB_FRAME_ROWS][VL53L1CB_FRAME_COLS];

int32_t VL53L1CB_Setup4x4FrameCapture(void)
{
  s_initialized = 0U;
  s_state = STATE_IDLE;
  s_zone = 0U;
  memset((void *)s_frame_mm, 0, sizeof(s_frame_mm));
  if (vl53l1cb_drv_init() != 0) return VL53L1CB_ERROR;
  s_initialized = 1U;
  return VL53L1CB_OK;
}

static void finish_zone(uint16_t mm)
{
  s_frame_mm[s_zone / VL53L1CB_FRAME_COLS][s_zone % VL53L1CB_FRAME_COLS] = mm;
  s_zone = (s_zone + 1U) % VL53L1CB_FRAME_ZONE_COUNT;
  s_state = STATE_IDLE;
}

int32_t VL53L1CB_CaptureNextFrameZoneOnce(void)
{
  if (s_initialized == 0U) return VL53L1CB_ERROR;

  if (s_state == STATE_IDLE) {
    /* Zone (r, c) is the 4x4 SPAD block at columns 4c to 4c+3, counted from
     * the left, and rows 15-4r down to 12-4r, counted from the bottom. */
    const uint8_t r = (uint8_t)(s_zone / VL53L1CB_FRAME_COLS);
    const uint8_t c = (uint8_t)(s_zone % VL53L1CB_FRAME_COLS);
    if ((vl53l1cb_drv_set_roi((uint8_t)(14U - 4U * r), (uint8_t)(4U * c + 2U),
                              (uint8_t)VL53L1CB_FRAME_MIN_ROI_SPADS) != 0) ||
        (vl53l1cb_drv_start() != 0)) {
      finish_zone(0U);
      return VL53L1CB_ERROR;
    }
    s_started_ms = HAL_GetTick();
    s_state = STATE_WAIT;
    return VL53L1CB_OK;
  }

  const int32_t ready = vl53l1cb_drv_data_ready();
  if (ready == 1) {
    uint16_t mm = 0U;
    uint8_t valid = 0U;
    if (vl53l1cb_drv_read(&mm, &valid) != 0) valid = 0U;
    vl53l1cb_drv_stop();
    finish_zone(valid ? mm : 0U);
    return VL53L1CB_OK;
  }
  if ((ready < 0) || ((HAL_GetTick() - s_started_ms) > ZONE_TIMEOUT_MS)) {
    vl53l1cb_drv_stop();
    finish_zone(0U);
    return VL53L1CB_ERROR;
  }
  return VL53L1CB_OK;
}

void vnd_meta_build_tof(void)
{
  vnd_meta_builder_t meta;
  uint8_t payload[VL53L1CB_FRAME_ZONE_COUNT * sizeof(uint32_t)];
  uint32_t off = 0u;

  vnd_meta_builder_begin(&meta, VND_META_SLOT_TOF);
  for (uint32_t row = 0u; row < VL53L1CB_FRAME_ROWS; row++) {
    for (uint32_t col = 0u; col < VL53L1CB_FRAME_COLS; col++) {
      vnd_meta_put_u32_le(&payload[off], s_frame_mm[row][col]);
      off += sizeof(uint32_t);
    }
  }
  (void)vnd_meta_builder_put_tlv(&meta, META_TOF_FRAME_LEGACY, payload, (uint16_t)sizeof(payload));
  vnd_meta_builder_finish(&meta);
}

uint16_t VL53L1CB_WhoAmI(void)
{
  if (s_initialized == 0U) return 0u;
  return vl53l1cb_drv_model_id();
}

void VL53L1CB_GetFrameMm(uint16_t out[VL53L1CB_FRAME_ROWS][VL53L1CB_FRAME_COLS])
{
  if (out == NULL) return;
  for (uint32_t row = 0u; row < VL53L1CB_FRAME_ROWS; row++) {
    for (uint32_t col = 0u; col < VL53L1CB_FRAME_COLS; col++) {
      out[row][col] = s_frame_mm[row][col];
    }
  }
}
