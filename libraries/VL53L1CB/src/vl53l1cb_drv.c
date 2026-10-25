/**
 ******************************************************************************
 * @file    vl53l1cb_drv.c
 * @brief   VL53L1CB register access and single-zone ranging.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#include "vl53l1cb_drv.h"
#include "vl53l1cb_st_tables.h"
#include "stm32n6xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern void n6_i2c_lock(void);
extern void n6_i2c_unlock(void);

/* Registers, 16-bit addresses, big-endian data. */
#define REG_SOFT_RESET              0x0000u
#define REG_VHV_LOOP_BOUND          0x0008u
#define REG_VHV_INIT                0x000Bu
#define REG_GPIO_HV_MUX_CTRL        0x0030u
#define REG_GPIO_TIO_HV_STATUS      0x0031u
#define REG_SIGMA_THRESH            0x0064u
#define REG_MIN_COUNT_RATE          0x0066u
#define REG_ROI_CENTRE_SPAD         0x007Fu
#define REG_ROI_XY_SIZE             0x0080u
#define REG_INTERRUPT_CLEAR         0x0086u
#define REG_MODE_START              0x0087u
#define REG_RANGE_STATUS            0x0089u
#define REG_RANGE_MM                0x0096u
#define REG_FIRMWARE_STATUS         0x00E5u
#define REG_MODEL_ID                0x010Fu

#define MODE_STOP                   0x00u
#define MODE_TIMED                  0x40u
#define RANGE_STATUS_GOOD           0x09u

#define I2C_TIMEOUT_MS              20u

static int32_t wr(uint16_t reg, const uint8_t *buf, uint16_t n)
{
  n6_i2c_lock();
  const HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, VL53L1CB_DRV_I2C_ADDR, reg,
                                                 I2C_MEMADD_SIZE_16BIT, (uint8_t *)buf, n,
                                                 I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  return (st == HAL_OK) ? 0 : -1;
}

static int32_t rd(uint16_t reg, uint8_t *buf, uint16_t n)
{
  n6_i2c_lock();
  const HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, VL53L1CB_DRV_I2C_ADDR, reg,
                                                I2C_MEMADD_SIZE_16BIT, buf, n, I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  return (st == HAL_OK) ? 0 : -1;
}

static int32_t wr8(uint16_t reg, uint8_t v) { return wr(reg, &v, 1); }

static int32_t rd8(uint16_t reg, uint8_t *v) { return rd(reg, v, 1); }

uint16_t vl53l1cb_drv_model_id(void)
{
  uint8_t b[2];
  if (rd(REG_MODEL_ID, b, 2) != 0) return 0u;
  return (uint16_t)((b[0] << 8) | b[1]);
}

int32_t vl53l1cb_drv_data_ready(void)
{
  uint8_t mux, tio;
  if (rd8(REG_GPIO_HV_MUX_CTRL, &mux) != 0) return -1;
  if (rd8(REG_GPIO_TIO_HV_STATUS, &tio) != 0) return -1;
  const uint8_t active_level = (mux & 0x10u) ? 0u : 1u;   /* bit 4 set: active low */
  return ((tio & 1u) == active_level) ? 1 : 0;
}

static int32_t wait_ready(uint32_t timeout_ms)
{
  const uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < timeout_ms) {
    const int32_t r = vl53l1cb_drv_data_ready();
    if (r != 0) return r;
    HAL_Delay(1);
  }
  return -2;
}

int32_t vl53l1cb_drv_init(void)
{
  if (vl53l1cb_drv_model_id() != VL53L1CB_DRV_MODEL_ID) return -1;

  /* A soft reset clears any configuration another driver left, such as
   * histogram ranging, which this one cannot run. */
  if (wr8(REG_SOFT_RESET, 0x00u) != 0) return -2;
  HAL_Delay(2);
  if (wr8(REG_SOFT_RESET, 0x01u) != 0) return -2;
  HAL_Delay(2);
  uint32_t t0 = HAL_GetTick();
  uint8_t fw = 0u;
  while (((rd8(REG_FIRMWARE_STATUS, &fw) != 0) || ((fw & 1u) == 0u)) &&
         ((HAL_GetTick() - t0) < 500u)) {
    HAL_Delay(2);
  }
  if ((fw & 1u) == 0u) return -3;

  if (wr(VL53L1_ST_CONFIG_FIRST_REG, vl53l1_st_default_config,
         (uint16_t)sizeof(vl53l1_st_default_config)) != 0) return -4;

  /* One timed measurement calibrates VHV; later ones start from it. */
  (void)wr8(REG_INTERRUPT_CLEAR, 0x01u);
  if (wr8(REG_MODE_START, MODE_TIMED) != 0) return -5;
  const int32_t r = wait_ready(300u);
  (void)wr8(REG_INTERRUPT_CLEAR, 0x01u);
  (void)wr8(REG_MODE_START, MODE_STOP);
  if (r != 1) return -6;
  (void)wr8(REG_VHV_LOOP_BOUND, 0x09u);
  (void)wr8(REG_VHV_INIT, 0x00u);

  /* A zone sees 16 of the 256 SPADs: 0.6 to 1.1 MCPS from a target at 0.75 m,
   * under the configuration's 1.0 MCPS minimum. Zones take 0.125 MCPS (9.7)
   * and ST's 15 mm sigma limit (14.2). */
  static const uint8_t sigma[2] = { 0x00u, 0x3Cu };
  static const uint8_t rate[2] = { 0x00u, 0x10u };
  if ((wr(REG_SIGMA_THRESH, sigma, 2) != 0) || (wr(REG_MIN_COUNT_RATE, rate, 2) != 0)) return -7;

  /* The configuration leaves long distance mode, 100 ms timing budget. */
  return 0;
}

int32_t vl53l1cb_drv_set_roi(uint8_t row, uint8_t col, uint8_t size)
{
  if ((row > 15u) || (col > 15u) || (size < 4u) || (size > 16u)) return -1;
  /* SPAD numbering of the 16x16 array. */
  const uint8_t spad = (row > 7u) ? (uint8_t)(128u + (col << 3) + (15u - row))
                                  : (uint8_t)(((15u - col) << 3) + row);
  const uint8_t xy = (uint8_t)(((size - 1u) << 4) | (size - 1u));
  if (wr8(REG_ROI_CENTRE_SPAD, spad) != 0) return -2;
  if (wr8(REG_ROI_XY_SIZE, xy) != 0) return -2;
  return 0;
}

int32_t vl53l1cb_drv_start(void)
{
  if (wr8(REG_INTERRUPT_CLEAR, 0x01u) != 0) return -1;
  return (wr8(REG_MODE_START, MODE_TIMED) == 0) ? 0 : -1;
}

int32_t vl53l1cb_drv_read(uint16_t *mm, uint8_t *valid)
{
  uint8_t st = 0u, b[2] = { 0u, 0u };
  if ((rd8(REG_RANGE_STATUS, &st) != 0) || (rd(REG_RANGE_MM, b, 2) != 0)) return -1;
  (void)wr8(REG_INTERRUPT_CLEAR, 0x01u);
  if (mm) *mm = (uint16_t)((b[0] << 8) | b[1]);
  if (valid) *valid = ((st & 0x1Fu) == RANGE_STATUS_GOOD) ? 1u : 0u;
  return 0;
}

void vl53l1cb_drv_stop(void)
{
  (void)wr8(REG_INTERRUPT_CLEAR, 0x01u);
  (void)wr8(REG_MODE_START, MODE_STOP);
}
