/**
  ******************************************************************************
  * @file    OV5640_Arduino.c
  * @brief   Arduino wrapper for the OV5640 camera driver.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#include "OV5640_Arduino.h"
#include "ov5640.h"
#include "stm32n6xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

/* hi2c1 is shared across tasks: every transaction holds this lock. */
extern void n6_i2c_lock(void);
extern void n6_i2c_unlock(void);

#define OV5640_I2C_TIMEOUT_MS  100u

static OV5640_Object_t OV5640_Obj;

static int32_t OV5640_I2C_Init(void);
static int32_t OV5640_I2C_DeInit(void);
static int32_t OV5640_I2C_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
static int32_t OV5640_I2C_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
static int32_t OV5640_GetTick(void);

void ov5640_write(uint16_t reg, uint8_t val)
{
  uint8_t buf[3];
  buf[0] = (reg >> 8) & 0xFF;
  buf[1] = reg & 0xFF;
  buf[2] = val;

  n6_i2c_lock();
  HAL_I2C_Master_Transmit(&hi2c1, 0x78, buf, 3, OV5640_I2C_TIMEOUT_MS);
  n6_i2c_unlock();
}

uint8_t ov5640_read(uint16_t reg)
{
  uint8_t val = 0xFF;
  uint8_t buf[2];

  buf[0] = (reg >> 8) & 0xFF;
  buf[1] = reg & 0xFF;

  /* The register address write and the read share one lock. */
  n6_i2c_lock();
  HAL_I2C_Master_Transmit(&hi2c1, 0x78, buf, 2, OV5640_I2C_TIMEOUT_MS);
  HAL_I2C_Master_Receive(&hi2c1,  0x78, &val, 1, OV5640_I2C_TIMEOUT_MS);
  n6_i2c_unlock();

  return val;
}

int32_t ov5640_init(CameraResolution resolution, CameraMirrorFlip mirror_flip)
{
  int32_t ret = OV5640_OK;
  uint32_t camera_id;
  OV5640_IO_t ov5640_IO;

  ov5640_IO.Init      = OV5640_I2C_Init;
  ov5640_IO.DeInit    = OV5640_I2C_DeInit;
  ov5640_IO.Address   = 0x78;
  ov5640_IO.WriteReg  = OV5640_I2C_WriteReg;
  ov5640_IO.ReadReg   = OV5640_I2C_ReadReg;
  ov5640_IO.GetTick   = OV5640_GetTick;

  OV5640_Obj.Mode = SERIAL_MODE;
  OV5640_Obj.VirtualChannelID = 0;

  if (OV5640_RegisterBusIO(&OV5640_Obj, &ov5640_IO) != OV5640_OK)
  {
    return -1;
  }

  uint8_t reset_val = 0x82;
  ov5640_IO.WriteReg(0x78, 0x3008, &reset_val, 1);
  HAL_Delay(50);

  reset_val = 0x42;
  ov5640_IO.WriteReg(0x78, 0x3008, &reset_val, 1);
  HAL_Delay(10);

  if (OV5640_ReadID(&OV5640_Obj, &camera_id) != OV5640_OK)
  {
    return -2;
  }

  if (camera_id != 0x5640)
  {
    return -3;
  }

  ret = OV5640_Init(&OV5640_Obj, (uint32_t)resolution, OV5640_RGB565);
  if (ret != OV5640_OK)
  {
    return -4;
  }
  ret = OV5640_MirrorFlipConfig(&OV5640_Obj, (uint32_t)mirror_flip);
  if (ret != OV5640_OK)
  {
    return -5;
  }

  ret = OV5640_Start(&OV5640_Obj);
  if (ret != OV5640_OK)
  {
    return -6;
  }

  return 0;
}

void ov5640_stream_off(void)
{
  ov5640_write(0x3008, 0x42);
  HAL_Delay(20);
}

void ov5640_stream_on(void)
{
  ov5640_write(0x3008, 0x02);
  HAL_Delay(20);
}

static int32_t OV5640_I2C_Init(void)
{
  return OV5640_OK;
}

static int32_t OV5640_I2C_DeInit(void)
{
  return OV5640_OK;
}

static int32_t OV5640_I2C_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, OV5640_I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  return (st == HAL_OK) ? OV5640_OK : OV5640_ERROR;
}

static int32_t OV5640_I2C_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, OV5640_I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  return (st == HAL_OK) ? OV5640_OK : OV5640_ERROR;
}

static int32_t OV5640_GetTick(void)
{
  return (int32_t)HAL_GetTick();
}
