/**
 * @file    LSM6DSL_Arduino.cpp
 * @brief   LSM6DSL Arduino Wire driver.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include <LSM6DSL_Arduino.h>
#include "NeuroN6_FrameQueue.h"
#include "../../PostProcess/src/vnd_meta.h"
#include <string.h>

#define REG_WHO_AM_I    0x0Fu
#define REG_CTRL1_XL    0x10u   /* accel: ODR + full-scale */
#define REG_CTRL2_G     0x11u   /* gyro:  ODR + full-scale */
#define REG_CTRL3_C     0x12u   /* BDU + auto-increment    */
#define REG_OUT_TEMP_L  0x20u   /* temperature, 2 bytes    */
#define REG_OUTX_L_G    0x22u   /* gyro  X/Y/Z, 6 bytes    */
#define REG_OUTX_L_XL   0x28u   /* accel X/Y/Z, 6 bytes    */

#define ACC_SENS_G      0.000061f   /* g/LSB at +/-2 g */
#define GYR_SENS_DPS    0.00875f    /* dps/LSB at 245 dps */

bool LSM6DSL::begin(uint8_t addr, TwoWire *theWire)
{
  _wire = theWire;
  _addr = addr;

  if (whoami() != LSM6DSL_WHO_AM_I_VAL) {
    _addr = (addr == LSM6DSL_ADDR_LOW) ? LSM6DSL_ADDR_HIGH : LSM6DSL_ADDR_LOW;
    if (whoami() != LSM6DSL_WHO_AM_I_VAL) return false;
  }

  writeReg(REG_CTRL3_C,  0x44u);   /* BDU=1, IF_INC=1 */
  writeReg(REG_CTRL1_XL, 0x40u);   /* accel: 104 Hz, +/-2 g */
  writeReg(REG_CTRL2_G,  0x40u);   /* gyro:  104 Hz, 245 dps */
  return true;
}

uint8_t LSM6DSL::whoami()
{
  uint8_t v = 0u;
  if (readRegs(REG_WHO_AM_I, &v, 1u)) return v;
  return 0u;
}

imuVec3 LSM6DSL::readAccel()
{
  imuVec3 a = {0.0f, 0.0f, 0.0f};
  uint8_t b[6];
  if (readRegs(REG_OUTX_L_XL, b, 6u)) {
    a.x = (float)(int16_t)((uint16_t)b[1] << 8 | b[0]) * ACC_SENS_G;
    a.y = (float)(int16_t)((uint16_t)b[3] << 8 | b[2]) * ACC_SENS_G;
    a.z = (float)(int16_t)((uint16_t)b[5] << 8 | b[4]) * ACC_SENS_G;
  }
  return a;
}

imuVec3 LSM6DSL::readGyro()
{
  imuVec3 g = {0.0f, 0.0f, 0.0f};
  uint8_t b[6];
  if (readRegs(REG_OUTX_L_G, b, 6u)) {
    g.x = (float)(int16_t)((uint16_t)b[1] << 8 | b[0]) * GYR_SENS_DPS;
    g.y = (float)(int16_t)((uint16_t)b[3] << 8 | b[2]) * GYR_SENS_DPS;
    g.z = (float)(int16_t)((uint16_t)b[5] << 8 | b[4]) * GYR_SENS_DPS;
  }
  return g;
}

float LSM6DSL::readTemp()
{
  uint8_t b[2];
  if (readRegs(REG_OUT_TEMP_L, b, 2u)) {
    int16_t raw = (int16_t)((uint16_t)b[1] << 8 | b[0]);
    return 25.0f + (float)raw / 256.0f;
  }
  return 0.0f;
}

bool LSM6DSL::writeReg(uint8_t reg, uint8_t val)
{
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(val);
  return (_wire->endTransmission() == 0);
}

bool LSM6DSL::readRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission() != 0) return false;
  if (_wire->requestFrom(_addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)_wire->read();
  return true;
}

/* META_IMU_FRAME: 7 x float32 LE: accel x, y, z in g, gyro x, y, z in dps,
 * degC. Sent when the sketch includes <PostProcess.h>. */
void LSM6DSL::sendNS()
{
  NeuroN6_StreamUsbOnlyStart();   /* idempotent */

  imuVec3 a = readAccel();
  imuVec3 g = readGyro();
  float   t = readTemp();
  float   v[7] = { a.x, a.y, a.z, g.x, g.y, g.z, t };

  (void)vnd_meta_publish_tlv(VND_META_SLOT_IMU, (uint8_t)META_IMU_FRAME, v, (uint16_t)sizeof(v));
}
