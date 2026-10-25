/**
 * @file    LSM6DSL_Arduino.h
 * @brief   LSM6DSL 6-axis IMU, accelerometer and gyroscope, on Arduino Wire.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#ifndef LSM6DSL_ARDUINO_H
#define LSM6DSL_ARDUINO_H

#ifdef __cplusplus

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

/* 7-bit I2C addresses. */
#define LSM6DSL_ADDR_LOW      0x6Au   /* SA0 = 0 */
#define LSM6DSL_ADDR_HIGH     0x6Bu   /* SA0 = 1 */
#define LSM6DSL_WHO_AM_I_VAL  0x6Au

typedef struct {
  float x;
  float y;
  float z;
} imuVec3;

class LSM6DSL {
  public:
    /* Sets accel and gyro to 104 Hz. Tries addr, then the other SA0 address,
     * and returns false if not found. */
    bool    begin(uint8_t addr = LSM6DSL_ADDR_LOW, TwoWire *theWire = &Wire);
    uint8_t whoami();
    imuVec3 readAccel();   /* g, full scale +/-2 g */
    imuVec3 readGyro();    /* dps, full scale 245 dps */
    float   readTemp();    /* degrees Celsius */

    /* Sends the current readings to Neuro Studio as META_IMU_FRAME. A call
     * made while the link is busy is dropped. */
    void    sendNS();

  private:
    TwoWire *_wire = &Wire;
    uint8_t  _addr = LSM6DSL_ADDR_LOW;
    bool     writeReg(uint8_t reg, uint8_t val);
    bool     readRegs(uint8_t reg, uint8_t *buf, uint8_t len);
};

#endif /* __cplusplus */
#endif /* LSM6DSL_ARDUINO_H */
