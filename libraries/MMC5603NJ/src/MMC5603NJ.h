/*
  MMC5603NJ: MEMSIC 3-axis magnetometer over I2C.

  Copyright (c) 2026 Ohm Lab Ltd.
  SPDX-License-Identifier: MIT
*/
#ifndef MMC5603NJ_H
#define MMC5603NJ_H

#include <Arduino.h>
#include <Wire.h>

#define MMC5603NJ_DEFAULT_ADDRESS   0x30
#define MMC5603NJ_WHO_AM_I          0x10

/* Registers */
#define MMC5603NJ_REG_XOUT0         0x00
#define MMC5603NJ_REG_XOUT1         0x01
#define MMC5603NJ_REG_YOUT0         0x02
#define MMC5603NJ_REG_YOUT1         0x03
#define MMC5603NJ_REG_ZOUT0         0x04
#define MMC5603NJ_REG_ZOUT1         0x05
#define MMC5603NJ_REG_XOUT2         0x06
#define MMC5603NJ_REG_YOUT2         0x07
#define MMC5603NJ_REG_ZOUT2         0x08
#define MMC5603NJ_REG_TOUT          0x09
#define MMC5603NJ_REG_STATUS1       0x18
#define MMC5603NJ_REG_ODR           0x1A
#define MMC5603NJ_REG_CONF0         0x1B
#define MMC5603NJ_REG_CONF1         0x1C
#define MMC5603NJ_REG_CONF2         0x1D
#define MMC5603NJ_REG_ID            0x39

/* Control 0 */
#define MMC5603NJ_REG_CONF0_MEASM   0x01   /* start one field measurement */
#define MMC5603NJ_REG_CONF0_MEAST   0x02   /* start one temperature measurement */
#define MMC5603NJ_REG_CONF0_SET     0x08   /* SET pulse: magnetise the sensor */
#define MMC5603NJ_REG_CONF0_RESET   0x10   /* RESET pulse */
#define MMC5603NJ_REG_CONF0_SR      0x20   /* automatic set/reset around each measurement */
#define MMC5603NJ_REG_CONF0_FREQ    0x80   /* enable the continuous-mode clock */
/* Control 1 */
#define MMC5603NJ_REG_CONF1_BW66    0x00   /* 6.6 ms per measurement, lowest noise */
#define MMC5603NJ_REG_CONF1_BW35    0x01
#define MMC5603NJ_REG_CONF1_BW20    0x02
#define MMC5603NJ_REG_CONF1_BW12    0x03
#define MMC5603NJ_REG_CONF1_SWRESET 0x80
/* Control 2 */
#define MMC5603NJ_REG_CONF2_CMMEN   0x10   /* continuous measurement mode */
/* Status 1 bit numbers */
#define MMC5603NJ_REG_STATUS_MEASM  6
#define MMC5603NJ_REG_STATUS_MEAST  7

typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
} magSensorXYZ;          /* 20-bit counts with the zero-field offset removed */

typedef struct {
  float x;
  float y;
  float z;
} magSensorFloatXYZ;     /* microtesla */

class MMC5603NJ {
public:
  /* True when the sensor answers with its product ID. Does not start Wire. */
  bool begin(uint8_t addr = MMC5603NJ_DEFAULT_ADDRESS, TwoWire *theWire = &Wire);
  uint8_t whoami();
  /* Samples per second, 1 to 255; 0 returns to single-shot mode. */
  void setContinuousMode(uint8_t odr);
  magSensorXYZ getRawMag();
  magSensorFloatXYZ getMag();
  uint8_t getRawTemp();
  float getTemp();   /* degrees Celsius */
  /* Publishes field and temperature to Neuro Studio as META_MAG_FRAME. */
  void sendNS();

private:
  TwoWire *_wire = &Wire;
  uint8_t  _addr = MMC5603NJ_DEFAULT_ADDRESS;
  bool     _continuous = false;   /* free-running asked for */
  bool     _running = false;      /* free-running, as the last writes left it */
  bool write(uint8_t reg, const uint8_t *wbuf, uint8_t len);
  bool writeByte(uint8_t reg, uint8_t value);
  bool startContinuous();
  bool readRegister(uint8_t reg, uint8_t *rbuf, uint8_t len = 1);
  bool waitStatus(uint8_t bit, uint32_t timeoutMs);
};

#endif
