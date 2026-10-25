/* MMC5603NJ driver.
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include "MMC5603NJ.h"
#include "NeuroN6_FrameQueue.h"
#include "../../PostProcess/src/vnd_meta.h"

/* 20-bit output, one count is 0.0625 mG, so 0.00625 microtesla. */
static const float kMicroteslaPerCount = 0.00625f;
static const int32_t kZeroField = 1L << 19;

bool MMC5603NJ::write(uint8_t reg, const uint8_t *wbuf, uint8_t len) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (len) _wire->write(wbuf, len);
  return _wire->endTransmission() == 0;
}

bool MMC5603NJ::writeByte(uint8_t reg, uint8_t value) {
  return write(reg, &value, 1);
}

bool MMC5603NJ::readRegister(uint8_t reg, uint8_t *rbuf, uint8_t len) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) return false;
  if (_wire->requestFrom((uint8_t)_addr, (uint8_t)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) rbuf[i] = (uint8_t)_wire->read();
  return true;
}

bool MMC5603NJ::waitStatus(uint8_t bit, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  do {
    uint8_t st = 0;
    if (readRegister(MMC5603NJ_REG_STATUS1, &st) && (st & (1u << bit))) return true;
    delay(1);
  } while (millis() - t0 < timeoutMs);
  return false;
}

bool MMC5603NJ::begin(uint8_t addr, TwoWire *theWire) {
  _addr = addr;
  _wire = theWire;
  _continuous = false;
  _running = false;
  if (whoami() != MMC5603NJ_WHO_AM_I) return false;
  writeByte(MMC5603NJ_REG_CONF1, MMC5603NJ_REG_CONF1_SWRESET);
  delay(20);                                   /* 20 ms after a software reset */
  writeByte(MMC5603NJ_REG_CONF1, MMC5603NJ_REG_CONF1_BW66);
  writeByte(MMC5603NJ_REG_CONF0, MMC5603NJ_REG_CONF0_SET);
  delay(1);
  return true;
}

uint8_t MMC5603NJ::whoami() {
  uint8_t id = 0;
  return readRegister(MMC5603NJ_REG_ID, &id) ? id : 0;
}

void MMC5603NJ::setContinuousMode(uint8_t odr) {
  if (odr == 0) {
    writeByte(MMC5603NJ_REG_ODR, 0);
    writeByte(MMC5603NJ_REG_CONF2, 0);
    _continuous = false;
    _running = false;
    return;
  }
  _continuous = true;
  const bool odr_set = writeByte(MMC5603NJ_REG_ODR, odr);
  _running = startContinuous() && odr_set;
  uint32_t settle = 1000u / odr + 10u;         /* until the first compensated sample */
  delay(settle < 100u ? settle : 100u);
}

bool MMC5603NJ::startContinuous() {
  return writeByte(MMC5603NJ_REG_CONF0, MMC5603NJ_REG_CONF0_FREQ | MMC5603NJ_REG_CONF0_SR) &&
         writeByte(MMC5603NJ_REG_CONF2, MMC5603NJ_REG_CONF2_CMMEN);
}

magSensorXYZ MMC5603NJ::getRawMag() {
  magSensorXYZ m = {0, 0, 0};
  /* One measurement on demand, also when a failed write has left free-running
   * stopped: the output registers hold the last sample. */
  if (!_running) {
    writeByte(MMC5603NJ_REG_CONF0, MMC5603NJ_REG_CONF0_MEASM | MMC5603NJ_REG_CONF0_SR);
    if (!waitStatus(MMC5603NJ_REG_STATUS_MEASM, 20)) return m;
  }
  uint8_t b[9];
  if (!readRegister(MMC5603NJ_REG_XOUT0, b, sizeof b)) return m;
  m.x = (int32_t)(((uint32_t)b[0] << 12) | ((uint32_t)b[1] << 4) | (b[6] >> 4)) - kZeroField;
  m.y = (int32_t)(((uint32_t)b[2] << 12) | ((uint32_t)b[3] << 4) | (b[7] >> 4)) - kZeroField;
  m.z = (int32_t)(((uint32_t)b[4] << 12) | ((uint32_t)b[5] << 4) | (b[8] >> 4)) - kZeroField;
  return m;
}

magSensorFloatXYZ MMC5603NJ::getMag() {
  magSensorXYZ r = getRawMag();
  magSensorFloatXYZ f;
  f.x = r.x * kMicroteslaPerCount;
  f.y = r.y * kMicroteslaPerCount;
  f.z = r.z * kMicroteslaPerCount;
  return f;
}

uint8_t MMC5603NJ::getRawTemp() {
  bool resume = _continuous;
  if (resume) {                                 /* the chip only measures temperature when idle */
    writeByte(MMC5603NJ_REG_CONF2, 0);
    _running = false;
    delay(1);
  }
  writeByte(MMC5603NJ_REG_CONF0, MMC5603NJ_REG_CONF0_MEAST);
  uint8_t t = 0;
  if (waitStatus(MMC5603NJ_REG_STATUS_MEAST, 20)) readRegister(MMC5603NJ_REG_TOUT, &t);
  if (resume) _running = startContinuous();
  return t;
}

float MMC5603NJ::getTemp() {
  return getRawTemp() * 0.8f - 75.0f;         /* 0.8 degC per count from -75 */
}

void MMC5603NJ::sendNS() {
  NeuroN6_StreamUsbOnlyStart();   /* idempotent */
  magSensorFloatXYZ m = getMag();
  float v[4] = { m.x, m.y, m.z, getTemp() };
  vnd_meta_publish_tlv(VND_META_SLOT_MAG, META_MAG_FRAME, v, sizeof v);
}
