/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* GT911: Goodix capacitive touch controller. */

#include "GT911.h"

bool GT911::readReg(uint16_t reg, uint8_t *buf, uint8_t len) const
{
  if (_addr == 0 || buf == nullptr || len == 0) return false;
  /* Wire's buffer is 32 bytes. */
  if (len > 32) return false;

  Wire.beginTransmission(_addr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(_addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

bool GT911::writeReg(uint16_t reg, uint8_t val) const
{
  if (_addr == 0) return false;
  Wire.beginTransmission(_addr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool GT911::probe(uint8_t addr)
{
  _addr = addr;                       /* readReg needs it set */
  uint8_t id[4] = { 0 };
  if (!readReg(GT911_REG_CHIP_ID, id, 4)) { _addr = 0; return false; }

  if (id[0] != '9' || id[1] != '1' || id[2] != '1') { _addr = 0; return false; }

  _chip_id[0] = (char)id[0];
  _chip_id[1] = (char)id[1];
  _chip_id[2] = (char)id[2];
  _chip_id[3] = (id[3] >= 32 && id[3] < 127) ? (char)id[3] : '\0';
  _chip_id[4] = '\0';
  return true;
}

bool GT911::begin()
{
  __HAL_RCC_GPIOD_CLK_ENABLE();
  GPIO_InitTypeDef gi = {};
  gi.Pin  = GT911_INT_PIN;
  gi.Mode = GPIO_MODE_INPUT;
  gi.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GT911_INT_PORT, &gi);

  if (!probe(GT911_ADDR_PRIMARY) && !probe(GT911_ADDR_SECONDARY)) return false;
  defaultMapping();
  return true;
}

/* A controller configured for 480 x 800 reports swapped axes, both reversed. */
void GT911::defaultMapping()
{
  uint8_t r[4];
  if (!readReg(GT911_REG_X_MAX, r, 4)) return;
  _ctrl_w = (int)((uint16_t)r[0] | ((uint16_t)r[1] << 8));
  _ctrl_h = (int)((uint16_t)r[2] | ((uint16_t)r[3] << 8));
  if (_mapping_set) return;
  const bool portrait = (_ctrl_w < _ctrl_h) && (_panel_w > _panel_h);
  _swap_xy = portrait;
  _flip_x  = portrait;
  _flip_y  = portrait;
}

bool GT911::intAsserted() const
{
  return HAL_GPIO_ReadPin(GT911_INT_PORT, GT911_INT_PIN) == GPIO_PIN_SET;
}

int GT911::read()
{
  if (_addr == 0) { _n = 0; return 0; }

  uint8_t st = 0;
  /* Bus error or no fresh sample: keep the previous points. */
  if (!readReg(GT911_REG_STATUS, &st, 1) || (st & 0x80) == 0) {
    _was_touching = (_n > 0);
    return _n;
  }

  _was_touching = (_n > 0);
  _n = 0;

  int n = st & 0x0F;
  if (n > GT911_MAX_POINTS) n = GT911_MAX_POINTS;

  for (int i = 0; i < n; i++) {
    uint8_t *b = _rawpt[i];
    if (!readReg((uint16_t)(GT911_REG_POINT1 + i * GT911_POINT_BYTES),
                 b, GT911_POINT_BYTES)) {
      n = i;
      break;
    }

    /* [0] track id, [1..2] x LE, [3..4] y LE, [5..6] size LE, [7] reserved */
    const int rid = b[0];
    int rx = (int)((uint16_t)b[1] | ((uint16_t)b[2] << 8));
    int ry = (int)((uint16_t)b[3] | ((uint16_t)b[4] << 8));
    const int rs = (int)((uint16_t)b[5] | ((uint16_t)b[6] << 8));

    if (_swap_xy) { const int t = rx; rx = ry; ry = t; }
    if (_flip_x)  rx = (_panel_w - 1) - rx;
    if (_flip_y)  ry = (_panel_h - 1) - ry;

    if (rx < 0) rx = 0; else if (rx >= _panel_w) rx = _panel_w - 1;
    if (ry < 0) ry = 0; else if (ry >= _panel_h) ry = _panel_h - 1;

    _pt[i].x = rx; _pt[i].y = ry; _pt[i].size = rs; _pt[i].id = rid;
  }

  /* Acknowledge, zero points included, or the controller stops sampling. */
  (void)writeReg(GT911_REG_STATUS, 0x00);

  _n = n;
  return _n;
}

bool GT911::pressed(int rx, int ry, int rw, int rh) const
{
  for (int i = 0; i < _n; i++) {
    if (_pt[i].x >= rx && _pt[i].x < rx + rw &&
        _pt[i].y >= ry && _pt[i].y < ry + rh) {
      return true;
    }
  }
  return false;
}

bool GT911::clicked(int rx, int ry, int rw, int rh) const
{
  return pressed(rx, ry, rw, rh) && !_was_touching;
}

void GT911::setMapping(int w, int h, bool swap_xy, bool flip_x, bool flip_y)
{
  if (w > 0) _panel_w = w;
  if (h > 0) _panel_h = h;
  _swap_xy = swap_xy;
  _flip_x  = flip_x;
  _flip_y  = flip_y;
  _mapping_set = true;
}

bool GT911::raw(int i, uint8_t out[GT911_POINT_BYTES]) const
{
  if (i < 0 || i >= _n || out == nullptr) return false;
  for (int k = 0; k < GT911_POINT_BYTES; k++) out[k] = _rawpt[i][k];
  return true;
}
