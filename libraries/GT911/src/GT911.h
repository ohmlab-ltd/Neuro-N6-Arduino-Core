/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* GT911: Goodix capacitive touch controller for the Neuro N6 TFT carrier. */

#ifndef GT911_H
#define GT911_H

#include <Arduino.h>
#include <Wire.h>

/* Strapped from the INT level at reset: low 0x5D, high 0x14. RESET is tied to
 * the system reset. */
#define GT911_ADDR_PRIMARY    0x5D
#define GT911_ADDR_SECONDARY  0x14

/* Registers, 16-bit addressed, big-endian on the wire. */
#define GT911_REG_X_MAX       0x8048   /* configured X resolution, 16-bit LE */
#define GT911_REG_CHIP_ID     0x8140   /* 4 ASCII bytes, "911" for this part */
#define GT911_REG_STATUS      0x814E   /* bit7 = data ready, bits3:0 = count  */
#define GT911_REG_POINT1      0x814F   /* first of up to 5 x 8-byte records   */

#define GT911_MAX_POINTS      5
#define GT911_POINT_BYTES     8

#define GT911_INT_PORT        GPIOD
#define GT911_INT_PIN         GPIO_PIN_9

class GT911 {
public:
  GT911() {}

  /* Call after Wire.begin(). False if neither address answers as a GT911. */
  bool begin();

  /* Returns the number of touch points, 0 to 5. With no fresh sample it
   * returns the same points again. */
  int read();

  /* Points from the last read(), in panel coordinates. */
  int count() const { return _n; }
  bool touched() const { return _n > 0; }
  int  x(int i = 0) const    { return (i >= 0 && i < _n) ? _pt[i].x : -1; }
  int  y(int i = 0) const    { return (i >= 0 && i < _n) ? _pt[i].y : -1; }
  int  size(int i = 0) const { return (i >= 0 && i < _n) ? _pt[i].size : 0; }
  int  id(int i = 0) const   { return (i >= 0 && i < _n) ? _pt[i].id : -1; }

  /* True if any current touch point falls inside the rectangle. */
  bool pressed(int rx, int ry, int rw, int rh) const;

  /* True only after the read() that first reports a touch starting inside the
   * rectangle. */
  bool clicked(int rx, int ry, int rw, int rh) const;

  /* Overrides the mapping begin() picks. w and h are the panel size, and
   * swap_xy is applied before the flips. */
  void setMapping(int w, int h, bool swap_xy, bool flip_x, bool flip_y);

  /* The resolution the controller is configured for, or 0 if unread. */
  int controllerWidth() const  { return _ctrl_w; }
  int controllerHeight() const { return _ctrl_h; }

  /* The address that answered, or 0 before a successful begin(). */
  uint8_t address() const { return _addr; }

  /* Chip id string read at begin(). Empty if begin() failed. */
  const char *chipId() const { return _chip_id; }

  /* INT line level. read() does not use it. */
  bool intAsserted() const;

  /* The 8-byte record of point i: id, x LE, y LE, size LE, reserved. */
  bool raw(int i, uint8_t out[GT911_POINT_BYTES]) const;

private:
  struct Point { int x, y, size, id; };

  bool readReg(uint16_t reg, uint8_t *buf, uint8_t len) const;
  bool writeReg(uint16_t reg, uint8_t val) const;
  bool probe(uint8_t addr);
  void defaultMapping();

  uint8_t _addr = 0;
  char    _chip_id[5] = { 0 };

  int   _n = 0;
  Point _pt[GT911_MAX_POINTS] = {};
  uint8_t _rawpt[GT911_MAX_POINTS][GT911_POINT_BYTES] = {};

  int  _panel_w = 800, _panel_h = 480;
  bool _swap_xy = false, _flip_x = false, _flip_y = false;
  bool _mapping_set = false;
  int  _ctrl_w = 0, _ctrl_h = 0;

  /* clicked() edge state. */
  bool _was_touching = false;
};

#endif /* GT911_H */
