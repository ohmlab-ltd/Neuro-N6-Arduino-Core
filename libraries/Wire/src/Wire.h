/**
 * @file    Wire.h
 * @brief   Arduino TwoWire: I2C master only, 7-bit addresses, 32-byte buffers.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#ifndef NEURON6_WIRE_H
#define NEURON6_WIRE_H

#include <stdint.h>
#include <stddef.h>

#define WIRE_BUFFER_LENGTH    32u
#define WIRE_INTERFACES_COUNT 2

class TwoWire {
public:
  TwoWire();                     /* the on-board bus, same as TwoWire(0)        */
  explicit TwoWire(uint8_t bus); /* 0 = on-board bus, 1 = header                */

  void begin();                  /* Wire1 claims the header pins; Wire no-op    */
  void begin(uint8_t address);   /* slave mode: unsupported, does nothing       */
  void end();                    /* Wire1 releases the header pins; Wire no-op  */
  void setClock(uint32_t freqHz);/* no-op: both buses run at ~295 kHz           */

  void    beginTransmission(uint8_t address);
  uint8_t endTransmission(void);
  /* With sendStop false and one or two bytes written, the bytes are held and
   * returns 0: the next requestFrom() to the same address sends them as a
   * register address, then a repeated start and the read. */
  uint8_t endTransmission(uint8_t sendStop);

  uint8_t requestFrom(uint8_t address, uint8_t quantity);
  uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop);

  size_t write(uint8_t data);
  size_t write(const uint8_t *data, size_t len);

  int  available(void);
  int  read(void);
  int  peek(void);
  void flush(void);

private:
  bool start();                  /* Wire1 initialises on first use              */
  void lock();
  void unlock();
  uint8_t transmit(uint8_t address, const uint8_t *data, uint8_t len);
  void flushHeld();

  uint8_t _bus;

  uint8_t _txAddress;
  uint8_t _txBuffer[WIRE_BUFFER_LENGTH];
  uint8_t _txLength;
  uint8_t _transmitting;

  uint8_t _heldAddress;          /* register address held for a repeated start */
  uint8_t _heldBuffer[2];
  uint8_t _heldLength;           /* 0: nothing held                            */

  uint8_t _rxBuffer[WIRE_BUFFER_LENGTH];
  uint8_t _rxLength;
  uint8_t _rxIndex;
};

/* Wire   I2C1, SDA PE6, SCL PH9. The on-board sensor bus, started by the core.
 * Wire1  I2C2, SDA PD15, SCL PD14. The header pins, 4.7k pull-ups to 3.3 V. */
extern TwoWire Wire;
extern TwoWire Wire1;

#endif /* NEURON6_WIRE_H */
