/**
 * @file    SPI.h
 * @brief   Arduino SPI master on SPI2, the header pins. Polling, software NSS.
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef NEURON6_SPI_H
#define NEURON6_SPI_H

#include <Arduino.h>

#define SPI_HAS_TRANSACTION 1

#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03

class SPISettings {
public:
  SPISettings() : clockFreq(4000000), bitOrder(MSBFIRST), dataMode(SPI_MODE0) {}
  SPISettings(uint32_t clock, uint8_t order, uint8_t mode)
    : clockFreq(clock), bitOrder(order), dataMode(mode) {}
  uint32_t clockFreq;
  uint8_t  bitOrder;
  uint8_t  dataMode;
};

class SPIClass {
public:
  void begin();
  void end();

  void beginTransaction(SPISettings settings);
  void endTransaction();

  /* Sets SPI2's kernel clock to an RCC_SPI2CLKSOURCE_* value, after begin().
   * SCK is at least 781 kHz on PCLK1, and 250 kHz to 32 MHz on HSI. */
  void setKernelSource(uint32_t rccSpi2Source);

  /* Forces the SPI mode, overriding beginTransaction(). -1 turns it off. */
  void forceDataMode(int mode);

  uint8_t  transfer(uint8_t data);
  uint16_t transfer16(uint16_t data);
  void     transfer(void *buf, size_t count);          /* in place */
  void     transfer(const void *tx, void *rx, size_t count);

  void setBitOrder(uint8_t order);
  void setDataMode(uint8_t mode);
  void setClockDivider(uint8_t div);                   /* divider of the kernel clock */

  /* No SPI interrupts on this core. */
  void usingInterrupt(int n) { (void)n; }
  void notUsingInterrupt(int n) { (void)n; }
  void attachInterrupt() { }
  void detachInterrupt() { }

private:
  void reconfigure();
  void txnOpen();
  void txnClose();
  bool        _txnOpen = false;
  SPISettings _settings;
  bool        _began = false;
  bool        _dirty = true;
  int         _forcedMode = -1;
};

extern SPIClass SPI;

#endif /* NEURON6_SPI_H */
