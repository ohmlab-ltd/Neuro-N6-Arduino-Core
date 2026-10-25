/**
 ******************************************************************************
 * @file    HardwareSerial.h
 * @brief   Arduino `Serial` over USB-CDC.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#ifndef NEURON6_HARDWARESERIAL_H
#define NEURON6_HARDWARESERIAL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus

#include "api/Stream.h"

#ifndef DEC
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#endif

class HardwareSerial : public arduino::Stream {
public:
  void begin(unsigned long baud = 115200) { (void)baud; }
  void begin(unsigned long baud, uint16_t config) { (void)baud; (void)config; }
  void end() { }
  operator bool();                 /* true once the host asserted DTR */

  /* Stream */
  int  available() override;
  int  read() override;
  int  peek() override;

  /* Print */
  size_t write(uint8_t b) override;
  size_t write(const uint8_t *buf, size_t len) override;
  using arduino::Print::write;
  void   flush() override;
  int    availableForWrite() override;
};

extern HardwareSerial Serial;

#endif /* __cplusplus */

/* Called from the core USB task for every CDC byte received. */
#ifdef __cplusplus
extern "C" {
#endif
void NeuroN6_SerialRxSink(uint8_t ch);
#ifdef __cplusplus
}
#endif

#endif /* NEURON6_HARDWARESERIAL_H */
