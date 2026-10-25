/**
 ******************************************************************************
 * @file    Udp.h
 * @brief   Arduino's UDP interface: datagrams, one packet at a time.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#pragma once

#include "api/Stream.h"
#include "IPAddress.h"

namespace arduino {

class UDP : public Stream {
public:
  /* Listens on a local port. Returns 1, or 0 if no socket is available. */
  virtual uint8_t begin(uint16_t port) = 0;
  virtual uint8_t beginMulticast(IPAddress, uint16_t) { return 0; }
  virtual void stop() = 0;

  /* beginPacket() and endPacket() return 1 on success and 0 on failure. */
  virtual int beginPacket(IPAddress ip, uint16_t port) = 0;
  virtual int beginPacket(const char *host, uint16_t port) = 0;
  virtual int endPacket() = 0;
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t *buffer, size_t size) = 0;

  /* Takes the next datagram and returns its size, 0 if none. */
  virtual int parsePacket() = 0;
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int read(unsigned char *buffer, size_t len) = 0;
  virtual int read(char *buffer, size_t len) = 0;
  virtual int peek() = 0;
  virtual void flush() = 0;

  /* Sender of the datagram parsePacket() took. */
  virtual IPAddress remoteIP() = 0;
  virtual uint16_t remotePort() = 0;
};

} /* namespace arduino */

using arduino::UDP;
