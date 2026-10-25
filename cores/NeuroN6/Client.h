/**
 ******************************************************************************
 * @file    Client.h
 * @brief   Arduino's Client interface: a byte stream to one network peer.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#pragma once

#include "api/Stream.h"
#include "IPAddress.h"

namespace arduino {

class Client : public Stream {
public:
  virtual int connect(IPAddress ip, uint16_t port) = 0;
  virtual int connect(const char *host, uint16_t port) = 0;
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t *buf, size_t size) = 0;
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int read(uint8_t *buf, size_t size) = 0;
  virtual int peek() = 0;
  virtual void flush() = 0;
  virtual void stop() = 0;
  virtual uint8_t connected() = 0;
  virtual operator bool() = 0;
};

} /* namespace arduino */

using arduino::Client;
