/**
 ******************************************************************************
 * @file    WiFiServer.h
 * @brief   Arduino's WiFiServer: accepts TCP connections on a local port.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#pragma once

#include <Arduino.h>
#include "Server.h"
#include "WiFiClient.h"

class WiFiServer : public Server {
public:
  explicit WiFiServer(uint16_t port = 80);
  ~WiFiServer();

  void begin() override;
  void begin(uint16_t port);
  void end();
  void close() { end(); }

  WiFiClient available();
  WiFiClient accept();

  uint8_t status();            /* 1 while listening */
  explicit operator bool() { return status() != 0; }

  size_t write(uint8_t b) override;
  size_t write(const uint8_t *buf, size_t size) override;
  using Print::write;

  /* Connections this server tracks for available() and write(). */
  static const int kMaxClients = 4;

private:
  WiFiServer(const WiFiServer &) = delete;
  WiFiServer &operator=(const WiFiServer &) = delete;

  uint16_t   _port;
  int        _fd;
  WiFiClient _clients[kMaxClients];

  int  accept_fd();
  void take_new_clients();
};
