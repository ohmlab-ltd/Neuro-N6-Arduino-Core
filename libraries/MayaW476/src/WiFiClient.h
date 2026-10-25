/**
 ******************************************************************************
 * @file    WiFiClient.h
 * @brief   Arduino's WiFiClient: one TCP connection over the board's WiFi.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#pragma once

#include <Arduino.h>
#include "Client.h"
#include "IPAddress.h"

/* Copies share the connection, which closes on stop() or with the last copy. */
class WiFiClient : public Client {
public:
  WiFiClient();
  explicit WiFiClient(int fd);                 /* adopts a connected socket */
  WiFiClient(const WiFiClient &other);
  WiFiClient &operator=(const WiFiClient &other);
  ~WiFiClient();

  int connect(IPAddress ip, uint16_t port) override;
  int connect(const char *host, uint16_t port) override;

  /* Waits while the connection is full, up to the Stream timeout per stall. */
  size_t write(uint8_t b) override;
  size_t write(const uint8_t *buf, size_t size) override;
  using Print::write;

  int available() override;
  int read() override;                         /* -1 if none */
  int read(uint8_t *buf, size_t size) override;/* bytes read, -1 if none */
  int peek() override;
  void flush() override;
  void stop() override;

  uint8_t connected() override;
  operator bool() override;

  IPAddress remoteIP();
  uint16_t  remotePort();
  IPAddress localIP();
  uint16_t  localPort();

  /* For connect(), default 10 s. */
  void setConnectionTimeout(uint32_t ms) { _connect_timeout_ms = ms; }
  /* TCP_NODELAY, off by default. */
  void setNoDelay(bool on);

  bool operator==(const WiFiClient &other) const;
  bool operator!=(const WiFiClient &other) const { return !(*this == other); }

  /* The lwIP socket, or -1. */
  int fd() const;

private:
  struct Shared;
  Shared  *_s;
  uint32_t _connect_timeout_ms;

  void release();
  bool wait_writable(uint32_t ms);
  bool failed();
};
