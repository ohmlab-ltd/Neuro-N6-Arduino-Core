/**
 ******************************************************************************
 * @file    WiFiUdp.h
 * @brief   Arduino's WiFiUDP: datagrams over the board's WiFi.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#pragma once

#include <Arduino.h>
#include "Udp.h"
#include "IPAddress.h"

class WiFiUDP : public UDP {
public:
  WiFiUDP();
  ~WiFiUDP();

  uint8_t begin(uint16_t port) override;
  uint8_t beginMulticast(IPAddress ip, uint16_t port) override;
  void stop() override;

  int beginPacket(IPAddress ip, uint16_t port) override;
  int beginPacket(const char *host, uint16_t port) override;
  int endPacket() override;
  size_t write(uint8_t b) override;
  size_t write(const uint8_t *buffer, size_t size) override;
  using Print::write;

  int parsePacket() override;
  int available() override;
  int read() override;
  int read(unsigned char *buffer, size_t len) override;
  int read(char *buffer, size_t len) override;
  int peek() override;
  void flush() override;

  IPAddress remoteIP() override;
  uint16_t remotePort() override;

  static const size_t kMaxDatagram = 1472;   /* bytes, one Ethernet-sized frame */

private:
  WiFiUDP(const WiFiUDP &) = delete;
  WiFiUDP &operator=(const WiFiUDP &) = delete;

  int       _fd;
  uint16_t  _port;
  uint8_t  *_tx;
  size_t    _tx_len;
  bool      _tx_open;
  IPAddress _tx_ip;
  uint16_t  _tx_port;
  uint8_t  *_rx;
  size_t    _rx_len;
  size_t    _rx_pos;
  IPAddress _rx_ip;
  uint16_t  _rx_port;

  bool open_socket(uint16_t port, bool reuse);
};

typedef WiFiUDP WiFiUdp;
