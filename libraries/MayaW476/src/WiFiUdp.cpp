/**
 ******************************************************************************
 * @file    WiFiUdp.cpp
 * @brief   WiFiUDP over lwIP's BSD sockets.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "WiFiUdp.h"
#include "MayaW476.h"

#include <string.h>
#include <stdlib.h>

#include "maya_w4_board/maya_w4_sockets.h"

WiFiUDP::WiFiUDP()
  : _fd(-1), _port(0), _tx(nullptr), _tx_len(0), _tx_open(false), _tx_port(0),
    _rx(nullptr), _rx_len(0), _rx_pos(0), _rx_port(0) {}

WiFiUDP::~WiFiUDP()
{
  stop();
  free(_tx);
  free(_rx);
}

bool WiFiUDP::open_socket(uint16_t port, bool reuse)
{
  stop();
  const int fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) return false;
  int on = 1;
  (void)lwip_setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
  if (reuse) (void)lwip_setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port        = htons(port);
  if (lwip_bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
    lwip_close(fd);
    return false;
  }
  (void)lwip_ioctl(fd, FIONBIO, &on);
  _fd = fd;
  _port = port;
  return true;
}

uint8_t WiFiUDP::begin(uint16_t port)
{
  return open_socket(port, true) ? 1 : 0;
}

uint8_t WiFiUDP::beginMulticast(IPAddress ip, uint16_t port)
{
  if (!open_socket(port, true)) return 0;
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = ip.raw();
  mreq.imr_interface.s_addr = htonl(INADDR_ANY);
  if (lwip_setsockopt(_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) != 0) {
    stop();
    return 0;
  }
  return 1;
}

void WiFiUDP::stop()
{
  if (_fd >= 0) {
    lwip_close(_fd);
    _fd = -1;
  }
  _port = 0;
  _rx_len = _rx_pos = 0;
  _tx_open = false;
}

/* Sending */

int WiFiUDP::beginPacket(IPAddress ip, uint16_t port)
{
  if (_tx == nullptr) _tx = (uint8_t *)malloc(kMaxDatagram);
  if (_tx == nullptr) return 0;
  if (_fd < 0 && !open_socket(0, false)) return 0;   /* send-only: any local port */
  _tx_ip = ip;
  _tx_port = port;
  _tx_len = 0;
  _tx_open = true;
  return 1;
}

int WiFiUDP::beginPacket(const char *host, uint16_t port)
{
  IPAddress ip;
  if (!WiFi.hostByName(host, ip)) return 0;
  return beginPacket(ip, port);
}

size_t WiFiUDP::write(uint8_t b)
{
  return write(&b, 1);
}

size_t WiFiUDP::write(const uint8_t *buffer, size_t size)
{
  if (!_tx_open || buffer == nullptr) return 0;
  const size_t room = kMaxDatagram - _tx_len;
  const size_t n = (size < room) ? size : room;
  memcpy(_tx + _tx_len, buffer, n);
  _tx_len += n;
  return n;
}

int WiFiUDP::endPacket()
{
  if (!_tx_open || _fd < 0) return 0;
  _tx_open = false;
  struct sockaddr_in to;
  memset(&to, 0, sizeof(to));
  to.sin_family      = AF_INET;
  to.sin_addr.s_addr = _tx_ip.raw();
  to.sin_port        = htons(_tx_port);
  const int n = lwip_sendto(_fd, _tx, _tx_len, 0, (struct sockaddr *)&to, sizeof(to));
  return (n >= 0 && (size_t)n == _tx_len) ? 1 : 0;
}

/* Receiving */

int WiFiUDP::parsePacket()
{
  _rx_len = _rx_pos = 0;
  if (_fd < 0) return 0;
  if (_rx == nullptr) _rx = (uint8_t *)malloc(kMaxDatagram);
  if (_rx == nullptr) return 0;

  fd_set rd;
  FD_ZERO(&rd);
  FD_SET(_fd, &rd);
  struct timeval now = { 0, 0 };
  if (lwip_select(_fd + 1, &rd, nullptr, nullptr, &now) <= 0) return 0;

  struct sockaddr_in from;
  socklen_t len = sizeof(from);
  memset(&from, 0, sizeof(from));
  const int n = lwip_recvfrom(_fd, _rx, kMaxDatagram, MSG_DONTWAIT,
                              (struct sockaddr *)&from, &len);
  if (n <= 0) return 0;
  _rx_len  = (size_t)n;
  _rx_ip   = IPAddress((uint32_t)from.sin_addr.s_addr);
  _rx_port = ntohs(from.sin_port);
  return n;
}

int WiFiUDP::available()
{
  return (int)(_rx_len - _rx_pos);
}

int WiFiUDP::read()
{
  return (_rx_pos < _rx_len) ? _rx[_rx_pos++] : -1;
}

int WiFiUDP::read(unsigned char *buffer, size_t len)
{
  if (buffer == nullptr || _rx_pos >= _rx_len) return -1;
  const size_t left = _rx_len - _rx_pos;
  const size_t n = (len < left) ? len : left;
  memcpy(buffer, _rx + _rx_pos, n);
  _rx_pos += n;
  return (int)n;
}

int WiFiUDP::read(char *buffer, size_t len)
{
  return read((unsigned char *)buffer, len);
}

int WiFiUDP::peek()
{
  return (_rx_pos < _rx_len) ? _rx[_rx_pos] : -1;
}

void WiFiUDP::flush()
{
  _rx_pos = _rx_len;     /* drops the rest of the current datagram */
}

IPAddress WiFiUDP::remoteIP()
{
  return _rx_ip;
}

uint16_t WiFiUDP::remotePort()
{
  return _rx_port;
}
