/**
 ******************************************************************************
 * @file    WiFiServer.cpp
 * @brief   WiFiServer over lwIP's BSD sockets.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "WiFiServer.h"

#include <string.h>

#include "maya_w4_board/maya_w4_sockets.h"

WiFiServer::WiFiServer(uint16_t port) : _port(port), _fd(-1) {}

WiFiServer::~WiFiServer()
{
  end();
}

void WiFiServer::begin(uint16_t port)
{
  _port = port;
  begin();
}

void WiFiServer::begin()
{
  end();
  const int fd = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) return;

  int on = 1;
  (void)lwip_setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port        = htons(_port);
  if (lwip_bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
      lwip_listen(fd, kMaxClients) != 0) {
    lwip_close(fd);
    return;
  }
  (void)lwip_ioctl(fd, FIONBIO, &on);
  _fd = fd;
}

void WiFiServer::end()
{
  for (int i = 0; i < kMaxClients; i++) {
    _clients[i].stop();
    _clients[i] = WiFiClient();
  }
  if (_fd >= 0) {
    lwip_close(_fd);
    _fd = -1;
  }
}

uint8_t WiFiServer::status()
{
  return (_fd >= 0) ? 1 : 0;
}

/* One waiting connection's socket, or -1. */
int WiFiServer::accept_fd()
{
  if (_fd < 0) return -1;
  fd_set rd;
  FD_ZERO(&rd);
  FD_SET(_fd, &rd);
  struct timeval now = { 0, 0 };
  if (lwip_select(_fd + 1, &rd, nullptr, nullptr, &now) <= 0) return -1;
  struct sockaddr_in peer;
  socklen_t len = sizeof(peer);
  return lwip_accept(_fd, (struct sockaddr *)&peer, &len);
}

WiFiClient WiFiServer::accept()
{
  return WiFiClient(accept_fd());
}

/* With every slot busy, new connections wait in the listen backlog. */
void WiFiServer::take_new_clients()
{
  for (int i = 0; i < kMaxClients; i++) {
    if (_clients[i].fd() >= 0 && !_clients[i].connected()) {
      _clients[i].stop();
      _clients[i] = WiFiClient();
    }
  }
  for (int i = 0; i < kMaxClients; i++) {
    if (_clients[i].fd() >= 0) continue;
    const int fd = accept_fd();
    if (fd < 0) break;
    _clients[i] = WiFiClient(fd);
  }
}

WiFiClient WiFiServer::available()
{
  take_new_clients();
  for (int i = 0; i < kMaxClients; i++) {
    if (_clients[i].fd() >= 0 && _clients[i].available() > 0) return _clients[i];
  }
  return WiFiClient();
}

size_t WiFiServer::write(uint8_t b)
{
  return write(&b, 1);
}

size_t WiFiServer::write(const uint8_t *buf, size_t size)
{
  take_new_clients();
  size_t n = 0;
  for (int i = 0; i < kMaxClients; i++) {
    if (_clients[i].fd() >= 0) n = _clients[i].write(buf, size);
  }
  return n;
}
