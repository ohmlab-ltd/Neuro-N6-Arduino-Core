/**
 ******************************************************************************
 * @file    WiFiClient.cpp
 * @brief   WiFiClient over lwIP's BSD sockets.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "WiFiClient.h"
#include "MayaW476.h"

#include <string.h>
#include <stdlib.h>

#include "maya_w4_board/maya_w4_sockets.h"

/* lwIP's errno is one global shared by every task and is not read here: a
 * zero-timeout select reports "would block" and SO_ERROR reports a failure. */

struct WiFiClient::Shared {
  int     fd;
  int     refs;
  int     peeked;    /* one byte read ahead by peek(), or -1 */
  bool    eof;       /* the peer closed its side */
};

WiFiClient::WiFiClient() : _s(nullptr), _connect_timeout_ms(10000U) {}

WiFiClient::WiFiClient(int fd) : _s(nullptr), _connect_timeout_ms(10000U)
{
  if (fd < 0) return;
  _s = (Shared *)malloc(sizeof(Shared));
  if (_s == nullptr) { lwip_close(fd); return; }
  _s->fd = fd;
  _s->refs = 1;
  _s->peeked = -1;
  _s->eof = false;
  int on = 1;
  (void)lwip_ioctl(fd, FIONBIO, &on);
}

WiFiClient::WiFiClient(const WiFiClient &other)
  : _s(other._s), _connect_timeout_ms(other._connect_timeout_ms)
{
  if (_s) _s->refs++;
}

WiFiClient &WiFiClient::operator=(const WiFiClient &other)
{
  if (this != &other) {
    if (other._s) other._s->refs++;
    release();
    _s = other._s;
    _connect_timeout_ms = other._connect_timeout_ms;
  }
  return *this;
}

WiFiClient::~WiFiClient()
{
  release();
}

void WiFiClient::release()
{
  if (_s == nullptr) return;
  if (--_s->refs == 0) {
    if (_s->fd >= 0) lwip_close(_s->fd);
    free(_s);
  }
  _s = nullptr;
}

int WiFiClient::fd() const
{
  return (_s != nullptr) ? _s->fd : -1;
}

bool WiFiClient::operator==(const WiFiClient &other) const
{
  return fd() == other.fd();
}

/* Connecting */

int WiFiClient::connect(IPAddress ip, uint16_t port)
{
  stop();
  release();

  const int fd = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) return 0;
  int on = 1;
  (void)lwip_ioctl(fd, FIONBIO, &on);

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(port);
  addr.sin_addr.s_addr = ip.raw();
  (void)lwip_connect(fd, (struct sockaddr *)&addr, sizeof(addr));

  /* The outcome arrives as writability. SO_ERROR says which. */
  fd_set wr;
  FD_ZERO(&wr);
  FD_SET(fd, &wr);
  struct timeval tv;
  tv.tv_sec  = (long)(_connect_timeout_ms / 1000U);
  tv.tv_usec = (long)((_connect_timeout_ms % 1000U) * 1000U);
  int err = -1;
  if (lwip_select(fd + 1, nullptr, &wr, nullptr, &tv) > 0) {
    socklen_t len = sizeof(err);
    if (lwip_getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) != 0) err = -1;
  }
  if (err != 0) {
    lwip_close(fd);
    return 0;
  }

  /* The assignment would copy the default timeout: setConnectionTimeout() stays. */
  const uint32_t timeout_ms = _connect_timeout_ms;
  WiFiClient fresh(fd);
  *this = fresh;
  _connect_timeout_ms = timeout_ms;
  return (_s != nullptr) ? 1 : 0;
}

int WiFiClient::connect(const char *host, uint16_t port)
{
  IPAddress ip;
  if (!WiFi.hostByName(host, ip)) return 0;
  return connect(ip, port);
}

void WiFiClient::setNoDelay(bool on)
{
  if (_s == nullptr || _s->fd < 0) return;
  int v = on ? 1 : 0;
  (void)lwip_setsockopt(_s->fd, IPPROTO_TCP, TCP_NODELAY, &v, sizeof(v));
}

/* Stops the client and returns true when the connection has gone. lwIP reports
 * a connection's error once, so an unreadable peer address also counts. */
bool WiFiClient::failed()
{
  int err = 0;
  socklen_t len = sizeof(err);
  struct sockaddr_in peer;
  socklen_t plen = sizeof(peer);
  if (lwip_getsockopt(_s->fd, SOL_SOCKET, SO_ERROR, &err, &len) != 0 || err != 0 ||
      lwip_getpeername(_s->fd, (struct sockaddr *)&peer, &plen) != 0) {
    stop();
    return true;
  }
  return false;
}

/* Writing */

bool WiFiClient::wait_writable(uint32_t ms)
{
  fd_set wr;
  FD_ZERO(&wr);
  FD_SET(_s->fd, &wr);
  struct timeval tv;
  tv.tv_sec  = (long)(ms / 1000U);
  tv.tv_usec = (long)((ms % 1000U) * 1000U);
  return lwip_select(_s->fd + 1, nullptr, &wr, nullptr, &tv) > 0;
}

size_t WiFiClient::write(uint8_t b)
{
  return write(&b, 1);
}

size_t WiFiClient::write(const uint8_t *buf, size_t size)
{
  if (_s == nullptr || _s->fd < 0 || buf == nullptr) return 0;
  size_t sent = 0;
  while (sent < size) {
    const int n = lwip_send(_s->fd, buf + sent, size - sent, MSG_DONTWAIT);
    if (n > 0) { sent += (size_t)n; continue; }

    /* The send buffer is full, or the connection has gone. */
    if (failed()) break;
    if (!wait_writable(getTimeout())) break;
  }
  return sent;
}

void WiFiClient::flush()
{
  /* write() hands every byte to the stack before it returns. */
}

/* Reading */

int WiFiClient::available()
{
  if (_s == nullptr || _s->fd < 0) return 0;
  int n = 0;
  if (lwip_ioctl(_s->fd, FIONREAD, &n) != 0 || n < 0) n = 0;
  return n + ((_s->peeked >= 0) ? 1 : 0);
}

int WiFiClient::read(uint8_t *buf, size_t size)
{
  if (_s == nullptr || buf == nullptr || size == 0) return -1;
  size_t got = 0;
  if (_s->peeked >= 0) {
    buf[got++] = (uint8_t)_s->peeked;
    _s->peeked = -1;
  }
  if (got < size && _s->fd >= 0 && !_s->eof) {
    fd_set rd;
    FD_ZERO(&rd);
    FD_SET(_s->fd, &rd);
    struct timeval now = { 0, 0 };
    if (lwip_select(_s->fd + 1, &rd, nullptr, nullptr, &now) > 0) {
      const int n = lwip_recv(_s->fd, buf + got, size - got, MSG_DONTWAIT);
      if (n > 0)       got += (size_t)n;
      else if (n == 0) _s->eof = true;
      else             (void)failed();
    }
  }
  return (got > 0) ? (int)got : -1;
}

int WiFiClient::read()
{
  uint8_t b;
  return (read(&b, 1) == 1) ? (int)b : -1;
}

int WiFiClient::peek()
{
  if (_s == nullptr) return -1;
  if (_s->peeked < 0) {
    uint8_t b;
    if (read(&b, 1) != 1) return -1;
    _s->peeked = b;
  }
  return _s->peeked;
}

/* State */

void WiFiClient::stop()
{
  if (_s == nullptr) return;
  if (_s->fd >= 0) {
    lwip_close(_s->fd);
    _s->fd = -1;
  }
  _s->peeked = -1;
}

uint8_t WiFiClient::connected()
{
  if (_s == nullptr) return 0;
  if (available() > 0) return 1;
  if (_s->fd < 0 || _s->eof) return 0;

  fd_set rd;
  FD_ZERO(&rd);
  FD_SET(_s->fd, &rd);
  struct timeval now = { 0, 0 };
  if (lwip_select(_s->fd + 1, &rd, nullptr, nullptr, &now) > 0) {
    uint8_t b;
    const int n = lwip_recv(_s->fd, &b, 1, MSG_PEEK | MSG_DONTWAIT);
    if (n == 0) { _s->eof = true; return 0; }
  }
  return failed() ? 0 : 1;
}

WiFiClient::operator bool()
{
  return connected() != 0;
}

static bool sock_name(int fd, bool peer, IPAddress &ip, uint16_t &port)
{
  struct sockaddr_in a;
  socklen_t len = sizeof(a);
  memset(&a, 0, sizeof(a));
  const int rc = peer ? lwip_getpeername(fd, (struct sockaddr *)&a, &len)
                      : lwip_getsockname(fd, (struct sockaddr *)&a, &len);
  if (rc != 0) return false;
  ip   = IPAddress((uint32_t)a.sin_addr.s_addr);
  port = ntohs(a.sin_port);
  return true;
}

IPAddress WiFiClient::remoteIP()
{
  IPAddress ip; uint16_t port = 0;
  if (_s != nullptr && _s->fd >= 0) (void)sock_name(_s->fd, true, ip, port);
  return ip;
}

uint16_t WiFiClient::remotePort()
{
  IPAddress ip; uint16_t port = 0;
  if (_s != nullptr && _s->fd >= 0) (void)sock_name(_s->fd, true, ip, port);
  return port;
}

IPAddress WiFiClient::localIP()
{
  IPAddress ip; uint16_t port = 0;
  if (_s != nullptr && _s->fd >= 0) (void)sock_name(_s->fd, false, ip, port);
  return ip;
}

uint16_t WiFiClient::localPort()
{
  IPAddress ip; uint16_t port = 0;
  if (_s != nullptr && _s->fd >= 0) (void)sock_name(_s->fd, false, ip, port);
  return port;
}
