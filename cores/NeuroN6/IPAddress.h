/**
 ******************************************************************************
 * @file    IPAddress.h
 * @brief   Arduino IPAddress value type, IPv4 only.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef NEURON6_IPADDRESS_H
#define NEURON6_IPADDRESS_H

#include <stdint.h>

#ifdef __cplusplus

#include "api/Printable.h"
#include "api/Print.h"
#include "api/String.h"

class IPAddress : public arduino::Printable {
public:
  IPAddress() {
    _bytes[0] = _bytes[1] = _bytes[2] = _bytes[3] = 0;
  }

  IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    _bytes[0] = a; _bytes[1] = b; _bytes[2] = c; _bytes[3] = d;
  }

  /* addr is in wifi_nxp host-endian layout: the first octet is the low byte. */
  explicit IPAddress(uint32_t addr) {
    _bytes[0] = (uint8_t)( addr        & 0xFFu);
    _bytes[1] = (uint8_t)((addr >>  8) & 0xFFu);
    _bytes[2] = (uint8_t)((addr >> 16) & 0xFFu);
    _bytes[3] = (uint8_t)((addr >> 24) & 0xFFu);
  }

  uint8_t  operator[](int idx) const { return _bytes[idx & 3]; }
  uint8_t& operator[](int idx)       { return _bytes[idx & 3]; }

  /* The host-endian uint32 form wifi_nxp and the IP-announce TLV use. */
  uint32_t raw() const {
    return (uint32_t)_bytes[0]
         | ((uint32_t)_bytes[1] <<  8)
         | ((uint32_t)_bytes[2] << 16)
         | ((uint32_t)_bytes[3] << 24);
  }

  bool operator==(const IPAddress& o) const {
    return _bytes[0] == o._bytes[0] && _bytes[1] == o._bytes[1]
        && _bytes[2] == o._bytes[2] && _bytes[3] == o._bytes[3];
  }
  bool operator!=(const IPAddress& o) const { return !(*this == o); }

  size_t printTo(arduino::Print& p) const override {
    size_t n = 0;
    for (int i = 0; i < 4; i++) {
      if (i) n += p.print('.');
      n += p.print(_bytes[i]);
    }
    return n;
  }

  /* False for 0.0.0.0, which means no address. */
  bool isSet() const {
    return _bytes[0] | _bytes[1] | _bytes[2] | _bytes[3];
  }
  /* Parses "a.b.c.d". Returns false, leaving the address unchanged, for
   * anything else. */
  bool fromString(const char *s) {
    uint8_t b[4];
    for (int i = 0; i < 4; i++) {
      if (s == nullptr || *s < '0' || *s > '9') return false;
      unsigned v = 0;
      int digits = 0;
      while (*s >= '0' && *s <= '9') {
        v = v * 10u + (unsigned)(*s++ - '0');
        if (++digits > 3 || v > 255u) return false;
      }
      b[i] = (uint8_t)v;
      if (i < 3 && *s++ != '.') return false;
    }
    if (*s != '\0') return false;
    for (int i = 0; i < 4; i++) _bytes[i] = b[i];
    return true;
  }
  bool fromString(const arduino::String &s) { return fromString(s.c_str()); }
  arduino::String toString() const {
    char buf[16];
    int n = 0;
    for (int i = 0; i < 4; i++) {
      unsigned v = _bytes[i];
      if (i) buf[n++] = '.';
      if (v >= 100u) buf[n++] = (char)('0' + v / 100u);
      if (v >= 10u)  buf[n++] = (char)('0' + (v / 10u) % 10u);
      buf[n++] = (char)('0' + v % 10u);
    }
    buf[n] = '\0';
    return arduino::String(buf);
  }

private:
  uint8_t _bytes[4];
};

#endif /* __cplusplus */
#endif /* NEURON6_IPADDRESS_H */
