/**
 * @file    A7683E.cpp
 * @brief   A7683E modem driver.
 * @copyright Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "A7683E.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

A7683EUart  A7683ESerial;
A7683EClass A7683E;

A7683EClass::A7683EClass()
  : _io(&A7683ESerial), _dbg(NULL),
    _pwrkey(A7683E_NO_PIN), _reset(A7683E_NO_PIN), _netlight(A7683E_NO_PIN),
    _httpStatus(-1), _httpOpen(false), _baud(115200), _netPeriodMs(0)
{
  _line[0] = '\0';
}

/* Lifecycle */

void A7683EClass::attachControlPins()
{
  /* Open-drain: the GPIO sinks and never sources into the module rail. */
  if (_pwrkey >= 0) {
    pinMode((uint32_t)_pwrkey, OUTPUT_OPENDRAIN);
    digitalWrite((uint32_t)_pwrkey, HIGH);
  }
  if (_reset >= 0) {
    pinMode((uint32_t)_reset, OUTPUT_OPENDRAIN);
    digitalWrite((uint32_t)_reset, HIGH);
  }
  /* No pull: NETLIGHT is driven by the module. */
  if (_netlight >= 0) {
    pinMode((uint32_t)_netlight, INPUT);
  }
}

void A7683EClass::setPins(int pwrkeyPin, int resetPin, int netlightPin)
{
  _pwrkey   = pwrkeyPin;
  _reset    = resetPin;
  _netlight = netlightPin;
  attachControlPins();
}

bool A7683EClass::begin(int pwrkeyPin, int resetPin, int netlightPin, unsigned long baud)
{
  _pwrkey   = pwrkeyPin;
  _reset    = resetPin;
  _netlight = netlightPin;
  attachControlPins();

  _io = &A7683ESerial;
  if (!A7683ESerial.begin(baud)) {
    return false;
  }
  _baud = (uint32_t)baud;
  flushInput();
  return true;
}

bool A7683EClass::beginOn(Stream &io, int pwrkeyPin, int resetPin, int netlightPin)
{
  _pwrkey   = pwrkeyPin;
  _reset    = resetPin;
  _netlight = netlightPin;
  attachControlPins();
  _io = &io;
  flushInput();
  return true;
}

void A7683EClass::end()
{
  if (_io == &A7683ESerial) {
    A7683ESerial.end();
  }
}

/* Power and reset */

void A7683EClass::pulsePowerKey(uint16_t ms)
{
  if (_pwrkey < 0) {
    return;
  }
  digitalWrite((uint32_t)_pwrkey, LOW);
  delay(ms);
  digitalWrite((uint32_t)_pwrkey, HIGH);
}

void A7683EClass::pulseReset(uint16_t ms)
{
  if (_reset < 0) {
    return;
  }
  digitalWrite((uint32_t)_reset, LOW);
  delay(ms);
  digitalWrite((uint32_t)_reset, HIGH);
}

bool A7683EClass::powerOn(uint32_t timeoutMs, int attempts)
{
  if (findBaud(NULL)) {
    return true;
  }
  if (_pwrkey < 0) {
    return false;
  }

  uint32_t deadline = millis() + timeoutMs;
  for (int i = 0; i < attempts && (int32_t)(deadline - millis()) > 0; i++) {
    if (_dbg != NULL) {
      _dbg->print("[A7683E] PWRKEY pulse ");
      _dbg->print(i + 1);
      _dbg->print('/');
      _dbg->println(attempts);
    }
    /* PWRKEY low: >= 500 ms switches on, about 2.5 s switches off. */
    pulsePowerKey(1000);

    /* Boot window: probing through the boot URCs collects framing errors. */
    delay(6000);
    for (int t = 0; t < 5 && (int32_t)(deadline - millis()) > 0; t++) {
      if (findBaud(NULL)) {
        return true;
      }
      delay(1500);
    }
  }
  return false;
}

bool A7683EClass::powerOff(uint32_t timeoutMs)
{
  if (sendAT("AT+CPOF", timeoutMs)) {
    return true;
  }
  if (_pwrkey >= 0) {
    pulsePowerKey(3000);                   /* >= 2.5 s low powers down */
    delay(1000);
    return !isAlive(1000);
  }
  return false;
}

bool A7683EClass::reset(uint32_t timeoutMs)
{
  if (_reset < 0) {
    return false;
  }
  if (_dbg != NULL) {
    _dbg->println("[A7683E] hardware RESET");
  }
  pulseReset(300);
  delay(6000);                             /* boot window */

  uint32_t deadline = millis() + timeoutMs;
  while ((int32_t)(deadline - millis()) > 0) {
    if (findBaud(NULL)) {
      return true;
    }
    delay(1500);
  }
  return false;
}

bool A7683EClass::isAlive(uint32_t timeoutMs)
{
  return sendAT("AT", timeoutMs);
}

/* NETLIGHT, classified by blink period */

A7683ENetlight A7683EClass::netlightState(uint16_t windowMs)
{
  if (_netlight < 0) {
    _netPeriodMs = 0;
    return NETLIGHT_UNWIRED;
  }

  uint32_t start = millis();
  int      last  = digitalRead((uint32_t)_netlight);
  uint32_t edges = 0;
  uint32_t firstEdge = 0, lastEdgeSeen = 0;

  while ((millis() - start) < windowMs) {
    int now = digitalRead((uint32_t)_netlight);
    if (now != last) {
      uint32_t t = millis();
      if (edges == 0) { firstEdge = t; }
      lastEdgeSeen = t;
      edges++;
      last = now;
    }
    delay(2);           /* resolves a 64 ms pulse */
  }

  if (edges < 2) {
    _netPeriodMs = 0;
    return (last == HIGH) ? NETLIGHT_SOLID : NETLIGHT_DARK;
  }

  /* The period is twice the mean gap between edges. */
  uint32_t span   = (lastEdgeSeen > firstEdge) ? (lastEdgeSeen - firstEdge) : 1u;
  uint32_t period = (2u * span) / (edges - 1u);
  _netPeriodMs = (uint16_t)((period > 65535u) ? 65535u : period);

  /* A76XX periods: near 300 ms data, 800 ms searching, 3 s registered. */
  if (period < 500u)  { return NETLIGHT_DATA; }
  if (period < 1800u) { return NETLIGHT_SEARCHING; }
  return NETLIGHT_REGISTERED;
}

const char *A7683EClass::netlightName(A7683ENetlight s)
{
  switch (s) {
    case NETLIGHT_DARK:       return "dark";
    case NETLIGHT_SEARCHING:  return "searching";
    case NETLIGHT_REGISTERED: return "registered";
    case NETLIGHT_DATA:       return "data";
    case NETLIGHT_SOLID:      return "solid";
    default:                  return "unwired";
  }
}

/* AT plumbing */

void A7683EClass::flushInput()
{
  uint32_t t0 = millis();
  while (_io->available() > 0 && (millis() - t0) < 100u) {
    (void)_io->read();
  }
}

size_t A7683EClass::writeRaw(const char *s)
{
  if (s == NULL) {
    return 0;
  }
  return _io->write((const uint8_t *)s, strlen(s));
}

size_t A7683EClass::writeBytes(const uint8_t *d, size_t n)
{
  if (d == NULL || n == 0u) {
    return 0;
  }
  /* Chunked so that no single HAL call blocks for the whole body. */
  const size_t CHUNK = 1024u;
  size_t sent = 0;
  while (sent < n) {
    size_t take  = (n - sent) > CHUNK ? CHUNK : (n - sent);
    size_t wrote = _io->write(d + sent, take);
    if (wrote == 0u) {
      break;
    }
    sent += wrote;
  }
  return sent;
}

int A7683EClass::readLine(char *out, size_t outLen, uint32_t timeoutMs)
{
  if (out == NULL || outLen == 0u) {
    return -1;
  }
  size_t   n  = 0;
  uint32_t t0 = millis();
  out[0] = '\0';

  for (;;) {
    if ((millis() - t0) >= timeoutMs) {
      out[n] = '\0';
      return -1;
    }
    int c = _io->read();
    if (c < 0) {
      delay(1);
      continue;
    }
    if (c == '\r') { continue; }
    if (c == '\n') {
      if (n == 0u) { continue; }
      out[n] = '\0';
      return (int)n;
    }
    if ((n + 1u) < outLen) {
      out[n++] = (char)c;
    }
  }
}

int A7683EClass::waitResponse(const char *expect, uint32_t timeoutMs)
{
  uint32_t t0 = millis();

  for (;;) {
    uint32_t elapsed = millis() - t0;
    if (elapsed >= timeoutMs) {
      return -1;
    }
    if (readLine(_line, sizeof(_line), timeoutMs - elapsed) < 0) {
      return -1;
    }
    if (_dbg != NULL) {
      _dbg->print("[A7683E] < ");
      _dbg->println(_line);
    }
    if (expect != NULL && strncmp(_line, expect, strlen(expect)) == 0) {
      return 1;
    }
    if (strcmp(_line, "ERROR") == 0 ||
        strncmp(_line, "+CME ERROR", 10) == 0 ||
        strncmp(_line, "+CMS ERROR", 10) == 0) {
      return 0;
    }
    /* Any other line is an echo or a URC. */
  }
}

bool A7683EClass::sendATExpect(const char *cmd, const char *expect, uint32_t timeoutMs)
{
  flushInput();
  if (_dbg != NULL) {
    _dbg->print("[A7683E] > ");
    _dbg->println(cmd);
  }
  writeRaw(cmd);
  _io->write((uint8_t)'\r');
  return waitResponse(expect, timeoutMs) == 1;
}

bool A7683EClass::sendAT(const char *cmd, uint32_t timeoutMs)
{
  return sendATExpect(cmd, "OK", timeoutMs);
}

bool A7683EClass::queryLine(const char *cmd, char *out, size_t outLen, uint32_t timeoutMs)
{
  if (out == NULL || outLen == 0u) {
    return false;
  }
  out[0] = '\0';

  flushInput();
  if (_dbg != NULL) {
    _dbg->print("[A7683E] > ");
    _dbg->println(cmd);
  }
  writeRaw(cmd);
  _io->write((uint8_t)'\r');

  bool     got = false;
  uint32_t t0  = millis();

  for (;;) {
    uint32_t elapsed = millis() - t0;
    if (elapsed >= timeoutMs) {
      return false;
    }
    if (readLine(_line, sizeof(_line), timeoutMs - elapsed) < 0) {
      return false;
    }
    if (_dbg != NULL) {
      _dbg->print("[A7683E] < ");
      _dbg->println(_line);
    }
    if (strcmp(_line, "OK") == 0) {
      return got;
    }
    if (strcmp(_line, "ERROR") == 0 || strncmp(_line, "+CME ERROR", 10) == 0) {
      return false;
    }
    if (strcmp(_line, cmd) == 0) {
      continue;                          /* echo */
    }
    if (!got) {
      strncpy(out, _line, outLen - 1u);
      out[outLen - 1u] = '\0';
      got = true;
    }
  }
}

/* Serial speed */

static const uint32_t A7683E_BAUD_CANDIDATES[] = {
  115200u, 921600u, 460800u, 230400u, 57600u, 38400u, 9600u
};

bool A7683EClass::findBaud(uint32_t *foundBaud)
{
  if (_io != &A7683ESerial) {
    bool ok = isAlive(700);
    if (ok && foundBaud != NULL) { *foundBaud = 0u; }
    return ok;
  }

  if (isAlive(600)) {
    if (foundBaud != NULL) { *foundBaud = _baud; }
    return true;
  }

  const size_t n = sizeof(A7683E_BAUD_CANDIDATES) / sizeof(A7683E_BAUD_CANDIDATES[0]);
  for (size_t i = 0; i < n; i++) {
    uint32_t rate = A7683E_BAUD_CANDIDATES[i];
    if (rate == _baud) { continue; }
    if (!A7683ESerial.begin(rate)) { continue; }
    _baud = rate;
    delay(30);
    flushInput();
    if (isAlive(700)) {
      if (_dbg != NULL) {
        _dbg->print("[A7683E] found the modem at ");
        _dbg->println(rate);
      }
      if (foundBaud != NULL) { *foundBaud = rate; }
      return true;
    }
  }
  return false;
}

bool A7683EClass::syncBaud(uint32_t baud)
{
  if (_io != &A7683ESerial) {
    return false;
  }
  if (!findBaud(NULL)) {
    return false;
  }
  if (_baud == baud) {
    return true;
  }

  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+IPR=%lu", (unsigned long)baud);
  if (!sendAT(cmd, 3000)) {
    return false;
  }
  delay(50);

  if (!A7683ESerial.begin(baud)) {
    return false;
  }
  _baud = baud;
  delay(100);
  flushInput();
  return isAlive(2000);
}

/* Status */

bool A7683EClass::waitForReady(uint32_t timeoutMs)
{
  uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    if (isAlive(1000)) {
      (void)sendAT("ATE0", 2000);        /* echo off */
      if (simReady(2000)) {
        return true;
      }
    }
    delay(500);
  }
  return false;
}

bool A7683EClass::simReady(uint32_t timeoutMs)
{
  return sendATExpect("AT+CPIN?", "+CPIN: READY", timeoutMs);
}

int A7683EClass::signalQuality()
{
  if (!sendATExpect("AT+CSQ", "+CSQ:", 3000)) {
    return -1;
  }
  const char *p = strchr(_line, ':');
  return (p != NULL) ? atoi(p + 1) : -1;
}

int A7683EClass::signalDbm()
{
  int csq = signalQuality();
  if (csq < 0 || csq > 31) {             /* 99: not known */
    return 0;
  }
  return -113 + (2 * csq);
}

int A7683EClass::registrationStatus()
{
  if (!sendATExpect("AT+CREG?", "+CREG:", 3000)) {
    return -1;
  }
  const char *p = strchr(_line, ',');
  return (p != NULL) ? atoi(p + 1) : -1;
}

bool A7683EClass::isRegistered()
{
  int stat = registrationStatus();
  return (stat == 1) || (stat == 5);
}

bool A7683EClass::waitForNetwork(uint32_t timeoutMs)
{
  uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    if (isRegistered()) {
      return true;
    }
    delay(2000);
  }
  return false;
}

bool A7683EClass::getIMEI(char *out, size_t outLen)  { return queryLine("AT+CGSN", out, outLen, 3000); }
bool A7683EClass::getModel(char *out, size_t outLen) { return queryLine("AT+CGMM", out, outLen, 3000); }

bool A7683EClass::getOperator(char *out, size_t outLen)
{
  if (out == NULL || outLen == 0u) {
    return false;
  }
  out[0] = '\0';

  /* +COPS: the name is the first quoted field, absent while unregistered. */
  if (!sendATExpect("AT+COPS?", "+COPS:", 5000)) {
    return false;
  }
  const char *q1 = strchr(_line, '"');
  if (q1 == NULL) { return false; }
  const char *q2 = strchr(q1 + 1, '"');
  if (q2 == NULL) { return false; }

  size_t n = (size_t)(q2 - q1 - 1);
  if (n >= outLen) { n = outLen - 1u; }
  memcpy(out, q1 + 1, n);
  out[n] = '\0';
  return n > 0u;
}

/* Packet data */

bool A7683EClass::setApn(const char *apn)
{
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", (apn != NULL) ? apn : "");
  return sendAT(cmd, 5000);
}

bool A7683EClass::setApnAuth(const char *user, const char *pass, int authType)
{
  char cmd[160];

  if (user == NULL || user[0] == '\0') {
    return sendAT("AT+CGAUTH=1,0", 5000);
  }

  /* Write order is <cid>,<auth_type>,<passwd>,<user>: password first. */
  snprintf(cmd, sizeof(cmd), "AT+CGAUTH=1,%d,\"%s\",\"%s\"",
           authType, (pass != NULL) ? pass : "", user);
  return sendAT(cmd, 5000);
}

bool A7683EClass::getLocalIP(char *out, size_t outLen)
{
  if (out == NULL || outLen == 0u) {
    return false;
  }
  out[0] = '\0';

  /* +CGPADDR: 1,"10.1.2.3". Some firmwares omit the quotes. */
  if (!sendATExpect("AT+CGPADDR=1", "+CGPADDR:", 5000)) {
    return false;
  }
  const char *p = strchr(_line, ',');
  if (p == NULL) { return false; }
  p++;
  while (*p == ' ' || *p == '"') { p++; }

  size_t n = 0;
  while (p[n] != '\0' && p[n] != '"' && p[n] != ',' && (n + 1u) < outLen) { n++; }
  memcpy(out, p, n);
  out[n] = '\0';
  return (n > 0u) && (strcmp(out, "0.0.0.0") != 0);
}

bool A7683EClass::attachData(const char *apn, uint32_t timeoutMs)
{
  return attachData(apn, NULL, NULL, timeoutMs);
}

bool A7683EClass::attachData(const char *apn, const char *user, const char *pass,
                             uint32_t timeoutMs)
{
  if (!setApn(apn)) {
    return false;
  }
  if (user != NULL && user[0] != '\0') {
    /* PAP, then CHAP. */
    if (!setApnAuth(user, pass, 1)) {
      (void)setApnAuth(user, pass, 2);
    }
  }

  /* Both can answer ERROR when already attached, so CGPADDR decides. */
  (void)sendAT("AT+CGATT=1", 20000);
  (void)sendAT("AT+CGACT=1,1", 20000);

  char     ip[24];
  uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    if (getLocalIP(ip, sizeof(ip))) {
      if (_dbg != NULL) {
        _dbg->print("[A7683E] PDP context up, IP ");
        _dbg->println(ip);
      }
      return true;
    }
    delay(2000);
    (void)sendAT("AT+CGACT=1,1", 10000);
  }
  return false;
}

/* HTTP */

bool A7683EClass::httpBegin(const char *url, const char *contentType)
{
  if (url == NULL) {
    return false;
  }
  httpEnd();

  if (!sendAT("AT+HTTPINIT", 10000)) {
    return false;
  }
  _httpOpen = true;

  char cmd[320];
  snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
  if (!sendAT(cmd, 5000)) {
    httpEnd();
    return false;
  }

  if (contentType != NULL && contentType[0] != '\0') {
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"CONTENT\",\"%s\"", contentType);
    (void)sendAT(cmd, 5000);
  }

  if (strncmp(url, "https://", 8) == 0) {
    /* No CA store: the link is encrypted but not authenticated. */
    (void)sendAT("AT+HTTPPARA=\"SSLCFG\",0", 5000);
    (void)sendAT("AT+SSLCFG=\"sslversion\",0,4", 5000);
    (void)sendAT("AT+SSLCFG=\"authmode\",0,0", 5000);
  }
  return true;
}

void A7683EClass::httpEnd()
{
  (void)sendAT("AT+HTTPTERM", 5000);     /* ERROR when no session is open */
  _httpOpen = false;
}

bool A7683EClass::waitPrompt(uint32_t timeoutMs)
{
  char     tail[8] = {0};
  size_t   n  = 0;
  uint32_t t0 = millis();

  while ((millis() - t0) < timeoutMs) {
    int c = _io->read();
    if (c < 0) { delay(1); continue; }
    if (c == '>') { return true; }

    if (n == (sizeof(tail) - 1u)) {
      memmove(tail, tail + 1, sizeof(tail) - 2u);
      n--;
    }
    tail[n++] = (char)c;
    tail[n]   = '\0';
    if (strstr(tail, "ERROR") != NULL) { return false; }
  }
  return false;
}

bool A7683EClass::httpPost(const uint8_t *body, size_t len, uint32_t timeoutMs)
{
  _httpStatus = -1;

  if (body == NULL || len == 0u || !_httpOpen) {
    return false;
  }

  /* AT+HTTPDATA <size> caps at 153600 bytes and each call appends. Its <time>
   * argument is in seconds. */
  const size_t HTTPDATA_MAX = 153600u;

  size_t offset = 0;
  while (offset < len) {
    size_t chunk = (len - offset) > HTTPDATA_MAX ? HTTPDATA_MAX : (len - offset);

    char cmd[48];
    snprintf(cmd, sizeof(cmd), "AT+HTTPDATA=%lu,60", (unsigned long)chunk);

    flushInput();
    if (_dbg != NULL) {
      _dbg->print("[A7683E] > ");
      _dbg->println(cmd);
    }
    writeRaw(cmd);
    _io->write((uint8_t)'\r');

    if (!waitPrompt(10000)) {
      return false;
    }
    if (writeBytes(body + offset, chunk) != chunk) {
      return false;
    }
    if (waitResponse("OK", 30000) != 1) {
      return false;
    }
    offset += chunk;
  }

  if (!sendAT("AT+HTTPACTION=1", 10000)) {   /* 1: POST */
    return false;
  }

  /* The result arrives as a +HTTPACTION URC. */
  if (waitResponse("+HTTPACTION:", timeoutMs) != 1) {
    return false;
  }

  const char *p = strchr(_line, ',');
  if (p != NULL) {
    _httpStatus = atoi(p + 1);
  }
  return (_httpStatus >= 200) && (_httpStatus < 300);
}
