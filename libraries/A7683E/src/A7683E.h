/**
 * @file    A7683E.h
 * @brief   SIMCom A7683E LTE Cat-1 bis modem driver.
 * @copyright Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef A7683E_H
#define A7683E_H

#include <Arduino.h>
#include "a7683e_uart.h"

/* A control pin that is not wired. */
#define A7683E_NO_PIN   (-1)

/* Longest response line kept. */
#define A7683E_LINE_MAX 160

/* UART4 as an Arduino Stream. */
class A7683EUart : public Stream {
public:
  bool begin(unsigned long baud = 115200) { return a7683e_uart_begin((uint32_t)baud); }
  void end()                              { a7683e_uart_end(); }

  int available() override                { return a7683e_uart_available(); }
  int read() override                     { return a7683e_uart_read(); }
  int peek() override                     { return a7683e_uart_peek(); }

  size_t write(uint8_t b) override        { return a7683e_uart_write(&b, 1); }
  size_t write(const uint8_t *buf, size_t len) override { return a7683e_uart_write(buf, len); }
  using Print::write;

  void flush() override                   { }      /* TX is blocking */
  int  availableForWrite() override       { return 1; }

  void     discardRx()                    { a7683e_uart_discard(); }
  uint32_t rxDropped()                    { return a7683e_uart_dropped(); }
  uint32_t rxTotal()                      { return a7683e_uart_rx_total(); }
  uint32_t kernelHz()                     { return a7683e_uart_kernel_hz(); }
};

extern A7683EUart A7683ESerial;

/* Network state from the NETLIGHT blink period. */
enum A7683ENetlight {
  NETLIGHT_UNWIRED = 0,   /* no pin */
  NETLIGHT_DARK,          /* steady low: module off or not registered */
  NETLIGHT_SEARCHING,     /* fast blink */
  NETLIGHT_REGISTERED,    /* slow blink */
  NETLIGHT_DATA,          /* fastest blink */
  NETLIGHT_SOLID,         /* steady high */
};

/* Every call is synchronous and bounded by a timeout. */
class A7683EClass {
public:
  A7683EClass();

  /* Lifecycle */

  /* Sets up the control pins and UART4. Does not power the module on. */
  bool begin(int pwrkeyPin = A7683E_NO_PIN,
             int resetPin  = A7683E_NO_PIN,
             int netlightPin = A7683E_NO_PIN,
             unsigned long baud = 115200);

  /* Uses another Stream instead of UART4. */
  bool beginOn(Stream &io, int pwrkeyPin = A7683E_NO_PIN,
               int resetPin = A7683E_NO_PIN, int netlightPin = A7683E_NO_PIN);

  void end();

  void setPins(int pwrkeyPin, int resetPin, int netlightPin);

  int pwrkeyPin()   const { return _pwrkey; }
  int resetPin()    const { return _reset; }
  int netlightPin() const { return _netlight; }

  /* Power and reset */

  /* Holds PWRKEY low for ms. 500 ms or more switches the module on, 2.5 s or
   * more switches it off. */
  void pulsePowerKey(uint16_t ms = 1000);

  /* Holds RESET low for ms, 100 ms or more. */
  void pulseReset(uint16_t ms = 300);

  /* Pulses PWRKEY up to attempts times until the module answers AT. Returns
   * true at once when it already answers. */
  bool powerOn(uint32_t timeoutMs = 40000, int attempts = 3);

  /* AT+CPOF, then a long PWRKEY pulse if that fails. */
  bool powerOff(uint32_t timeoutMs = 10000);

  /* Pulses RESET, then waits for the module to answer AT. */
  bool reset(uint32_t timeoutMs = 40000);

  bool isAlive(uint32_t timeoutMs = 1000);

  /* NETLIGHT */

  /* Samples NETLIGHT for windowMs and classifies the blink period. Blocks for
   * the window. */
  A7683ENetlight netlightState(uint16_t windowMs = 2200);

  /* Last blink period in ms, 0 when steady. */
  uint16_t netlightPeriodMs() const { return _netPeriodMs; }

  static const char *netlightName(A7683ENetlight s);

  /* AT plumbing */

  void   flushInput();
  size_t writeRaw(const char *s);
  size_t writeBytes(const uint8_t *d, size_t n);

  bool   sendAT(const char *cmd, uint32_t timeoutMs = 3000);
  bool   sendATExpect(const char *cmd, const char *expect, uint32_t timeoutMs = 3000);
  int    readLine(char *out, size_t outLen, uint32_t timeoutMs);
  const char *lastLine() const { return _line; }

  /* Serial speed */

  /* Tries each candidate rate and leaves UART4 on the one that answers. The
   * A76XX saves AT+IPR, so a changed rate survives a power cycle. */
  bool findBaud(uint32_t *foundBaud = NULL);
  uint32_t baud() const { return _baud; }

  /* Moves the module and UART4 to baud. Calls findBaud() first. */
  bool syncBaud(uint32_t baud);

  /* Status */

  bool waitForReady(uint32_t timeoutMs = 20000);
  bool simReady(uint32_t timeoutMs = 5000);

  int  signalQuality();                 /* +CSQ rssi 0..31, 99 unknown, -1 on error */
  int  signalDbm();                     /* 0 when unknown */
  int  registrationStatus();            /* +CREG stat, -1 on error */
  bool isRegistered();                  /* stat 1 home or 5 roaming */
  bool waitForNetwork(uint32_t timeoutMs = 90000);

  bool getIMEI(char *out, size_t outLen);
  bool getModel(char *out, size_t outLen);
  bool getOperator(char *out, size_t outLen);

  /* Packet data */

  bool setApn(const char *apn);

  /* AT+CGAUTH. authType: 0 none, 1 PAP, 2 CHAP. */
  bool setApnAuth(const char *user, const char *pass, int authType = 1);

  bool attachData(const char *apn, uint32_t timeoutMs = 60000);
  bool attachData(const char *apn, const char *user, const char *pass,
                  uint32_t timeoutMs = 60000);
  bool getLocalIP(char *out, size_t outLen);

  /* HTTP */

  bool httpBegin(const char *url, const char *contentType = "application/octet-stream");
  bool httpPost(const uint8_t *body, size_t len, uint32_t timeoutMs = 90000);
  int  httpStatus() const { return _httpStatus; }
  void httpEnd();

  void setDebug(Print *out) { _dbg = out; }
  Stream &stream() { return *_io; }

private:
  Stream  *_io;
  Print   *_dbg;
  int      _pwrkey;
  int      _reset;
  int      _netlight;
  int      _httpStatus;
  bool     _httpOpen;
  uint32_t _baud;
  uint16_t _netPeriodMs;
  char     _line[A7683E_LINE_MAX];

  void attachControlPins();
  int  waitResponse(const char *expect, uint32_t timeoutMs);
  bool waitPrompt(uint32_t timeoutMs);
  bool queryLine(const char *cmd, char *out, size_t outLen, uint32_t timeoutMs);
};

extern A7683EClass A7683E;

#endif /* A7683E_H */
