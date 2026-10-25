/**
 * @file    NeuroN6_PowerSave.h
 * @brief   Sleep modes, clock profiles and peripheral power-down.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef NEURON6_POWERSAVE_H
#define NEURON6_POWERSAVE_H

#ifdef __cplusplus

#include <Arduino.h>
#include <stdint.h>

/* Clock profiles. AXI, HCLK and the APB buses stay on PLL4 in all of them. */
enum PsClockProfile {
  PS_CLOCK_FULL   = 0,  /* CPU 800 MHz, IC1 = PLL1/1. PLL2 and PLL3 on. */
  PS_CLOCK_CRUISE = 1,  /* CPU 400 MHz, IC1 = PLL1/2. PLL2 and PLL3 off. */
  PS_CLOCK_LOW    = 2,  /* CPU 100 MHz, IC1 = PLL1/8. PLL2 and PLL3 off. */
  PS_CLOCK_ECO    = 3,  /* CPU 64 MHz on HSI. PLL1, PLL2 and PLL3 off. No LTE. */
};

/* periphOff() and periphOn() mask bits. */
#define PS_PERIPH_DCMIPP  (1u << 0)   /* camera pixel pipeline */
#define PS_PERIPH_CSI     (1u << 1)   /* MIPI CSI-2 host and D-PHY kernel */
#define PS_PERIPH_I2C1    (1u << 2)   /* sensor and camera I2C bus */
#define PS_PERIPH_MDF1    (1u << 3)   /* microphone digital filter */
#define PS_PERIPH_CAMERA  (PS_PERIPH_DCMIPP | PS_PERIPH_CSI)

/* Phase marker pins for a power analyser's digital channels. */
enum PsMarker {
  PS_MARKER_AWAKE   = 0,   /* default D5, PE9. High while out of Stop. */
  PS_MARKER_CAMERA  = 1,   /* default D6, PF5 */
  PS_MARKER_THERMAL = 2,   /* default D9, PD12 */
  PS_MARKER_AUX     = 3,   /* default D10, PF10 */
};
#define PS_MARKER_COUNT 4

class NeuroN6PowerSave {
public:
  /* Turns the run LED off and prepares the RTC wakeup. Call once in setup(). */
  void begin();

  /* Clock control */

  /* Returns false if an RCC step failed, leaving the CPU on HSI. */
  bool setClockProfile(PsClockProfile p);
  PsClockProfile clockProfile() const { return _profile; }

  /* CPU = 800 MHz / div, div 1 to 256, until the next setClockProfile(). */
  bool setCpuDivider(uint32_t div);

  uint32_t cpuClockHz() const;

  /* RTC clock in Hz, read from the RTC prescalers. */
  uint32_t rtcClockHz() const { return _rtcClkHz; }

  /* Milliseconds of day from the RTC. Wraps at midnight, so differences are
   * taken modulo 86400000. */
  uint32_t rtcTimeMs();

  /* Powers off AXISRAM3 to AXISRAM6, the NPU's memory. Contents are lost: no
   * model may be loaded. */
  void npuRamOff();

  /* Puts the PSRAM into deep power down until reboot, or returns false. Its
   * contents are lost. XSPI2 leaves memory-mapped mode, so an access to
   * 0x70000000 then faults. */
  bool psramSleep();

  /* Puts the NOR flash into deep power down until reboot. XSPI1 leaves
   * memory-mapped mode, so an access to 0x90000000 then faults. */
  bool norSleep();

  /* Sets every pin still in input mode to analog with no pull and returns the
   * count. Call after all peripherals are set up. */
  uint32_t gpioFloatingToAnalog();

  /* External window watchdog, kicked by a falling edge on its WDI pin */

  /* Starts a falling edge on pin every targetMs, 0 meaning the window
   * midpoint. The target is clamped to windowLoMs x 33600/32000 and
   * windowHiMs x 28800/32000. */
  void externalWatchdogBegin(uint32_t pin, uint32_t windowLoMs, uint32_t windowHiMs,
                             uint32_t targetMs = 0);
  /* Drives the supervisor's SET1 line low, which disables the watchdog. The
   * pin needs an external pull-down of 2.2 k ohm. */
  void externalWatchdogArmPin(uint32_t pin);
  /* Enables the watchdog. Call after externalWatchdogBegin(). */
  void externalWatchdogArm();

  /* Stops every kick for good, so the supervisor resets the board. */
  void externalWatchdogPause();

  /* Kicks stop when no externalWatchdogHeartbeat() arrives within timeoutMs.
   * 0 disables the check. */
  void externalWatchdogHeartbeatTimeout(uint32_t timeoutMs);
  void externalWatchdogHeartbeat();

  /* Kicks only when the target interval has elapsed. */
  void externalWatchdogService();
  /* Kicks without the interval check. */
  void externalWatchdogKick();
  /* Target interval between falling edges in ms, 0 when disabled. */
  uint32_t externalWatchdogKickMs() const { return _wdKickMs; }

  /* Turns PLL2 and PLL3 off, moves IC6 and IC11 to PLL4/4 and calls
   * npuRamOff(). PS_CLOCK_FULL does not turn them back on. */
  void npuOff();

  /* Peripheral gating */
  void periphOff(uint32_t mask);
  void periphOn(uint32_t mask);

  /* Disconnects USB and turns the PHY supply off until reboot. */
  void usbOff();

  /* Board sensors */
  void sensorsSleep();     /* imuSleep(), magSleep() and micSleep() */
  void imuSleep();         /* LSM6DSL: accelerometer and gyroscope off */
  void magSleep();         /* MMC5603NJ: continuous mode off */
  void micSleep();         /* MDF1 clock off, PDM clock pin driven low */

  /* FLIR Lepton power: PWR_DWN_L on PD3, RESET_L on PD9. The first CCI
   * transaction must wait about 1 s after power-up. */
  void thermalPower(bool on);

  /* Sleep modes */

  /* Stop mode for ms milliseconds on the RTC wakeup, called from the sketch
   * task only. False if another interrupt ended it early, or if a USB host is
   * attached: then it sleeps with sleepLight(). */
  bool sleepStop(uint32_t ms);

  /* WFI sleep for ms milliseconds. Tasks and USB keep running. */
  void sleepLight(uint32_t ms);

  /* Standby for about ms milliseconds, at most 100000. The IWDG times it on
   * the LSI, +/-5 %, and its reset wakes the board, so RAM is lost and the
   * call does not return. */
  void standby(uint32_t ms);

  /* Longest single Stop entry, 500 to 6500 ms. The IWDG is fed between
   * entries. */
  void setStopChunkMs(uint32_t ms) { _chunkMs = (ms < 500u) ? 500u : (ms > 6500u ? 6500u : ms); }
  uint32_t stopChunkMs() const     { return _chunkMs; }
  /* true, the default, selects SVOS low: lower power, longer wake. */
  void setStopSvosLow(bool low)    { _svosLow = low; }

  /* Makes the idle task execute WFI. Call at the end of setup(), after all
   * bring-up. */
  void idleWfiEnable();
  void idleWfiDisable();

  /* Phase markers */
  void markerConfig(PsMarker m, uint32_t arduinoPin);  /* before markersBegin() */
  void markersBegin();                                 /* pins to output, low */
  void markerSet(PsMarker m, bool level);
  void markerPulse(PsMarker m, uint32_t microseconds);

  /* Blinks an active-low LED pin and blocks for ms. */
  void ledBlink(uint32_t ledPin, uint32_t ms = 15u);

private:
  bool applyClockTree(PsClockProfile p, uint32_t cpuDiv);
  bool wdHeartbeatLost();
  void rtcPrepare();
  void iwdgFeed();
  uint32_t wutTicksForMs(uint32_t ms) const;

  PsClockProfile _profile   = PS_CLOCK_FULL;
  uint32_t       _cpuDiv    = 1;       /* IC1 divider */
  uint32_t       _chunkMs   = 6000;    /* longest Stop entry */
  bool           _svosLow   = true;
  bool           _npuOff    = false;
  bool           _begun     = false;
  bool           _rtcReady  = false;
  bool           _markersOn = false;
  uint32_t       _rtcClkHz  = 32000;   /* LSI default */
  uint32_t       _markerPin[PS_MARKER_COUNT] = { 5u /*D5*/, 6u /*D6*/, 9u /*D9*/, 10u /*D10*/ };

  /* External watchdog. _wdPort NULL means disabled. */
  GPIO_TypeDef  *_wdPort    = nullptr;
  uint16_t       _wdMask    = 0;
  uint32_t       _wdKickMs  = 0;   /* target falling-edge interval */
  uint32_t       _wdLastMs  = 0;   /* RTC ms of the last falling edge */
  GPIO_TypeDef  *_wdArmPort = nullptr;  /* SET1 line, NULL = not used */
  uint16_t       _wdArmMask = 0;
  bool           _wdArmed   = false;
  bool           _wdPaused  = false;
  uint32_t       _wdBeatMs  = 0;   /* RTC ms of the last heartbeat */
  uint32_t       _wdBeatTo  = 0;   /* heartbeat timeout, 0 = not checked */
};

extern NeuroN6PowerSave PowerSave;

#endif /* __cplusplus */
#endif /* NEURON6_POWERSAVE_H */
