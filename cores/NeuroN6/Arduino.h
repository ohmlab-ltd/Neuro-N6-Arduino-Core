/**
  ******************************************************************************
  * @file    Arduino.h
  * @brief   Core Arduino header for the Neuro N6 board.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025-2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  *
  ******************************************************************************
  */

#pragma once

#include <stdint.h>

#include "NeuroN6.h"

#include "pins_arduino.h"

#ifdef __cplusplus
#include "NeuroN6_RGB.h"
#endif

#include "wiring_constants.h"

#include "wiring_digital.h"
#include "wiring_analog.h"
#include "WInterrupts.h"
#include "wiring_pulse.h"
#include "wiring_shift.h"

/* ArduinoCore-API 1.5.2 classes, LGPL-2.1: see api/LICENSE.txt. */
#ifdef __cplusplus
#include "api/String.h"
#include "api/Printable.h"
#include "api/Print.h"
#include "api/Stream.h"
#include "api/WCharacter.h"
using arduino::__FlashStringHelper;
using arduino::String;
using arduino::StringSumHelper;
using arduino::Printable;
using arduino::Print;
using arduino::Stream;

long random(long howbig);
long random(long howsmall, long howbig);
void randomSeed(unsigned long seed);
long map(long x, long in_min, long in_max, long out_min, long out_max);
#endif

#ifdef __cplusplus
#include "IPAddress.h"
#include "Client.h"
#include "Server.h"
#include "Udp.h"
#include "HardwareSerial.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

uint32_t millis(void);
uint32_t micros(void);
void     delay(uint32_t ms);
void     delayMicroseconds(uint32_t us);
void     yield(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
void setup(void);
void loop(void);
#endif