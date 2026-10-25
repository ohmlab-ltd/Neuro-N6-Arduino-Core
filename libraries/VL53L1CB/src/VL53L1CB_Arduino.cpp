/**
 ******************************************************************************
 * @file    VL53L1CB_Arduino.cpp
 * @brief   VL53L1CB class.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#include <VL53L1CB_Arduino.h>
#include "NeuroN6_FrameQueue.h"

bool VL53L1CB::begin(uint8_t addr, TwoWire *theWire)
{
  _addr = addr;
  _wire = theWire;
  return (VL53L1CB_Setup4x4FrameCapture() == VL53L1CB_OK);
}

uint16_t VL53L1CB::whoami()
{
  return VL53L1CB_WhoAmI();
}

tofFrame VL53L1CB::read()
{
  VL53L1CB_CaptureNextFrameZoneOnce();   /* does not block */
  tofFrame f;
  VL53L1CB_GetFrameMm(f.mm);
  return f;
}

void VL53L1CB::sendNS()
{
  NeuroN6_StreamUsbOnlyStart();          /* idempotent */
  VL53L1CB_CaptureNextFrameZoneOnce();
  vnd_meta_build_tof();                  /* VND_META_SLOT_TOF */
}
