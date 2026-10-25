/**
 ******************************************************************************
 * @file    MayaBLE.cpp
 * @brief   The MayaBLE API on top of ArduinoBLE.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "MayaBLE.h"

#include <stdio.h>

extern "C" void NeuroN6_DbgPrint(const char *msg);

MayaBLE_t MayaBLE;

bool MayaBLE_t::begin(const char *name, bool bringUpModule)
{
  (void)bringUpModule;
  if (!BLE.begin()) {
    NeuroN6_DbgPrint("[MayaBLE] BLE.begin() failed.\r\n");
    return false;
  }
  BLE.setLocalName(name);
  BLE.setDeviceName(name);
  BLE.setConnectable(false);
  started = BLE.advertise() != 0;
  printDiag();
  return started;
}

bool MayaBLE_t::isAdvertising(void) const
{
  return started;
}

void MayaBLE_t::printDiag(void) const
{
  char line[96];
  snprintf(line, sizeof(line), "[MayaBLE] %s, address %s (ArduinoBLE underneath).\r\n",
           started ? "advertising" : "not advertising", BLE.address().c_str());
  NeuroN6_DbgPrint(line);
}
