/**
 ******************************************************************************
 * @file    MayaBLE.h
 * @brief   A non-connectable BLE beacon over ArduinoBLE.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef MAYA_BLE_H
#define MAYA_BLE_H

#include <ArduinoBLE.h>

#ifdef __cplusplus

class MayaBLE_t
{
public:
  MayaBLE_t() = default;

  /* Advertises a beacon called name and returns true once it is advertising.
   * bringUpModule is ignored. */
  bool begin(const char *name = "Neuro-N6", bool bringUpModule = true);

  bool isAdvertising(void) const;

  /* One line on the USB console: advertising or not, and the address. */
  void printDiag(void) const;

private:
  bool started = false;
};

extern MayaBLE_t MayaBLE;

#endif /* __cplusplus */

#endif /* MAYA_BLE_H */
