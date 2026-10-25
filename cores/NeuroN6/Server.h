/**
 ******************************************************************************
 * @file    Server.h
 * @brief   Arduino's Server interface.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#pragma once

#include "api/Print.h"

namespace arduino {

class Server : public Print {
public:
  virtual void begin() = 0;
};

} /* namespace arduino */

using arduino::Server;
