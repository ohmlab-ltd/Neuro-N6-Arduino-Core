/**
 ******************************************************************************
 * @file    NeuroN6_RGB.h
 * @brief   On-board RGB LED: the global RGB object.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#ifndef NEURON6_RGB_H
#define NEURON6_RGB_H

#include "pins_arduino.h"

#ifdef __cplusplus

/* Red PE11, green PD6, blue PC1, active low. RGB switches each on or off;
 * analogWrite() on LEDR, LEDG and LEDB dims them. */
class NeuroN6RGB {
 public:
  /* Empty: the core configures the pins. */
  void begin() {}

  /* true is lit. */
  void set(bool r, bool g, bool b) {
    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin,
                      r ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin,
                      g ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin,
                      b ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }

  void off()     { set(false, false, false); }
  void red()     { set(true,  false, false); }
  void green()   { set(false, true,  false); }
  void blue()    { set(false, false, true ); }
  void yellow()  { set(true,  true,  false); }
  void cyan()    { set(false, true,  true ); }
  void magenta() { set(true,  false, true ); }
  void white()   { set(true,  true,  true ); }
};

extern NeuroN6RGB RGB;

#endif /* __cplusplus */
#endif /* NEURON6_RGB_H */
