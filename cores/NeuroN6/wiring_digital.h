/**
 ******************************************************************************
 * @file    wiring_digital.h
 * @brief   Arduino digital I/O and the pin table type.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef WIRING_DIGITAL_H
#define WIRING_DIGITAL_H

#include <stdint.h>
#include "stm32n6xx_hal.h"
#include "wiring_constants.h"
#include "pins_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

/* One row per Arduino pin number. */
typedef struct {
  GPIO_TypeDef *port;      /* GPIOx_S alias, NULL = NC */
  uint16_t      mask;      /* GPIO_PIN_x bit mask */
  uint8_t       pin;       /* pin index 0..15, also the EXTI line */
  uint8_t       adc_num;   /* 1 or 2, 0xFF = not an ADC pin */
  uint8_t       adc_chan;  /* INPx channel number, 0xFF = none */
  TIM_TypeDef  *pwm_tim;   /* NULL = analogWrite falls back to digitalWrite */
  uint8_t       pwm_chan;  /* timer channel number 1..4 */
  uint8_t       pwm_af;    /* GPIO alternate-function number for that timer */
} PinDescription;

extern const PinDescription g_APinDescription[PINS_COUNT];

void     pinMode(uint32_t pin, uint32_t mode);
void     digitalWrite(uint32_t pin, uint32_t val);
int      digitalRead(uint32_t pin);
void     digitalToggle(uint32_t pin);
int      pinHasPwm(uint32_t pin);

/* Internal helpers. */
const PinDescription *pin_desc(uint32_t pin);   /* NULL if invalid or NC */
void pin_enable_gpio_clock(const GPIO_TypeDef *port);

#ifdef __cplusplus
}
#endif

#endif /* WIRING_DIGITAL_H */
