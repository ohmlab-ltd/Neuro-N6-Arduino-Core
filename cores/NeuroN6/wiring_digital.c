/**
 ******************************************************************************
 * @file    wiring_digital.c
 * @brief   pinMode / digitalWrite / digitalRead / digitalToggle.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "wiring_digital.h"

const PinDescription *pin_desc(uint32_t pin)
{
  if (pin >= PINS_COUNT) { return NULL; }
  const PinDescription *d = &g_APinDescription[pin];
  return (d->port == NULL) ? NULL : d;
}

void pin_enable_gpio_clock(const GPIO_TypeDef *port)
{
  if      (port == GPIOA) { __HAL_RCC_GPIOA_CLK_ENABLE(); }
  else if (port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
  else if (port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
  else if (port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
  else if (port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
  else if (port == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); }
  else if (port == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); }
  else if (port == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); }
  else if (port == GPION) { __HAL_RCC_GPION_CLK_ENABLE(); }
}

void pinMode(uint32_t pin, uint32_t mode)
{
  const PinDescription *d = pin_desc(pin);
  if (d == NULL) { return; }

  pin_enable_gpio_clock(d->port);

  GPIO_InitTypeDef init = {0};
  init.Pin   = d->mask;
  init.Speed = GPIO_SPEED_FREQ_LOW;   /* slow edges: less crosstalk near the camera lanes */
  switch (mode) {
    case INPUT:
      init.Mode = GPIO_MODE_INPUT;
      init.Pull = GPIO_NOPULL;
      break;
    case INPUT_PULLUP:
      init.Mode = GPIO_MODE_INPUT;
      init.Pull = GPIO_PULLUP;
      break;
    case INPUT_PULLDOWN:
      init.Mode = GPIO_MODE_INPUT;
      init.Pull = GPIO_PULLDOWN;
      break;
    case OUTPUT_OPENDRAIN:
      init.Mode = GPIO_MODE_OUTPUT_OD;
      init.Pull = GPIO_NOPULL;
      break;
    case OUTPUT:
    default:
      init.Mode = GPIO_MODE_OUTPUT_PP;
      init.Pull = GPIO_NOPULL;
      break;
  }
  HAL_GPIO_Init(d->port, &init);
}

void digitalWrite(uint32_t pin, uint32_t val)
{
  const PinDescription *d = pin_desc(pin);
  if (d == NULL) { return; }
  /* BSRR: lower 16 bits set, upper 16 bits reset, in one atomic store. */
  d->port->BSRR = (val != LOW) ? (uint32_t)d->mask
                               : ((uint32_t)d->mask << 16);
}

int digitalRead(uint32_t pin)
{
  const PinDescription *d = pin_desc(pin);
  if (d == NULL) { return LOW; }
  return ((d->port->IDR & d->mask) != 0u) ? HIGH : LOW;
}

void digitalToggle(uint32_t pin)
{
  const PinDescription *d = pin_desc(pin);
  if (d == NULL) { return; }
  /* Not atomic when two contexts toggle the same pin at the same time. */
  uint32_t odr = d->port->ODR;
  d->port->BSRR = ((odr & d->mask) << 16) | (~odr & d->mask);
}

int pinHasPwm(uint32_t pin)
{
  const PinDescription *d = pin_desc(pin);
  return (d != NULL && d->pwm_tim != NULL) ? 1 : 0;
}
