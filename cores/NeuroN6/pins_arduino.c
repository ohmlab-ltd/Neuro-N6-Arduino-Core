/**
 ******************************************************************************
 * @file    pins_arduino.c
 * @brief   Pin table: Arduino pin number to GPIO, ADC and PWM routing.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "pins_arduino.h"
#include "wiring_digital.h"

/* NOT_A_PIN slot. */
#define NC { NULL, 0u, 0u, 0xFFu, 0xFFu, NULL, 0u, 0u }

/* ADC and PWM columns from the ST STM32N657I0HxQ pin database. */
const PinDescription g_APinDescription[PINS_COUNT] = {
  /*  0 D0/RX  PD0  */ { GPIOD, GPIO_PIN_0,  0u,  0xFFu, 0xFFu, NULL,  0u, 0u },
  /*  1 D1/TX  PD1  */ { GPIOD, GPIO_PIN_1,  1u,  0xFFu, 0xFFu, NULL,  0u, 0u },
  /*  2 - */ NC,
  /*  3 - */ NC,
  /*  4 - */ NC,
  /*  5 D5     PE9  */ { GPIOE, GPIO_PIN_9,  9u,  0xFFu, 0xFFu, TIM1,  1u, 1u },
  /*  6 D6     PF5  */ { GPIOF, GPIO_PIN_5,  5u,  0xFFu, 0xFFu, NULL,  0u, 0u },
  /*  7 - */ NC,
  /*  8 - */ NC,
  /*  9 D9     PD12 */ { GPIOD, GPIO_PIN_12, 12u, 0xFFu, 0xFFu, NULL,  0u, 0u },
  /* 10 D10    PF10 */ { GPIOF, GPIO_PIN_10, 10u, 0xFFu, 0xFFu, NULL,  0u, 0u },
  /* 11 D11    PF15 */ { GPIOF, GPIO_PIN_15, 15u, 0xFFu, 0xFFu, NULL,  0u, 0u },
  /* 12 D12    PA3  */ { GPIOA, GPIO_PIN_3,  3u,  0xFFu, 0xFFu, NULL,  0u, 0u },
  /* 13 D13    PG14 */ { GPIOG, GPIO_PIN_14, 14u, 0xFFu, 0xFFu, NULL,  0u, 0u },
  /* 14 A0     PF13 */ { GPIOF, GPIO_PIN_13, 13u, 2u,    2u,    NULL,  0u, 0u },
  /* 15 A1     PF11 */ { GPIOF, GPIO_PIN_11, 11u, 1u,    2u,    NULL,  0u, 0u },
  /* 16 A2     PF12 */ { GPIOF, GPIO_PIN_12, 12u, 1u,    6u,    NULL,  0u, 0u },
  /* 17 A3     PF3  */ { GPIOF, GPIO_PIN_3,  3u,  1u,    16u,   NULL,  0u, 0u },
  /* 18 A4     PF14 */ { GPIOF, GPIO_PIN_14, 14u, 2u,    6u,    TIM2,  2u, 1u },
  /* 19 A5     PG15 */ { GPIOG, GPIO_PIN_15, 15u, 1u,    7u,    NULL,  0u, 0u },
  /* 20 SDA    PD15 */ { GPIOD, GPIO_PIN_15, 15u, 0xFFu, 0xFFu, NULL,  0u, 0u },
  /* 21 SCL    PD14 */ { GPIOD, GPIO_PIN_14, 14u, 0xFFu, 0xFFu, NULL,  0u, 0u },
  /* 22 MOSI   PD7  */ { GPIOD, GPIO_PIN_7,  7u,  0xFFu, 0xFFu, NULL,  0u, 0u },
  /* 23 MISO   PD11 */ { GPIOD, GPIO_PIN_11, 11u, 0xFFu, 0xFFu, NULL,  0u, 0u },
  /* 24 SCK    PF2  */ { GPIOF, GPIO_PIN_2,  2u,  0xFFu, 0xFFu, NULL,  0u, 0u },
  /* 25 LEDR   PE11 */ { GPIOE, GPIO_PIN_11, 11u, 0xFFu, 0xFFu, TIM1,  2u, 1u },
  /* 26 LEDG   PD6  */ { GPIOD, GPIO_PIN_6,  6u,  0xFFu, 0xFFu, TIM15, 2u, 4u },
  /* 27 LEDB   PC1  */ { GPIOC, GPIO_PIN_1,  1u,  0xFFu, 0xFFu, TIM17, 1u, 1u },
  /* 28 USER   PG8  */ { GPIOG, GPIO_PIN_8,  8u,  0xFFu, 0xFFu, NULL,  0u, 0u },
};
