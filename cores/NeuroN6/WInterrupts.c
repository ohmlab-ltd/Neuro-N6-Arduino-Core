/**
 ******************************************************************************
 * @file    WInterrupts.c
 * @brief   attachInterrupt()/detachInterrupt() over EXTI lines 0..15.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include <stdio.h>
#include "WInterrupts.h"
#include "wiring_digital.h"

extern void NeuroN6_DbgPrint(const char *msg);

static void winterrupts_warn(const char *what, uint32_t pin, uint32_t extra)
{
  char msg[120];
  snprintf(msg, sizeof msg, "[WIRING] attachInterrupt(%lu): %s (%lu)\r\n",
           (unsigned long)pin, what, (unsigned long)extra);
  NeuroN6_DbgPrint(msg);
}

/* Numerically above configMAX_SYSCALL_INTERRUPT_PRIORITY, so callbacks may
 * use the FreeRTOS FromISR APIs. */
#define EXTI_NVIC_PRIO 8u

typedef struct {
  voidFuncPtr cb;       /* NULL = line free */
  uint32_t    pin;      /* owning Arduino pin number */
} exti_slot_t;

static volatile exti_slot_t s_exti[16];

static const IRQn_Type s_exti_irqn[16] = {
  EXTI0_IRQn,  EXTI1_IRQn,  EXTI2_IRQn,  EXTI3_IRQn,
  EXTI4_IRQn,  EXTI5_IRQn,  EXTI6_IRQn,  EXTI7_IRQn,
  EXTI8_IRQn,  EXTI9_IRQn,  EXTI10_IRQn, EXTI11_IRQn,
  EXTI12_IRQn, EXTI13_IRQn, EXTI14_IRQn, EXTI15_IRQn,
};

void attachInterrupt(uint32_t pin, voidFuncPtr callback, uint32_t mode)
{
  const PinDescription *d = pin_desc(pin);
  if (d == NULL || callback == NULL) { return; }

  uint32_t line = d->pin;

  if (s_exti[line].cb != NULL && s_exti[line].pin != pin) {
    winterrupts_warn("EXTI line in use by another port's pin",
                     pin, s_exti[line].pin);
    return;
  }

  uint32_t hal_mode;
  switch (mode) {
    case RISING:  hal_mode = GPIO_MODE_IT_RISING;         break;
    case FALLING: hal_mode = GPIO_MODE_IT_FALLING;        break;
    case CHANGE:  hal_mode = GPIO_MODE_IT_RISING_FALLING; break;
    default:
      winterrupts_warn("unsupported mode; use RISING, FALLING or CHANGE",
                       pin, mode);
      return;
  }

  pin_enable_gpio_clock(d->port);

  uint32_t pupdr = (d->port->PUPDR >> (2u * line)) & 0x3u;

  GPIO_InitTypeDef init = {0};
  init.Pin   = d->mask;
  init.Mode  = hal_mode;
  init.Pull  = (pupdr == 1u) ? GPIO_PULLUP
             : (pupdr == 2u) ? GPIO_PULLDOWN
                             : GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_LOW;

  s_exti[line].pin = pin;
  s_exti[line].cb  = callback;

  HAL_GPIO_Init(d->port, &init);   /* routes EXTICR and unmasks the line */

  HAL_NVIC_SetPriority(s_exti_irqn[line], EXTI_NVIC_PRIO, 0);
  HAL_NVIC_EnableIRQ(s_exti_irqn[line]);
}

void detachInterrupt(uint32_t pin)
{
  const PinDescription *d = pin_desc(pin);
  if (d == NULL) { return; }

  uint32_t line = d->pin;
  if (s_exti[line].cb == NULL || s_exti[line].pin != pin) { return; }

  HAL_NVIC_DisableIRQ(s_exti_irqn[line]);
  s_exti[line].cb = NULL;

  /* Input mode masks the EXTI line. */
  uint32_t pupdr = (d->port->PUPDR >> (2u * line)) & 0x3u;
  GPIO_InitTypeDef init = {0};
  init.Pin   = d->mask;
  init.Mode  = GPIO_MODE_INPUT;
  init.Pull  = (pupdr == 1u) ? GPIO_PULLUP
             : (pupdr == 2u) ? GPIO_PULLDOWN
                             : GPIO_NOPULL;
  init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(d->port, &init);
}

static inline void exti_dispatch(uint32_t line)
{
  uint32_t mask = 1UL << line;
  uint32_t pend = (EXTI->RPR1 & mask) | (EXTI->FPR1 & mask);
  if (pend == 0u) { return; }
  EXTI->RPR1 = mask;   /* write 1 to clear */
  EXTI->FPR1 = mask;

  voidFuncPtr cb = s_exti[line].cb;
  if (cb != NULL) { cb(); }
}

void EXTI0_IRQHandler(void)  { exti_dispatch(0);  }
void EXTI1_IRQHandler(void)  { exti_dispatch(1);  }
void EXTI2_IRQHandler(void)  { exti_dispatch(2);  }
void EXTI3_IRQHandler(void)  { exti_dispatch(3);  }
void EXTI4_IRQHandler(void)  { exti_dispatch(4);  }
void EXTI5_IRQHandler(void)  { exti_dispatch(5);  }
void EXTI6_IRQHandler(void)  { exti_dispatch(6);  }
void EXTI7_IRQHandler(void)  { exti_dispatch(7);  }
void EXTI8_IRQHandler(void)  { exti_dispatch(8);  }
void EXTI9_IRQHandler(void)  { exti_dispatch(9);  }
void EXTI10_IRQHandler(void) { exti_dispatch(10); }
void EXTI11_IRQHandler(void) { exti_dispatch(11); }
void EXTI12_IRQHandler(void) { exti_dispatch(12); }
void EXTI13_IRQHandler(void) { exti_dispatch(13); }
void EXTI14_IRQHandler(void) { exti_dispatch(14); }
void EXTI15_IRQHandler(void) { exti_dispatch(15); }
