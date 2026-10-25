/**
 * @file    a7683e_uart.c
 * @brief   UART4 and RX ring for the A7683E modem link.
 * @copyright Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "a7683e_uart.h"
#include "stm32n6xx_hal.h"

/* A power of two: the head and tail wrap with a mask. */
#define A7683E_RX_SIZE  1024u
#define A7683E_RX_MASK  (A7683E_RX_SIZE - 1u)

static UART_HandleTypeDef s_uart;
static volatile uint8_t   s_ring[A7683E_RX_SIZE];
static volatile uint16_t  s_head;      /* written by the ISR only */
static volatile uint16_t  s_tail;      /* written by the task only */
static volatile uint32_t  s_dropped;
static volatile uint32_t  s_rx_total;
static bool               s_open;

/* UART4's kernel clock is IC9, shared with the BLE link on USART6, so
 * this UART starts first: re-muxing IC9 can clip a byte on that link. Not done
 * in HAL_UART_MspInit, which ArduinoBLE's MAYA-W476 transport defines. */
static void a7683e_uart_hw_init(void)
{
  RCC_PeriphCLKInitTypeDef clk  = {0};
  GPIO_InitTypeDef         gpio = {0};

  clk.PeriphClockSelection = RCC_PERIPHCLK_UART4;
  clk.Uart4ClockSelection  = RCC_UART4CLKSOURCE_IC9;
  clk.ICSelection[RCC_IC9].ClockSelection = RCC_ICCLKSOURCE_PLL1;
  clk.ICSelection[RCC_IC9].ClockDivider   = 16;
  (void)HAL_RCCEx_PeriphCLKConfig(&clk);

  __HAL_RCC_UART4_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* PD1 = UART4_TX */
  gpio.Pin       = GPIO_PIN_1;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOD, &gpio);

  /* PD0 = UART4_RX, pulled up so an absent modem reads idle-high. */
  gpio.Pin  = GPIO_PIN_0;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &gpio);
}

bool a7683e_uart_begin(uint32_t baud)
{
  if (s_open) {
    a7683e_uart_end();
  }

  a7683e_uart_hw_init();

  s_uart.Instance                    = UART4;
  s_uart.Init.BaudRate               = baud;
  s_uart.Init.WordLength             = UART_WORDLENGTH_8B;
  s_uart.Init.StopBits               = UART_STOPBITS_1;
  s_uart.Init.Parity                 = UART_PARITY_NONE;
  s_uart.Init.Mode                   = UART_MODE_TX_RX;
  s_uart.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  s_uart.Init.OverSampling           = UART_OVERSAMPLING_16;
  s_uart.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  s_uart.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
  s_uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&s_uart) != HAL_OK) {
    return false;
  }

  /* 8-deep RX FIFO, interrupt when non-empty. */
  (void)HAL_UARTEx_SetRxFifoThreshold(&s_uart, UART_RXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_SetTxFifoThreshold(&s_uart, UART_TXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_EnableFifoMode(&s_uart);

  s_head     = 0u;
  s_tail     = 0u;
  s_dropped  = 0u;
  s_rx_total = 0u;
  s_open     = true;

  /* Below SDMMC and vision, above the FreeRTOS syscall ceiling. */
  HAL_NVIC_SetPriority(UART4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);

  __HAL_UART_ENABLE_IT(&s_uart, UART_IT_RXNE);   /* UART_IT_RXFNE with the FIFO on */
  return true;
}

void a7683e_uart_end(void)
{
  if (!s_open) {
    return;
  }
  __HAL_UART_DISABLE_IT(&s_uart, UART_IT_RXNE);
  HAL_NVIC_DisableIRQ(UART4_IRQn);
  (void)HAL_UART_DeInit(&s_uart);
  s_open = false;
}

size_t a7683e_uart_write(const uint8_t *data, size_t len)
{
  if (!s_open || data == NULL || len == 0u) {
    return 0u;
  }
  /* HAL_UART_Transmit counts in 16 bits, so a longer write goes in parts. */
  size_t sent = 0u;
  while (sent < len) {
    const uint16_t n = (len - sent > 0xFFFFu) ? 0xFFFFu : (uint16_t)(len - sent);
    /* 10 bits per byte, plus slack. */
    const uint32_t timeout = 100u + (uint32_t)(((uint32_t)n * 10u * 1000u) / s_uart.Init.BaudRate) + 10u;
    if (HAL_UART_Transmit(&s_uart, data + sent, n, timeout) != HAL_OK) {
      break;
    }
    sent += n;
  }
  return sent;
}

int a7683e_uart_available(void)
{
  uint16_t head = s_head;      /* one load, atomic against the ISR */
  return (int)((uint16_t)(head - s_tail) & A7683E_RX_MASK);
}

int a7683e_uart_read(void)
{
  if (s_head == s_tail) {
    return -1;
  }
  uint8_t b = s_ring[s_tail];
  s_tail = (uint16_t)((s_tail + 1u) & A7683E_RX_MASK);
  return (int)b;
}

int a7683e_uart_peek(void)
{
  if (s_head == s_tail) {
    return -1;
  }
  return (int)s_ring[s_tail];
}

void     a7683e_uart_discard(void)   { s_tail = s_head; }
uint32_t a7683e_uart_dropped(void)   { return s_dropped; }
uint32_t a7683e_uart_rx_total(void)  { return s_rx_total; }

uint32_t a7683e_uart_kernel_hz(void)
{
  return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_UART4);
}

/* RX ISR. HAL_UART_IRQHandler is bypassed: it expects an armed Receive_IT
 * transfer. */
void UART4_IRQHandler(void)
{
  USART_TypeDef *u = UART4;
  uint32_t isr = u->ISR;

  /* Error flags are cleared before the drain: a pending overrun stalls RX. */
  if ((isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0u) {
    u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    s_dropped++;
  }

  while ((u->ISR & USART_ISR_RXNE_RXFNE) != 0u) {
    uint8_t  b    = (uint8_t)(u->RDR & 0xFFu);
    uint16_t next = (uint16_t)((s_head + 1u) & A7683E_RX_MASK);
    s_rx_total++;
    if (next != s_tail) {
      s_ring[s_head] = b;
      s_head = next;
    } else {
      s_dropped++;
    }
  }
}
