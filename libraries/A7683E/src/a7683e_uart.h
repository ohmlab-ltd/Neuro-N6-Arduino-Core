/**
 * @file    a7683e_uart.h
 * @brief   Interrupt-driven UART4 for the A7683E link: PD0 = RX, PD1 = TX.
 * @copyright Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef A7683E_UART_H
#define A7683E_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Starts UART4 at baud, 8N1, no flow control. A repeat call changes the rate. */
bool     a7683e_uart_begin(uint32_t baud);
void     a7683e_uart_end(void);

/* Blocking. Returns the bytes accepted, 0 on timeout or when not open. */
size_t   a7683e_uart_write(const uint8_t *data, size_t len);

int      a7683e_uart_available(void);   /* bytes in the RX ring */
int      a7683e_uart_read(void);        /* -1 when empty */
int      a7683e_uart_peek(void);        /* -1 when empty */
void     a7683e_uart_discard(void);     /* empties the RX ring */

/* Bytes lost to ring overflow, plus receive error events, since begin(). */
uint32_t a7683e_uart_dropped(void);

/* Bytes the ISR has received since begin(). */
uint32_t a7683e_uart_rx_total(void);

/* UART4 kernel clock in Hz. */
uint32_t a7683e_uart_kernel_hz(void);

#ifdef __cplusplus
}
#endif

#endif /* A7683E_UART_H */
