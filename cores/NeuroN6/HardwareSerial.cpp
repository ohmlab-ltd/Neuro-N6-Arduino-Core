/**
 ******************************************************************************
 * @file    HardwareSerial.cpp
 * @brief   Arduino `Serial` over USB-CDC.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#include "HardwareSerial.h"
#include "tusb.h"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
/* Nonzero once a task pumps TinyUSB: the boot pump or n6_usb. */
int NeuroN6_UsbPumpTaskStarted(void);
}

HardwareSerial Serial;

/* RX ring. The size is a power of two. */
#define RX_RING_SIZE 512u
static uint8_t           s_rx_buf[RX_RING_SIZE];
static volatile uint32_t s_rx_head;   /* written only by the core USB task */
static volatile uint32_t s_rx_tail;   /* written only by the sketch task */

extern "C" void NeuroN6_SerialRxSink(uint8_t ch)
{
  uint32_t head = s_rx_head;
  uint32_t next = (head + 1u) & (RX_RING_SIZE - 1u);
  if (next == s_rx_tail) {
    return;                    /* full: drop newest */
  }
  s_rx_buf[head] = ch;
  s_rx_head = next;
}

/* Free bytes in the RX ring. */
extern "C" uint32_t NeuroN6_SerialRxSpace(void)
{
  return (RX_RING_SIZE - 1u) - ((s_rx_head - s_rx_tail) & (RX_RING_SIZE - 1u));
}

/* newlib stdin, called by _read() in syscalls.c for getchar() and scanf():
 * the next byte Serial would read, waiting for one. */
extern "C" int __io_getchar(void)
{
  int c;
  while ((c = Serial.read()) < 0) {
    vTaskDelay(1);
  }
  return c;
}

/* Stream: RX. */

int HardwareSerial::available()
{
  return (int)((s_rx_head - s_rx_tail) & (RX_RING_SIZE - 1u));
}

int HardwareSerial::read()
{
  uint32_t tail = s_rx_tail;
  if (tail == s_rx_head) { return -1; }
  int c = s_rx_buf[tail];
  s_rx_tail = (tail + 1u) & (RX_RING_SIZE - 1u);
  return c;
}

int HardwareSerial::peek()
{
  uint32_t tail = s_rx_tail;
  if (tail == s_rx_head) { return -1; }
  return s_rx_buf[tail];
}

/* Print: TX. */

HardwareSerial::operator bool()
{
  /* Nothing else services TinyUSB until the pump task exists. */
  if (!NeuroN6_UsbPumpTaskStarted()) {
    tud_task_ext(0, false);   /* non-blocking */
  }
  return tud_cdc_n_connected(0);
}

size_t HardwareSerial::write(uint8_t b)
{
  return write(&b, 1);
}

size_t HardwareSerial::write(const uint8_t *buf, size_t len)
{
  if (buf == NULL || len == 0 || !tud_cdc_n_connected(0)) { return 0; }

  bool pump_inline = !NeuroN6_UsbPumpTaskStarted();

  size_t sent = 0;
  uint32_t guard_ms = 100;   /* bounds the wait when the host stops reading */
  while (sent < len) {
    uint32_t n = tud_cdc_n_write(0, buf + sent, (uint32_t)(len - sent));
    sent += n;
    if (sent < len) {
      tud_cdc_n_write_flush(0);
      if (!tud_cdc_n_connected(0) || guard_ms == 0u) { break; }
      guard_ms--;
      if (pump_inline) {
        tud_task_ext(0, false);
      } else if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(1);
      } else {
        tud_task_ext(0, false);
      }
    }
  }
  tud_cdc_n_write_flush(0);
  if (pump_inline) { tud_task_ext(0, false); }
  return sent;
}

void HardwareSerial::flush()
{
  tud_cdc_n_write_flush(0);
  if (!NeuroN6_UsbPumpTaskStarted()) { tud_task_ext(0, false); }
}

int HardwareSerial::availableForWrite()
{
  return (int)tud_cdc_n_write_available(0);
}
