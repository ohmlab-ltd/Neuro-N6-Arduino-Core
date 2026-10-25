/*
  HCIMayaW476Transport.cpp: ArduinoBLE HCI transport for the Neuro N6. HCI H4
  over USART6 to the NXP IW610 in the u-blox MAYA-W476 module.

  Copyright (c) 2026 Ohm Lab Ltd.
  SPDX-License-Identifier: MIT
*/
#if defined(ARDUINO_NEURON6)

#include "HCITransport.h"
#include "HCI.h"
#include "stm32n6xx_hal.h"
#include <stdio.h>
#include <string.h>

#include <MayaW476.h>

extern "C" void NeuroN6_DbgPrint(const char *msg);
extern "C" void NeuroN6_Notice(const char *msg);   /* kept and replayed to each host */

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
}

/* One lock for the whole BLE stack, and the task that polls it. */
static SemaphoreHandle_t s_ble_lock = NULL;
static TaskHandle_t      s_ble_task = NULL;

extern "C" void n6_ble_lock(void)
{
  if (s_ble_lock != NULL) (void)xSemaphoreTakeRecursive(s_ble_lock, portMAX_DELAY);
}

extern "C" void n6_ble_unlock(void)
{
  if (s_ble_lock != NULL) (void)xSemaphoreGiveRecursive(s_ble_lock);
}

static void ble_task(void *arg)
{
  (void)arg;
  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
    n6_ble_lock();
    HCI.poll();
    n6_ble_unlock();
  }
}

/* USART6 */
static UART_HandleTypeDef huart6;

/* Transmit power cap in dBm, on the controller's own scale. Commands from a
 * sketch cannot set more. */
#define BLE_TX_POWER_CAP_DBM    8

#define HCI_OP_NXP_LE_TX_POWER  0xFC87u   /* level, front-end loss flag, limit */
#define HCI_OP_LE_TX_TEST_V4    0x207Bu   /* power level is the last parameter */

/* Single-producer, single-consumer RX ring. */
#define BLE_RB_SIZE 1024u
static volatile uint8_t  s_rb[BLE_RB_SIZE];
static volatile uint16_t s_rb_head = 0u;   /* advanced by the ISR */
static volatile uint16_t s_rb_tail = 0u;   /* advanced by the consumer */
static uint8_t           s_rx_byte;
static bool              s_uart_ready = false;

/* Lost-byte counters and the last HAL error, written by the interrupt. */
static volatile uint32_t s_rb_dropped   = 0u;
static volatile uint32_t s_uart_errors  = 0u;
static volatile uint32_t s_uart_err_code = 0u;

/* RTS is a GPIO, PD4: the USART's own RTS stays asserted while a received byte
 * waits in the data register. PD4 goes high once fewer than BLE_RTS_STOP_FREE
 * bytes of the ring are free and low again at BLE_RTS_GO_FREE. */
#define BLE_RTS_STOP_FREE  32u
#define BLE_RTS_GO_FREE    128u
static volatile bool s_rts_stopped = false;

static inline uint16_t rb_free(void)
{
  const uint16_t used = (uint16_t)((uint16_t)(s_rb_head + BLE_RB_SIZE - s_rb_tail) % BLE_RB_SIZE);
  return (uint16_t)((BLE_RB_SIZE - 1u) - used);
}

/* Called from the receive interrupt. */
static inline void rts_check(void)
{
  if (!s_rts_stopped && (rb_free() < BLE_RTS_STOP_FREE)) {
    s_rts_stopped = true;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
  }
}

/* Called by the reader. Interrupts are masked so the receive interrupt cannot
 * decide a stop between the check and the write. */
static inline void rts_release(void)
{
  if (!s_rts_stopped) return;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (s_rts_stopped && (rb_free() >= BLE_RTS_GO_FREE)) {
    s_rts_stopped = false;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
  }
  __set_PRIMASK(primask);
}

static void rb_reset(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_rb_head = s_rb_tail = 0u;
  s_rts_stopped = false;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
  __set_PRIMASK(primask);
}

static bool usart6_init(void)
{
  huart6.Instance                    = USART6;
  huart6.Init.BaudRate               = 115200;
  huart6.Init.WordLength             = UART_WORDLENGTH_8B;
  huart6.Init.StopBits               = UART_STOPBITS_1;
  huart6.Init.Parity                 = UART_PARITY_NONE;
  huart6.Init.Mode                   = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl              = UART_HWCONTROL_CTS;       /* RTS is PD4, driven by rts_check() */
  huart6.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK) return false;
  (void)HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8);
  (void)HAL_UARTEx_DisableFifoMode(&huart6);
  return true;
}

extern "C" void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART6) return;

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  GPIO_InitTypeDef         GPIO_InitStruct     = {0};

  PeriphClkInitStruct.PeriphClockSelection                = RCC_PERIPHCLK_USART6;
  PeriphClkInitStruct.Usart6ClockSelection                = RCC_USART6CLKSOURCE_IC9;
  PeriphClkInitStruct.ICSelection[RCC_IC9].ClockSelection = RCC_ICCLKSOURCE_PLL1;
  PeriphClkInitStruct.ICSelection[RCC_IC9].ClockDivider   = 16;
  (void)HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  __HAL_RCC_USART6_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;   /* PC6 TX, PC7 RX */
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = GPIO_PIN_3;                /* PD3 CTS */
  GPIO_InitStruct.Alternate = GPIO_AF7_USART6;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* PD4 RTS, a GPIO output. Low lets the controller send. */
  s_rts_stopped = false;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin       = GPIO_PIN_4;
  GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Alternate = 0u;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(USART6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
}

extern "C" void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart6);
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART6) return;
  uint16_t next = (uint16_t)((s_rb_head + 1u) % BLE_RB_SIZE);
  if (next != s_rb_tail) {
    s_rb[s_rb_head] = s_rx_byte;
    s_rb_head = next;
  } else {
    s_rb_dropped++;
  }
  rts_check();
  (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
  if (s_ble_task != NULL) {          /* IRQ priority 6: FreeRTOS calls allowed */
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_ble_task, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART6) return;
  s_uart_errors++;
  s_uart_err_code = (uint32_t)huart->ErrorCode;
  (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
}

/* ArduinoBLE transport */
class HCIMayaW476TransportClass : public HCITransportInterface {
public:
  int    begin() override;
  void   end() override;
  void   wait(unsigned long timeout) override;
  int    available() override;
  int    peek() override;
  int    read() override;
  size_t write(const uint8_t* data, size_t length) override;
};

int HCIMayaW476TransportClass::begin()
{
  /* The controller exists only once the module runs its firmware. */
  if (MayaW476_RadioStart() != 0) {
    NeuroN6_Notice("[BLE] the wireless module did not start - no BLE.\r\n");
    return 0;
  }

  if (s_ble_lock == NULL) {
    s_ble_lock = xSemaphoreCreateRecursiveMutex();
    if (s_ble_lock == NULL) return 0;
  }
  if (!s_uart_ready) {
    if (!usart6_init()) return 0;
    s_uart_ready = true;
  }

  /* What the controller queued while held off can start part-way through a
   * packet, so it is dropped once the line is quiet for 10 ms, 200 ms at most. */
  n6_ble_lock();
  __HAL_UART_CLEAR_FLAG(&huart6, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
  __HAL_UART_SEND_REQ(&huart6, UART_RXDATA_FLUSH_REQUEST);
  rb_reset();
  if (HAL_UART_Receive_IT(&huart6, &s_rx_byte, 1) != HAL_OK) {
    n6_ble_unlock();
    return 0;
  }
  const uint32_t t0 = HAL_GetTick();
  uint32_t quiet_from = t0;
  uint16_t seen = s_rb_head;
  while (((HAL_GetTick() - quiet_from) < 10u) && ((HAL_GetTick() - t0) < 200u)) {
    vTaskDelay(1);
    if (s_rb_head != seen) {
      seen = s_rb_head;
      quiet_from = HAL_GetTick();
    }
  }
  rb_reset();
  n6_ble_unlock();

  /* One priority above the sketch's task, ArduinoLoop, which runs at +2. */
  if (s_ble_task == NULL &&
      xTaskCreate(ble_task, "ble", 1024, NULL, tskIDLE_PRIORITY + 3, &s_ble_task) != pdPASS) {
    s_ble_task = NULL;
    return 0;
  }
  return 1;
}

void HCIMayaW476TransportClass::end()
{
  /* Holds the controller off until the next begin(). */
  (void)HAL_UART_AbortReceive_IT(&huart6);
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_rb_head = s_rb_tail = 0u;
  s_rts_stopped = true;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
  __set_PRIMASK(primask);
}

void HCIMayaW476TransportClass::wait(unsigned long timeout)
{
  for (unsigned long start = millis(); (millis() - start) < timeout;) {
    if (available()) break;
    /* A tick to lower-priority tasks. The receive interrupt fills the ring
     * meanwhile. */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) vTaskDelay(1);
  }
}

/* At most one line a second. Called under the stack lock, not from the ISR. */
static void report_losses(void)
{
  static uint32_t last_drop = 0u, last_err = 0u, last_tick = 0u;
  const uint32_t drop = s_rb_dropped, err = s_uart_errors;
  if ((drop == last_drop) && (err == last_err)) return;
  const uint32_t now = HAL_GetTick();
  if ((last_tick != 0u) && ((now - last_tick) < 1000u)) return;
  char line[120];
  snprintf(line, sizeof(line),
           "[BLE] HCI bytes lost: %lu ring-full, %lu uart errors (last code 0x%lX)\r\n",
           (unsigned long)drop, (unsigned long)err, (unsigned long)s_uart_err_code);
  NeuroN6_DbgPrint(line);
  last_drop = drop; last_err = err; last_tick = now;
}

int HCIMayaW476TransportClass::available()
{
  report_losses();
  return (int)((uint16_t)(s_rb_head + BLE_RB_SIZE - s_rb_tail) % BLE_RB_SIZE);
}

int HCIMayaW476TransportClass::peek()
{
  if (s_rb_head == s_rb_tail) return -1;
  return (int)s_rb[s_rb_tail];
}

int HCIMayaW476TransportClass::read()
{
  if (s_rb_head == s_rb_tail) return -1;
  uint8_t b = s_rb[s_rb_tail];
  s_rb_tail = (uint16_t)((s_rb_tail + 1u) % BLE_RB_SIZE);
  rts_release();
  return (int)b;
}

/* Clamps the power fields of a command copied into cmd. */
static void tx_power_clamp(uint8_t *cmd, size_t len, uint16_t op)
{
  const int8_t cap = BLE_TX_POWER_CAP_DBM;
  uint8_t *p = &cmd[4];
  const size_t n = (cmd[3] < (len - 4u)) ? cmd[3] : (len - 4u);
  if (op == HCI_OP_NXP_LE_TX_POWER) {
    if ((n >= 1u) && ((int8_t)p[0] > cap)) p[0] = (uint8_t)cap;
    if (n >= 2u) p[1] = 0u;              /* the flag adds the front-end loss */
    if ((n >= 3u) && ((int8_t)p[2] > cap)) p[2] = (uint8_t)cap;
  } else if (n >= 8u) {
    const int8_t lvl = (int8_t)p[n - 1u];
    if ((lvl > cap) && (lvl != 0x7E)) p[n - 1u] = (uint8_t)cap;   /* 0x7E: minimum */
  }
}

size_t HCIMayaW476TransportClass::write(const uint8_t* data, size_t length)
{
  uint8_t cmd[96];
  if ((length >= 4u) && (data[0] == 0x01u)) {
    const uint16_t op = (uint16_t)(data[1] | (data[2] << 8));
    /* Other vendor commands can rewrite the module's calibration. */
    if (((op >> 10) == 0x3Fu) && (op != HCI_OP_NXP_LE_TX_POWER)) {
      NeuroN6_DbgPrint("[BLE] vendor HCI command refused.\r\n");
      return 0;
    }
    if ((op == HCI_OP_NXP_LE_TX_POWER) || (op == HCI_OP_LE_TX_TEST_V4)) {
      if (length > sizeof(cmd)) return 0;
      memcpy(cmd, data, length);
      tx_power_clamp(cmd, length, op);
      data = cmd;
    }
  }
  const HAL_StatusTypeDef st = HAL_UART_Transmit(&huart6, (uint8_t*)data, (uint16_t)length, 1000u);
  if (st != HAL_OK) {
    /* With CTS held off, a byte left in the transmit register would go out
     * ahead of the next packet, and H4 cannot resynchronise after it. */
    if (st == HAL_TIMEOUT) __HAL_UART_SEND_REQ(&huart6, UART_TXDATA_FLUSH_REQUEST);
    return 0;
  }
  return length;
}

HCIMayaW476TransportClass HCIMayaW476Transport;
HCITransportInterface& HCITransport = HCIMayaW476Transport;

#endif /* ARDUINO_NEURON6 */
