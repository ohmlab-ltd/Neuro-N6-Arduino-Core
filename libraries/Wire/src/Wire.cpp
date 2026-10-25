/**
 * @file    Wire.cpp
 * @brief   TwoWire, an I2C master over HAL_I2C_Master_Transmit and Receive.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include "Wire.h"
#include "stm32n6xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* The core owns I2C1 and brings it up before setup(). */
extern "C" I2C_HandleTypeDef hi2c1;
/* Core I2C mutex: recursive, no-op before the scheduler starts or in an ISR. */
extern "C" void n6_i2c_lock(void);
extern "C" void n6_i2c_unlock(void);
/* Core: frees a bus a device holds after a reset mid-transfer. */
extern "C" uint32_t n6_i2c_bus_recover(GPIO_TypeDef *sda_port, uint16_t sda_pin,
                                       GPIO_TypeDef *scl_port, uint16_t scl_pin);

#define WIRE_TIMEOUT_MS  100u

/* Wire1: I2C2. HAL_I2C_MspInit in the core routes PD15 and PD14 to it. */
static I2C_HandleTypeDef s_hi2c2;

/* Same rules as n6_i2c_lock. Created on first use. */
static SemaphoreHandle_t s_i2c2_mutex = NULL;

static bool i2c2_lock_usable(void)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return false;
  if (xPortIsInsideInterrupt() != pdFALSE) return false;
  return true;
}

static void i2c2_lock(void)
{
  if (!i2c2_lock_usable()) return;
  if (s_i2c2_mutex == NULL) {
    vTaskSuspendAll();
    if (s_i2c2_mutex == NULL) {
      s_i2c2_mutex = xSemaphoreCreateRecursiveMutex();
    }
    (void)xTaskResumeAll();
  }
  if (s_i2c2_mutex != NULL) {
    (void)xSemaphoreTakeRecursive(s_i2c2_mutex, portMAX_DELAY);
  }
}

static void i2c2_unlock(void)
{
  if (!i2c2_lock_usable()) return;
  if (s_i2c2_mutex != NULL) {
    (void)xSemaphoreGiveRecursive(s_i2c2_mutex);
  }
}

/* The caller holds the Wire1 lock. */
static bool i2c2_start(void)
{
  if (s_hi2c2.Instance == I2C2 && s_hi2c2.State != HAL_I2C_STATE_RESET) {
    return true;
  }
  /* PD15 is SDA and PD14 SCL. */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  (void)n6_i2c_bus_recover(GPIOD, GPIO_PIN_15, GPIOD, GPIO_PIN_14);
  s_hi2c2.Instance                  = I2C2;
  s_hi2c2.Init.Timing               = 0x10707CBB;
  s_hi2c2.Init.OwnAddress1          = 0;
  s_hi2c2.Init.AddressingMode       = I2C_ADDRESSINGMODE_7BIT;
  s_hi2c2.Init.DualAddressMode      = I2C_DUALADDRESS_DISABLE;
  s_hi2c2.Init.OwnAddress2          = 0;
  s_hi2c2.Init.OwnAddress2Masks     = I2C_OA2_NOMASK;
  s_hi2c2.Init.GeneralCallMode      = I2C_GENERALCALL_DISABLE;
  s_hi2c2.Init.NoStretchMode        = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&s_hi2c2) != HAL_OK) {
    return false;
  }
  (void)HAL_I2CEx_ConfigAnalogFilter(&s_hi2c2, I2C_ANALOGFILTER_ENABLE);
  (void)HAL_I2CEx_ConfigDigitalFilter(&s_hi2c2, 0);
  return true;
}

TwoWire::TwoWire()
  : _bus(0), _txAddress(0), _txLength(0), _transmitting(0), _heldAddress(0), _heldLength(0),
    _rxLength(0), _rxIndex(0)
{
}

TwoWire::TwoWire(uint8_t bus)
  : _bus(bus ? 1 : 0), _txAddress(0), _txLength(0), _transmitting(0), _heldAddress(0),
    _heldLength(0), _rxLength(0), _rxIndex(0)
{
}

void TwoWire::lock()
{
  if (_bus == 0) n6_i2c_lock();
  else           i2c2_lock();
}

void TwoWire::unlock()
{
  if (_bus == 0) n6_i2c_unlock();
  else           i2c2_unlock();
}

/* Wire1 starts I2C2 on first use. The caller holds the bus lock. */
bool TwoWire::start()
{
  return (_bus == 0) ? true : i2c2_start();
}

void TwoWire::begin()
{
  if (_bus == 0) return;
  lock();
  (void)start();
  unlock();
}

void TwoWire::begin(uint8_t)     { /* slave mode unsupported */ }

void TwoWire::end()
{
  flushHeld();
  if (_bus == 0) return;
  lock();
  if (s_hi2c2.Instance == I2C2 && s_hi2c2.State != HAL_I2C_STATE_RESET) {
    (void)HAL_I2C_DeInit(&s_hi2c2);   /* MspDeInit returns PD15/PD14 to their reset state */
  }
  unlock();
}

void TwoWire::setClock(uint32_t) { /* both buses use timing 0x10707CBB, ~295 kHz */ }

void TwoWire::beginTransmission(uint8_t address)
{
  flushHeld();
  _txAddress    = address;
  _txLength     = 0;
  _transmitting = 1;
}

size_t TwoWire::write(uint8_t data)
{
  if (!_transmitting || _txLength >= WIRE_BUFFER_LENGTH) return 0;
  _txBuffer[_txLength++] = data;
  return 1;
}

size_t TwoWire::write(const uint8_t *data, size_t len)
{
  size_t n = 0;
  for (size_t i = 0; i < len; i++) n += write(data[i]);
  return n;
}

uint8_t TwoWire::transmit(uint8_t address, const uint8_t *data, uint8_t len)
{
  I2C_HandleTypeDef *h = (_bus == 0) ? &hi2c1 : &s_hi2c2;

  lock();
  if (!start()) {
    unlock();
    return 4;
  }
  HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(
      h, (uint16_t)(address << 1), (uint8_t *)data, len, WIRE_TIMEOUT_MS);
  uint32_t err = h->ErrorCode;
  unlock();

  /* Arduino codes: 0 ok, 2 NACK, 4 other, 5 timeout. The HAL reports a
   * timeout as HAL_ERROR with HAL_I2C_ERROR_TIMEOUT, and an address and a data
   * NACK alike as HAL_I2C_ERROR_AF. */
  if (st == HAL_OK)                                        return 0;
  if ((st == HAL_TIMEOUT) || (err & HAL_I2C_ERROR_TIMEOUT)) return 5;
  if (err & HAL_I2C_ERROR_AF)                               return 2;
  return 4;
}

/* Sends held register-address bytes as a plain write, with a STOP. */
void TwoWire::flushHeld()
{
  if (_heldLength == 0u) return;
  const uint8_t n = _heldLength;
  _heldLength = 0u;
  (void)transmit(_heldAddress, _heldBuffer, n);
}

uint8_t TwoWire::endTransmission(uint8_t sendStop)
{
  if (!_transmitting) return 4;
  _transmitting = 0;
  flushHeld();

  if (!sendStop && _txLength >= 1u && _txLength <= 2u) {
    _heldAddress = _txAddress;
    _heldBuffer[0] = _txBuffer[0];
    _heldBuffer[1] = _txBuffer[1];
    _heldLength = _txLength;
    return 0;
  }
  return transmit(_txAddress, _txBuffer, _txLength);
}

uint8_t TwoWire::endTransmission(void) { return endTransmission(1); }

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
  (void)sendStop;
  if (quantity > WIRE_BUFFER_LENGTH) quantity = WIRE_BUFFER_LENGTH;
  _rxLength = 0;
  _rxIndex  = 0;

  /* Register-address bytes held for another device go out on their own. */
  if (_heldLength != 0u && _heldAddress != address) flushHeld();
  const uint8_t held = _heldLength;
  _heldLength = 0u;

  I2C_HandleTypeDef *h = (_bus == 0) ? &hi2c1 : &s_hi2c2;

  lock();
  if (!start()) {
    unlock();
    return 0;
  }
  HAL_StatusTypeDef st;
  if (held != 0u) {
    /* Write the register address, then a repeated start and the read. */
    const uint16_t reg = (held == 2u) ? (uint16_t)((_heldBuffer[0] << 8) | _heldBuffer[1])
                                      : (uint16_t)_heldBuffer[0];
    st = HAL_I2C_Mem_Read(h, (uint16_t)(address << 1), reg,
                          (held == 2u) ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT,
                          _rxBuffer, quantity, WIRE_TIMEOUT_MS);
  } else {
    st = HAL_I2C_Master_Receive(h, (uint16_t)(address << 1), _rxBuffer, quantity,
                                WIRE_TIMEOUT_MS);
  }
  unlock();

  if (st == HAL_OK) { _rxLength = quantity; return quantity; }
  return 0;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity)
{
  return requestFrom(address, quantity, 1);
}

int TwoWire::available(void) { return (int)_rxLength - (int)_rxIndex; }

int TwoWire::read(void)
{
  if (_rxIndex >= _rxLength) return -1;
  return _rxBuffer[_rxIndex++];
}

int TwoWire::peek(void)
{
  if (_rxIndex >= _rxLength) return -1;
  return _rxBuffer[_rxIndex];
}

void TwoWire::flush(void) { flushHeld(); }

TwoWire Wire(0);
TwoWire Wire1(1);
