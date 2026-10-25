/**
 * @file    SPI.cpp
 * @brief   Arduino SPI master on SPI2, the header pins, polling mode.
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "SPI.h"

SPIClass SPI;

static SPI_HandleTypeDef s_hspi2 = { .Instance = SPI2 };

/* The HAL takes a 16-bit count, so longer transfers go in chunks. Each gets a
 * timeout for its length at the SCK reconfigure() chose, plus 100 ms. */
#define SPI_CHUNK_MAX  0xFFFFu
static uint32_t s_sck_hz = 250000u;
static uint32_t spi_timeout_ms(uint32_t n)
{
  return (uint32_t)(((uint64_t)n * 8000u) / s_sck_hz) + 100u;
}

static const uint32_t s_presc_val[8] = {
  SPI_BAUDRATEPRESCALER_2,   SPI_BAUDRATEPRESCALER_4,
  SPI_BAUDRATEPRESCALER_8,   SPI_BAUDRATEPRESCALER_16,
  SPI_BAUDRATEPRESCALER_32,  SPI_BAUDRATEPRESCALER_64,
  SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256,
};

void SPIClass::setKernelSource(uint32_t rccSpi2Source)
{
  RCC_PeriphCLKInitTypeDef pclk = {0};
  pclk.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
  pclk.Spi2ClockSelection   = rccSpi2Source;
  (void)HAL_RCCEx_PeriphCLKConfig(&pclk);
  _dirty = true;
  reconfigure();
}

void SPIClass::begin()
{
  if (_began) { return; }

  RCC_PeriphCLKInitTypeDef pclk = {0};
  pclk.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
  pclk.Spi2ClockSelection   = RCC_SPI2CLKSOURCE_PCLK1;
  (void)HAL_RCCEx_PeriphCLKConfig(&pclk);

  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* MOSI=PD7, MISO=PD11, SCK=PF2, all AF5. */
  GPIO_InitTypeDef io = {0};
  io.Mode      = GPIO_MODE_AF_PP;
  io.Pull      = GPIO_NOPULL;
  io.Speed     = GPIO_SPEED_FREQ_HIGH;
  io.Alternate = GPIO_AF5_SPI2;
  io.Pin       = GPIO_PIN_7 | GPIO_PIN_11;
  HAL_GPIO_Init(GPIOD, &io);
  io.Pin       = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOF, &io);

  _began = true;
  _dirty = true;
  reconfigure();
}

void SPIClass::end()
{
  if (!_began) { return; }
  HAL_SPI_DeInit(&s_hspi2);
  _began = false;
}

void SPIClass::reconfigure()
{
  if (!_began || !_dirty) { return; }
  /* HAL_SPI_DeInit/Init below must not run on a live transfer. */
  if (_txnOpen) { txnClose(); }

  /* Largest SCK <= the requested clock. */
  uint32_t kernel_hz = (uint32_t)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2);
  if (kernel_hz == 0u) { kernel_hz = HAL_RCC_GetPCLK1Freq(); }
  if (kernel_hz == 0u) { kernel_hz = 100000000u; }
  unsigned pi = 0;
  while (pi < 7u && (kernel_hz >> (pi + 1u)) > _settings.clockFreq) { pi++; }
  s_sck_hz = kernel_hz >> (pi + 1u);
  if (s_sck_hz == 0u) { s_sck_hz = 1u; }

  HAL_SPI_DeInit(&s_hspi2);

  s_hspi2.Init.Mode              = SPI_MODE_MASTER;
  s_hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
  s_hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
  s_hspi2.Init.CLKPolarity       = (_settings.dataMode & 0x2) ? SPI_POLARITY_HIGH
                                                              : SPI_POLARITY_LOW;
  s_hspi2.Init.CLKPhase          = (_settings.dataMode & 0x1) ? SPI_PHASE_2EDGE
                                                              : SPI_PHASE_1EDGE;
  s_hspi2.Init.NSS               = SPI_NSS_SOFT;
  s_hspi2.Init.BaudRatePrescaler = s_presc_val[pi];
  s_hspi2.Init.FirstBit          = (_settings.bitOrder == LSBFIRST) ? SPI_FIRSTBIT_LSB
                                                                    : SPI_FIRSTBIT_MSB;
  s_hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
  s_hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  s_hspi2.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

  (void)HAL_SPI_Init(&s_hspi2);
  _dirty = false;
}

void SPIClass::forceDataMode(int mode)
{
  _forcedMode = mode;
  if (mode >= 0 && _settings.dataMode != (uint8_t)mode) {
    _settings.dataMode = (uint8_t)mode;
    _dirty = true;
    reconfigure();
  }
}

/* SPE and CSTART are held for the whole transaction. Each SPE enable drives
 * SCK to the CPOL idle level, which at CPOL = 0 reads as a clock edge. */
static inline uint8_t spi2_raw_xfer(uint8_t v)
{
  SPI_TypeDef *S = SPI2;
  while ((S->SR & SPI_SR_TXP) == 0u) { }
  *(volatile uint8_t *)&S->TXDR = v;
  while ((S->SR & SPI_SR_RXP) == 0u) { }
  return *(volatile uint8_t *)&S->RXDR;
}

void SPIClass::txnOpen()
{
  SPI_TypeDef *S = SPI2;
  MODIFY_REG(S->CR2, SPI_CR2_TSIZE, 0u);   /* endless: clock follows data */
  SET_BIT(S->CR1, SPI_CR1_SPE);
  SET_BIT(S->CR1, SPI_CR1_CSTART);
  _txnOpen = true;
}

void SPIClass::txnClose()
{
  SPI_TypeDef *S = SPI2;
  uint32_t guard = 1000000u;
  while (((S->SR & SPI_SR_TXC) == 0u) && --guard) { }
  /* An endless transfer is stopped by suspending it, not by clearing CSTART. */
  SET_BIT(S->CR1, SPI_CR1_CSUSP);
  guard = 1000000u;
  while (((S->SR & SPI_SR_SUSP) == 0u) && --guard) { }
  CLEAR_BIT(S->CR1, SPI_CR1_SPE);
  S->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_SUSPC;
  _txnOpen = false;
}

void SPIClass::beginTransaction(SPISettings settings)
{
  if (_txnOpen) { txnClose(); }
  if (_forcedMode >= 0) { settings.dataMode = (uint8_t)_forcedMode; }
  if (settings.clockFreq != _settings.clockFreq ||
      settings.bitOrder  != _settings.bitOrder  ||
      settings.dataMode  != _settings.dataMode) {
    _settings = settings;
    _dirty = true;
  }
  if (!_began) { begin(); }
  reconfigure();
  txnOpen();
}

void SPIClass::endTransaction()
{
  if (_txnOpen) { txnClose(); }
}

uint8_t SPIClass::transfer(uint8_t data)
{
  if (_txnOpen) { return spi2_raw_xfer(data); }
  uint8_t rx = 0xFF;
  (void)HAL_SPI_TransmitReceive(&s_hspi2, &data, &rx, 1, 100);
  return rx;
}

uint16_t SPIClass::transfer16(uint16_t data)
{
  uint8_t hi, lo;
  if (_settings.bitOrder == LSBFIRST) {
    lo = transfer((uint8_t)(data & 0xFF));
    hi = transfer((uint8_t)(data >> 8));
  } else {
    hi = transfer((uint8_t)(data >> 8));
    lo = transfer((uint8_t)(data & 0xFF));
  }
  return (uint16_t)((hi << 8) | lo);
}

void SPIClass::transfer(void *buf, size_t count)
{
  if (buf == NULL || count == 0) { return; }
  uint8_t *p = (uint8_t *)buf;
  if (_txnOpen) {
    for (size_t i = 0; i < count; i++) { p[i] = spi2_raw_xfer(p[i]); }
    return;
  }
  while (count > 0u) {
    const uint16_t n = (count > SPI_CHUNK_MAX) ? (uint16_t)SPI_CHUNK_MAX : (uint16_t)count;
    if (HAL_SPI_TransmitReceive(&s_hspi2, p, p, n, spi_timeout_ms(n)) != HAL_OK) { return; }
    p += n;
    count -= n;
  }
}

void SPIClass::transfer(const void *tx, void *rx, size_t count)
{
  if (count == 0) { return; }
  if (_txnOpen) {
    const uint8_t *t = (const uint8_t *)tx;
    uint8_t       *r = (uint8_t *)rx;
    for (size_t i = 0; i < count; i++) {
      const uint8_t got = spi2_raw_xfer(t ? t[i] : 0xFFu);
      if (r) { r[i] = got; }
    }
    return;
  }
  uint8_t *t = (uint8_t *)tx;
  uint8_t *r = (uint8_t *)rx;
  while (count > 0u) {
    const uint16_t n = (count > SPI_CHUNK_MAX) ? (uint16_t)SPI_CHUNK_MAX : (uint16_t)count;
    HAL_StatusTypeDef st = HAL_OK;
    if (t != NULL && r != NULL) {
      st = HAL_SPI_TransmitReceive(&s_hspi2, t, r, n, spi_timeout_ms(n));
    } else if (t != NULL) {
      st = HAL_SPI_Transmit(&s_hspi2, t, n, spi_timeout_ms(n));
    } else if (r != NULL) {
      st = HAL_SPI_Receive(&s_hspi2, r, n, spi_timeout_ms(n));
    }
    if (st != HAL_OK) { return; }
    if (t != NULL) { t += n; }
    if (r != NULL) { r += n; }
    count -= n;
  }
}

void SPIClass::setBitOrder(uint8_t order)
{
  _settings.bitOrder = order;
  _dirty = true;
  reconfigure();
}

void SPIClass::setDataMode(uint8_t mode)
{
  _settings.dataMode = mode;
  _dirty = true;
  reconfigure();
}

void SPIClass::setClockDivider(uint8_t div)
{
  /* The divider applies to the kernel clock setKernelSource() selected. */
  uint32_t kernel_hz = (uint32_t)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2);
  if (kernel_hz == 0u) { kernel_hz = HAL_RCC_GetPCLK1Freq(); }
  if (div == 0) { div = 4; }
  _settings.clockFreq = kernel_hz / div;
  _dirty = true;
  reconfigure();
}
