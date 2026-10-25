/**
  ******************************************************************************
  * @file    NeuroVisionThermal.cpp
  * @brief   FLIR Lepton 3.1R thermal camera driver.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#include "NeuroVisionThermal.h"
#include "stm32n6xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "NeuroN6_FrameQueue.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "../../PostProcess/src/vnd_meta.h"

/* I2C1, initialised by the core. */
extern I2C_HandleTypeDef hi2c1;

#define NVT_I2C_TIMEOUT_MS   100u

/* Steady I2C traffic to any device while the Lepton boots, about 1.5 s after
 * reset, can stop the boot until the next powerOn(). begin() polls only after
 * this. */
#define NVT_BOOT_QUIET_MS    2000u
static bool     _nvtResetReleased    = false;  /* powerOn() has run */
static uint32_t _nvtResetReleaseTick = 0u;     /* HAL tick at its reset release */

/* Lepton control pins, active low. */
#define NVT_PWRDWN_PORT      GPIOD            /* PWR_DWN_L = PD3 */
#define NVT_PWRDWN_PIN       GPIO_PIN_3
#define NVT_RESET_PORT       GPIOD            /* RESET_L   = PD9 */
#define NVT_RESET_PIN        GPIO_PIN_9

/* VoSPI on SPI1: SCK, MOSI and MISO on AF5, CS as a GPIO. */
#define NVT_SPI_INSTANCE     SPI1
#define NVT_SPI_AF           GPIO_AF5_SPI1
#define NVT_SPI_SCK_PORT     GPIOA            /* PA5  SPI1_SCK */
#define NVT_SPI_SCK_PIN      GPIO_PIN_5
#define NVT_SPI_MOSI_PORT    GPIOA            /* PA7  SPI1_MOSI, unused */
#define NVT_SPI_MOSI_PIN     GPIO_PIN_7
#define NVT_SPI_MISO_PORT    GPIOB            /* PB4  SPI1_MISO */
#define NVT_SPI_MISO_PIN     GPIO_PIN_4
#define NVT_SPI_CS_PORT      GPIOF            /* PF4  CS, active low */
#define NVT_SPI_CS_PIN       GPIO_PIN_4

/* The Lepton's maximum SCK. */
#define NVT_SPI_SCK_MAX_HZ   20000000u

/* CS high and SCK idle before a capture starts. FLIR's minimum is 185 ms. */
#ifndef NVT_RESYNC_IDLE_MS
#define NVT_RESYNC_IDLE_MS   300u
#endif

/* In sync, every packet is well formed. This many halves in a row with fewer
 * than half well formed means the sync is lost, and capture restarts. */
#define NVT_BAD_HALVES_MAX   4u

/* VoSPI capture. CS stays low and SCK never pauses, so the 164-byte packet
 * alignment holds once synced. */

/* The ring must fit HAL_SPI_Receive_DMA's 16-bit Size. */
#define NVT_DMA_PACKETS_PER_HALF  180
#define NVT_DMA_HALF_BYTES        (NVT_DMA_PACKETS_PER_HALF * NVT_VOSPI_PACKET_BYTES)  /* 29520 */
#define NVT_DMA_RING_BYTES        (2u * NVT_DMA_HALF_BYTES)                /* 59040 = 1845*32 */

static SPI_HandleTypeDef _nvtSpi;
static bool _nvtSpiReady = false;
static uint32_t _nvtSckHz = 0;
static uint32_t _nvtKernelHz = 0;   /* HAL_RCC_GetPCLK2Freq() reading */
static int      _nvtLastErr = 0;    /* NVT_VIDEO_ERR_* */
static bool     _nvtFlip180 = false;

/* DMA ring, written only by the GPDMA. Aligned to the 32-byte cache line and a
 * multiple of it, so a cache invalidate touches nothing outside the ring. */
__attribute__((aligned(32)))
static uint8_t _nvtRing[NVT_DMA_RING_BYTES];

static DMA_HandleTypeDef _nvtDma;
static SemaphoreHandle_t _nvtHalfSem = NULL;
static volatile uint32_t _nvtHalvesProduced = 0;
static uint32_t          _nvtHalvesConsumed = 0;

/* Software ring. The DMA interrupt copies each completed half here. */
#define NVT_SW_SLOTS 6u
__attribute__((aligned(32)))
static uint8_t _nvtSwRing[NVT_SW_SLOTS][NVT_DMA_HALF_BYTES];
static volatile bool     _nvtSpiError = false;
static volatile uint32_t _nvtSpiErrCount = 0;
static volatile uint32_t _nvtSpiErrCode = 0;    /* last hspi->ErrorCode */
static bool              _nvtStreaming = false;
/* True from videoBegin() to videoEnd(). CCI is refused while it is set. */
static volatile bool     _nvtVideoActive = false;

/* Frame assembly state, kept across readFrame() calls. */
static uint16_t _nvtSegPix[NVT_VOSPI_PACKET_PIXELS * NVT_VOSPI_PACKETS_PER_SEGMENT]; /* current segment */
static uint16_t _nvtFrameAcc[NVT_VOSPI_PIXELS];   /* in-order segments so far */
static int  _nvtExpectPkt = 0;    /* next packet number wanted, 0 while hunting packet 0 */
static int  _nvtCurSeg = -1;      /* segment id from packet 20: -1 unknown, 0 ignore */
static int  _nvtGotSegs = 0;      /* highest in-order segment placed, 0 to 4 */
static uint32_t _nvtProgressTick = 0;  /* last valid segment or well-formed half */
static uint32_t _nvtBadHalves = 0;     /* consecutive halves of mostly malformed packets */

/* Statistics, read through videoStats(). */
static volatile uint32_t _nvtSegsParsed = 0;    /* completed segments, any id */
static uint32_t          _nvtFramesParsed = 0;  /* complete frames delivered */
static uint32_t          _nvtRestarts = 0;      /* capture starts, resyncs included */
static uint32_t          _nvtLastRaw = 0;       /* first 4 bytes of the last parsed half */
static uint32_t          _nvtDiscards = 0;      /* discard packets */

static inline void _nvtCsLow(void)  { HAL_GPIO_WritePin(NVT_SPI_CS_PORT, NVT_SPI_CS_PIN, GPIO_PIN_RESET); }
static inline void _nvtCsHigh(void) { HAL_GPIO_WritePin(NVT_SPI_CS_PORT, NVT_SPI_CS_PIN, GPIO_PIN_SET); }

static void _nvtCaptureStop(void);

static void _nvtDelayMs(uint32_t ms)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    vTaskDelay(pdMS_TO_TICKS(ms));
  } else {
    HAL_Delay(ms);
  }
}

/* CCI register access */

/* The core's I2C bus mutex. Every CCI transaction holds it. */
extern "C" void n6_i2c_lock(void);
extern "C" void n6_i2c_unlock(void);
/* Exclusive claim on the same mutex, held while the Lepton boots. Other
 * traffic on the bus in that window stops the part coming up. */
extern "C" void n6_i2c_claim(void);
extern "C" void n6_i2c_release(void);
/* The core's CDC console print. */
extern "C" void NeuroN6_DbgPrint(const char *msg);

/* A CCI transaction while the Lepton streams VoSPI holds the I2C bus, so
 * every CCI entry point refuses while _nvtVideoActive is set. */
bool NeuroVisionThermal::readRegister(uint16_t reg, uint16_t &value)
{
  if (_nvtVideoActive) { _lastResult = NVT_ERR_STREAMING; return false; }
  uint8_t buf[2] = {0, 0};
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, NVT_I2C_ADDR_8BIT, reg,
                                          I2C_MEMADD_SIZE_16BIT, buf, 2,
                                          NVT_I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  if (st != HAL_OK) {
    _lastResult = NVT_ERR_I2C;
    return false;
  }
  value = (uint16_t)((uint16_t)buf[0] << 8 | buf[1]);
  return true;
}

bool NeuroVisionThermal::writeRegister(uint16_t reg, uint16_t value)
{
  if (_nvtVideoActive) { _lastResult = NVT_ERR_STREAMING; return false; }
  uint8_t buf[2];
  buf[0] = (uint8_t)(value >> 8);
  buf[1] = (uint8_t)(value & 0xFF);
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, NVT_I2C_ADDR_8BIT, reg,
                                           I2C_MEMADD_SIZE_16BIT, buf, 2,
                                           NVT_I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  if (st != HAL_OK) { _lastResult = NVT_ERR_I2C; return false; }
  return true;
}

/* Status, presence and boot */

bool NeuroVisionThermal::isPresent(void)
{
  if (_nvtVideoActive) { _lastResult = NVT_ERR_STREAMING; return false; }
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1, NVT_I2C_ADDR_8BIT, 2,
                                               NVT_I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  if (st != HAL_OK) { _lastResult = NVT_ERR_I2C; return false; }
  return true;
}

uint8_t NeuroVisionThermal::scanBus(uint8_t *foundAddrs7, uint8_t maxFound)
{
  uint8_t count = 0;
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    /* A probe of a streaming Lepton holds the bus. */
    if (_nvtVideoActive && a == NVT_I2C_ADDR_7BIT) { continue; }
    n6_i2c_lock();
    HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1,
                                                 NVT_I2C_TIMEOUT_MS);
    n6_i2c_unlock();
    if (st == HAL_OK) {
      if (foundAddrs7 && count < maxFound) {
        foundAddrs7[count] = a;
      }
      count++;
    }
  }
  return count;
}

bool NeuroVisionThermal::readStatusRegister(uint16_t &status)
{
  return readRegister(NVT_REG_STATUS, status);
}

bool NeuroVisionThermal::isBooted(void)
{
  uint16_t status;
  if (!readStatusRegister(status)) {
    return false;
  }
  return (status & NVT_STATUS_BOOT_STATUS) && !(status & NVT_STATUS_BUSY);
}

bool NeuroVisionThermal::waitReady(uint32_t timeoutMs)
{
  uint32_t start = HAL_GetTick();
  uint16_t status;
  for (;;) {
    if (readStatusRegister(status) && !(status & NVT_STATUS_BUSY)) {
      return true;
    }
    if ((HAL_GetTick() - start) >= timeoutMs) {
      return false;
    }
    HAL_Delay(2);
  }
}

void NeuroVisionThermal::powerOn(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* The Lepton samples the VoSPI chip select at power-up. CS must be high
   * through boot. */
  _nvtCaptureStop();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  HAL_GPIO_WritePin(NVT_SPI_CS_PORT, NVT_SPI_CS_PIN, GPIO_PIN_SET);
  {
    GPIO_InitTypeDef cs = {0};
    cs.Mode  = GPIO_MODE_OUTPUT_PP;
    cs.Pull  = GPIO_NOPULL;
    cs.Speed = GPIO_SPEED_FREQ_HIGH;
    cs.Pin   = NVT_SPI_CS_PIN;
    HAL_GPIO_Init(NVT_SPI_CS_PORT, &cs);
  }

  HAL_GPIO_WritePin(NVT_PWRDWN_PORT, NVT_PWRDWN_PIN, GPIO_PIN_RESET);  /* power down */
  HAL_GPIO_WritePin(NVT_RESET_PORT,  NVT_RESET_PIN,  GPIO_PIN_RESET);  /* assert reset */

  GPIO_InitTypeDef g = {0};
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = NVT_PWRDWN_PIN;
  HAL_GPIO_Init(NVT_PWRDWN_PORT, &g);
  g.Pin   = NVT_RESET_PIN;
  HAL_GPIO_Init(NVT_RESET_PORT, &g);

  _nvtDelayMs(100);
  /* Power up with reset still asserted. */
  HAL_GPIO_WritePin(NVT_PWRDWN_PORT, NVT_PWRDWN_PIN, GPIO_PIN_SET);
  _nvtDelayMs(30);
  /* Release reset. The Lepton boots in about 1.5 s. */
  HAL_GPIO_WritePin(NVT_RESET_PORT, NVT_RESET_PIN, GPIO_PIN_SET);
  _nvtResetReleaseTick = HAL_GetTick();
  _nvtResetReleased = true;
}

void NeuroVisionThermal::readControlPins(bool &pwrdwnHigh, bool &resetHigh)
{
  pwrdwnHigh = (HAL_GPIO_ReadPin(NVT_PWRDWN_PORT, NVT_PWRDWN_PIN) == GPIO_PIN_SET);
  resetHigh  = (HAL_GPIO_ReadPin(NVT_RESET_PORT,  NVT_RESET_PIN)  == GPIO_PIN_SET);
}

bool NeuroVisionThermal::begin(uint32_t bootTimeoutMs)
{
  if (_nvtVideoActive) { _lastResult = NVT_ERR_STREAMING; return false; }
  uint32_t start = HAL_GetTick();
  if (_nvtResetReleased) {
    const uint32_t since = start - _nvtResetReleaseTick;
    if (since < NVT_BOOT_QUIET_MS) _nvtDelayMs(NVT_BOOT_QUIET_MS - since);
  }
  uint16_t status;
  for (;;) {
    if (readStatusRegister(status)) {
      if ((status & NVT_STATUS_BOOT_STATUS) && !(status & NVT_STATUS_BUSY)) {
        return true;
      }
    }
    if ((HAL_GetTick() - start) >= bootTimeoutMs) {
      return false;
    }
    HAL_Delay(20);
  }
}

/* DATA registers, 16 words at most. A longer transfer needs the 0xF800 block
 * area, which this driver does not use. */

bool NeuroVisionThermal::_readData(uint16_t *outWords, uint16_t lenWords)
{
  if (lenWords > 16) {
    return false;
  }
  for (uint16_t i = 0; i < lenWords; i++) {
    if (!readRegister((uint16_t)(NVT_REG_DATA_0 + i * 2), outWords[i])) {
      return false;
    }
  }
  return true;
}

bool NeuroVisionThermal::_writeData(const uint16_t *inWords, uint16_t lenWords)
{
  if (lenWords > 16) {
    return false;
  }
  for (uint16_t i = 0; i < lenWords; i++) {
    if (!writeRegister((uint16_t)(NVT_REG_DATA_0 + i * 2), inWords[i])) {
      return false;
    }
  }
  return true;
}

/* CCI commands. STATUS[15:8] holds the signed LEP result once BUSY clears. */

static int16_t nvt_status_result(uint16_t status)
{
  return (int16_t)((int8_t)((status >> 8) & 0xFF));
}

int16_t NeuroVisionThermal::getAttribute(uint16_t command, uint16_t *outWords, uint16_t lenWords)
{
  if (_nvtVideoActive) { _lastResult = NVT_ERR_STREAMING; return _lastResult; }
  if (!waitReady(NVT_I2C_TIMEOUT_MS * 5)) { _lastResult = NVT_ERR_TIMEOUT; return _lastResult; }

  if (!writeRegister(NVT_REG_DATA_LENGTH, lenWords)) { _lastResult = NVT_ERR_I2C; return _lastResult; }
  if (!writeRegister(NVT_REG_COMMAND, command))      { _lastResult = NVT_ERR_I2C; return _lastResult; }

  if (!waitReady(NVT_I2C_TIMEOUT_MS * 10)) { _lastResult = NVT_ERR_TIMEOUT; return _lastResult; }

  uint16_t status;
  if (!readStatusRegister(status)) { _lastResult = NVT_ERR_I2C; return _lastResult; }

  int16_t result = nvt_status_result(status);
  if (result == NVT_OK && outWords && lenWords) {
    if (!_readData(outWords, lenWords)) { _lastResult = NVT_ERR_I2C; return _lastResult; }
  }
  _lastResult = result;
  return result;
}

int16_t NeuroVisionThermal::setAttribute(uint16_t command, const uint16_t *inWords, uint16_t lenWords)
{
  if (_nvtVideoActive) { _lastResult = NVT_ERR_STREAMING; return _lastResult; }
  if (!waitReady(NVT_I2C_TIMEOUT_MS * 5)) { _lastResult = NVT_ERR_TIMEOUT; return _lastResult; }

  if (inWords && lenWords) {
    if (!_writeData(inWords, lenWords)) { _lastResult = NVT_ERR_I2C; return _lastResult; }
  }
  if (!writeRegister(NVT_REG_DATA_LENGTH, lenWords)) { _lastResult = NVT_ERR_I2C; return _lastResult; }
  if (!writeRegister(NVT_REG_COMMAND, command))      { _lastResult = NVT_ERR_I2C; return _lastResult; }

  if (!waitReady(NVT_I2C_TIMEOUT_MS * 10)) { _lastResult = NVT_ERR_TIMEOUT; return _lastResult; }

  uint16_t status;
  if (!readStatusRegister(status)) { _lastResult = NVT_ERR_I2C; return _lastResult; }
  _lastResult = nvt_status_result(status);
  return _lastResult;
}

int16_t NeuroVisionThermal::runCommand(uint16_t command)
{
  if (_nvtVideoActive) { _lastResult = NVT_ERR_STREAMING; return _lastResult; }
  if (!waitReady(NVT_I2C_TIMEOUT_MS * 5)) { _lastResult = NVT_ERR_TIMEOUT; return _lastResult; }

  if (!writeRegister(NVT_REG_COMMAND, command)) { _lastResult = NVT_ERR_I2C; return _lastResult; }

  if (!waitReady(NVT_I2C_TIMEOUT_MS * 10)) { _lastResult = NVT_ERR_TIMEOUT; return _lastResult; }

  uint16_t status;
  if (!readStatusRegister(status)) { _lastResult = NVT_ERR_I2C; return _lastResult; }
  _lastResult = nvt_status_result(status);
  return _lastResult;
}

/* Command wrappers */

bool NeuroVisionThermal::ping(void)
{
  return (runCommand(NVT_CMD_SYS_PING) == NVT_OK);
}

bool NeuroVisionThermal::getSerialNumber(uint16_t serialWords[4])
{
  return (getAttribute(NVT_CMD_SYS_SERIAL, serialWords, 4) == NVT_OK);
}

bool NeuroVisionThermal::getUptimeMs(uint32_t &uptimeMs)
{
  uint16_t w[2] = {0, 0};
  if (getAttribute(NVT_CMD_SYS_UPTIME, w, 2) != NVT_OK) {
    return false;
  }
  /* Low word first. */
  uptimeMs = ((uint32_t)w[1] << 16) | w[0];
  return true;
}

bool NeuroVisionThermal::getFpaTemperatureK100(uint16_t &kelvinX100)
{
  return (getAttribute(NVT_CMD_SYS_FPA_TEMP_K, &kelvinX100, 1) == NVT_OK);
}

bool NeuroVisionThermal::getAuxTemperatureK100(uint16_t &kelvinX100)
{
  return (getAttribute(NVT_CMD_SYS_AUX_TEMP_K, &kelvinX100, 1) == NVT_OK);
}

bool NeuroVisionThermal::getAgcEnabled(bool &enabled)
{
  uint16_t w[2] = {0, 0};
  if (getAttribute(NVT_CMD_AGC_ENABLE_GET, w, 2) != NVT_OK) {
    return false;
  }
  enabled = (w[0] != 0);
  return true;
}

int16_t NeuroVisionThermal::runFfc(void)
{
  return runCommand(NVT_CMD_SYS_RUN_FFC);
}

/* VoSPI video */

/* SPI1_RX DMA. GPDMA1 channel 0 belongs to the Microphone library. */
extern "C" void GPDMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&_nvtDma);
}

/* Fires for errors only: the circular receive never raises EOT. */
extern "C" void SPI1_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&_nvtSpi);
}

static void _nvtHalfReady(uint32_t hwHalf)
{
  /* Interrupt context. This half is stable while the GPDMA fills the other. */
  uint8_t *src = &_nvtRing[hwHalf * NVT_DMA_HALF_BYTES];
  SCB_InvalidateDCache_by_Addr((void *)src, (int32_t)NVT_DMA_HALF_BYTES);
  memcpy(_nvtSwRing[_nvtHalvesProduced % NVT_SW_SLOTS], src, NVT_DMA_HALF_BYTES);
  _nvtHalvesProduced++;
  BaseType_t woken = pdFALSE;
  if (_nvtHalfSem) {
    xSemaphoreGiveFromISR(_nvtHalfSem, &woken);
  }
  portYIELD_FROM_ISR(woken);
}

extern "C" void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == NVT_SPI_INSTANCE) _nvtHalfReady(0u);
}

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == NVT_SPI_INSTANCE) _nvtHalfReady(1u);
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance != NVT_SPI_INSTANCE) return;
  _nvtSpiError = true;
  _nvtSpiErrCount++;
  _nvtSpiErrCode = hspi->ErrorCode;
  BaseType_t woken = pdFALSE;
  if (_nvtHalfSem) {
    xSemaphoreGiveFromISR(_nvtHalfSem, &woken);
  }
  portYIELD_FROM_ISR(woken);
}

/* A circular list makes HAL_SPI_Receive_DMA program TSIZE = 0, an endless
 * transaction. The GPDMA fetches the node from RAM, so it lives in the
 * non-cacheable .axisram_nc section. */
__attribute__((section(".axisram_nc"), aligned(32), used))
static DMA_NodeTypeDef _nvtDmaNode;

static bool _nvtDmaInit(void)
{
  static DMA_QListTypeDef    queue;
  static bool                built = false;

  if (built) return true;
  _nvtLastErr = NVT_VIDEO_ERR_DMA_BUILD;
  /* .axisram_nc is NOLOAD, so the node is zeroed here. */
  memset(&_nvtDmaNode, 0, sizeof(_nvtDmaNode));
  memset(&queue, 0, sizeof(queue));

  __HAL_RCC_GPDMA1_CLK_ENABLE();

  _nvtDma.Instance = GPDMA1_Channel1;

  DMA_NodeConfTypeDef cfg = {0};
  cfg.NodeType                         = DMA_GPDMA_LINEAR_NODE;
  cfg.Init.Request                     = GPDMA1_REQUEST_SPI1_RX;
  cfg.Init.BlkHWRequest                = DMA_BREQ_SINGLE_BURST;
  cfg.Init.Direction                   = DMA_PERIPH_TO_MEMORY;
  cfg.Init.SrcInc                      = DMA_SINC_FIXED;
  cfg.Init.DestInc                     = DMA_DINC_INCREMENTED;
  cfg.Init.SrcDataWidth                = DMA_SRC_DATAWIDTH_BYTE;
  cfg.Init.DestDataWidth               = DMA_DEST_DATAWIDTH_BYTE;
  cfg.Init.SrcBurstLength              = 1;
  cfg.Init.DestBurstLength             = 1;
  cfg.Init.Priority                    = DMA_HIGH_PRIORITY;
  cfg.Init.TransferEventMode           = DMA_TCEM_BLOCK_TRANSFER;
  cfg.Init.TransferAllocatedPort       = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
  cfg.DataHandlingConfig.DataExchange  = DMA_EXCHANGE_NONE;
  cfg.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  cfg.TriggerConfig.TriggerPolarity    = DMA_TRIG_POLARITY_MASKED;
  cfg.SrcSecure                        = DMA_CHANNEL_SRC_SEC;
  cfg.DestSecure                       = DMA_CHANNEL_DEST_SEC;

  if (HAL_DMAEx_List_BuildNode(&cfg, &_nvtDmaNode) != HAL_OK)        return false;
  if (HAL_DMAEx_List_InsertNode_Tail(&queue, &_nvtDmaNode) != HAL_OK) return false;
  if (HAL_DMAEx_List_SetCircularMode(&queue) != HAL_OK)        return false;

  _nvtDma.InitLinkedList.Priority          = DMA_HIGH_PRIORITY;
  _nvtDma.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
  _nvtDma.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
  _nvtDma.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  _nvtDma.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;
  _nvtLastErr = NVT_VIDEO_ERR_DMA_INIT;
  if (HAL_DMAEx_List_Init(&_nvtDma) != HAL_OK)                 return false;

  __HAL_RCC_RIFSC_CLK_ENABLE();
  _nvtLastErr = NVT_VIDEO_ERR_DMA_ATTR;
  if (HAL_DMA_ConfigChannelAttributes(&_nvtDma,
      DMA_CHANNEL_SEC | DMA_CHANNEL_PRIV | DMA_CHANNEL_SRC_SEC | DMA_CHANNEL_DEST_SEC) != HAL_OK) {
    return false;
  }
  _nvtLastErr = NVT_VIDEO_ERR_DMA_LINK;
  if (HAL_DMAEx_List_LinkQ(&_nvtDma, &queue) != HAL_OK)        return false;
  _nvtLastErr = NVT_VIDEO_OK;

  __HAL_LINKDMA(&_nvtSpi, hdmarx, _nvtDma);

  HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
  HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);

  built = true;
  return true;
}

static void _nvtCaptureStop(void)
{
  if (_nvtStreaming) {
    (void)HAL_SPI_Abort(&_nvtSpi);   /* aborts the RX DMA too */
    _nvtStreaming = false;
  }
  _nvtCsHigh();
}

/* Starts capture. CS high and SCK idle for NVT_RESYNC_IDLE_MS first, so the
 * Lepton restarts at segment 1, packet 0. */
static bool _nvtCaptureStart(void)
{
  _nvtCaptureStop();
  _nvtDelayMs(NVT_RESYNC_IDLE_MS);

  _nvtHalvesProduced = 0;
  _nvtHalvesConsumed = 0;
  _nvtSpiError       = false;
  _nvtExpectPkt      = 0;
  _nvtCurSeg         = -1;
  _nvtGotSegs        = 0;
  _nvtBadHalves      = 0;

  if (_nvtHalfSem) {
    while (xSemaphoreTake(_nvtHalfSem, 0) == pdTRUE) { }
  }

  /* Taken so a CCI call already in flight finishes before SCK starts. */
  n6_i2c_lock();
  _nvtCsLow();
  if (HAL_SPI_Receive_DMA(&_nvtSpi, _nvtRing, NVT_DMA_RING_BYTES) != HAL_OK) {
    _nvtLastErr = NVT_VIDEO_ERR_DMA_START;
    _nvtCsHigh();
    n6_i2c_unlock();
    return false;
  }
  _nvtStreaming = true;
  n6_i2c_unlock();
  _nvtRestarts++;
  _nvtProgressTick = HAL_GetTick();
  return true;
}

bool NeuroVisionThermal::videoBegin(void)
{
  /* Set on entry: a begin that returns false still refuses CCI until
   * videoEnd(). */
  _nvtVideoActive = true;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* A repeat call within 60 s of a full init only restarts capture. */
  static uint32_t s_lastFullInitTick = 0;
  static bool     s_fullInitDone = false;
  if (_nvtSpiReady && s_fullInitDone &&
      (HAL_GetTick() - s_lastFullInitTick) < 60000u) {
    _nvtCaptureStop();          /* readFrame() restarts it */
    _nvtLastErr = NVT_VIDEO_OK;
    return true;
  }

  /* No transfer may run across HAL_SPI_Init. */
  _nvtCaptureStop();
  _nvtLastErr = NVT_VIDEO_OK;

  /* SPI1 kernel clock from PCLK2, set once. A configuration error is not
   * fatal: PCLK2 is the reset default. */
  static bool s_kerclkDone = false;
  if (!s_kerclkDone) {
    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
    pclk.Spi1ClockSelection   = RCC_SPI1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) {
      _nvtLastErr = NVT_VIDEO_ERR_KERCLK_CFG;
    }
    s_kerclkDone = true;
  }
  __HAL_RCC_SPI1_CLK_ENABLE();

  static const uint32_t prescVal[8] = {
    SPI_BAUDRATEPRESCALER_2,   SPI_BAUDRATEPRESCALER_4,
    SPI_BAUDRATEPRESCALER_8,   SPI_BAUDRATEPRESCALER_16,
    SPI_BAUDRATEPRESCALER_32,  SPI_BAUDRATEPRESCALER_64,
    SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256,
  };
  _nvtKernelHz = HAL_RCC_GetPCLK2Freq();
  uint32_t kernelHz = _nvtKernelHz;
  if (kernelHz == 0u) {
    _nvtLastErr = NVT_VIDEO_ERR_KERCLK_ZERO;
    kernelHz = 200000000u;   /* the board's PCLK2 */
  }
  uint32_t div = 2u;
  uint32_t pi  = 0u;
  while (pi < 7u && (kernelHz / div) > NVT_SPI_SCK_MAX_HZ) {
    div <<= 1;
    pi++;
  }
  _nvtSckHz = kernelHz / div;

  GPIO_InitTypeDef g = {0};
  g.Mode  = GPIO_MODE_AF_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = NVT_SPI_AF;
  g.Pin = NVT_SPI_SCK_PIN;  HAL_GPIO_Init(NVT_SPI_SCK_PORT,  &g);
  g.Pin = NVT_SPI_MOSI_PIN; HAL_GPIO_Init(NVT_SPI_MOSI_PORT, &g);
  g.Pin = NVT_SPI_MISO_PIN; HAL_GPIO_Init(NVT_SPI_MISO_PORT, &g);

  /* CS is driven high before it becomes an output. */
  HAL_GPIO_WritePin(NVT_SPI_CS_PORT, NVT_SPI_CS_PIN, GPIO_PIN_SET);
  GPIO_InitTypeDef cs = {0};
  cs.Mode  = GPIO_MODE_OUTPUT_PP;
  cs.Pull  = GPIO_NOPULL;
  cs.Speed = GPIO_SPEED_FREQ_HIGH;
  cs.Pin   = NVT_SPI_CS_PIN;
  HAL_GPIO_Init(NVT_SPI_CS_PORT, &cs);

  /* Mode 3, receive only. KeepIOState holds SCK idle across a stop and start,
   * so a restart clocks no extra edge. */
  _nvtSpi.Instance                        = NVT_SPI_INSTANCE;
  _nvtSpi.Init.Mode                       = SPI_MODE_MASTER;
  _nvtSpi.Init.Direction                  = SPI_DIRECTION_2LINES_RXONLY;
  _nvtSpi.Init.DataSize                   = SPI_DATASIZE_8BIT;
  _nvtSpi.Init.CLKPolarity                = SPI_POLARITY_HIGH;
  _nvtSpi.Init.CLKPhase                   = SPI_PHASE_2EDGE;
  _nvtSpi.Init.NSS                        = SPI_NSS_SOFT;
  _nvtSpi.Init.BaudRatePrescaler          = prescVal[pi];
  _nvtSpi.Init.FirstBit                   = SPI_FIRSTBIT_MSB;
  _nvtSpi.Init.TIMode                     = SPI_TIMODE_DISABLE;
  _nvtSpi.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;
  _nvtSpi.Init.CRCPolynomial              = 0x7;
  _nvtSpi.Init.CRCLength                  = SPI_CRC_LENGTH_8BIT;
  _nvtSpi.Init.NSSPMode                   = SPI_NSS_PULSE_DISABLE;
  _nvtSpi.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
  _nvtSpi.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
  _nvtSpi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  _nvtSpi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  _nvtSpi.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
  _nvtSpi.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  _nvtSpi.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  _nvtSpi.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  _nvtSpi.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;
  _nvtSpi.Init.ReadyMasterManagement      = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  _nvtSpi.Init.ReadyPolarity              = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&_nvtSpi) != HAL_OK) {
    _nvtLastErr = NVT_VIDEO_ERR_SPI_INIT;
    return false;
  }

  if (!_nvtDmaInit()) {
    return false;        /* _nvtLastErr names the stage */
  }
  if (_nvtHalfSem == NULL) {
    _nvtHalfSem = xSemaphoreCreateCounting(8, 0);
    if (_nvtHalfSem == NULL) {
      _nvtLastErr = NVT_VIDEO_ERR_SEM;
      return false;
    }
  }

  _nvtSpiReady = true;
  s_fullInitDone = true;
  s_lastFullInitTick = HAL_GetTick();
  /* Capture starts on the first readFrame(). */
  return true;
}

void NeuroVisionThermal::videoEnd(void)
{
  _nvtCaptureStop();
  if (_nvtSpiReady) {
    HAL_SPI_DeInit(&_nvtSpi);
    _nvtSpiReady = false;
  }
  _nvtCsHigh();
  _nvtVideoActive = false;
}

void NeuroVisionThermal::videoResync(void)
{
  /* CS high for NVT_RESYNC_IDLE_MS. Capture restarts on the next readFrame(). */
  _nvtCaptureStop();
  _nvtDelayMs(NVT_RESYNC_IDLE_MS);
}

uint32_t NeuroVisionThermal::videoSckHz(void) const
{
  return _nvtSckHz;
}

uint32_t NeuroVisionThermal::videoKernelHz(void) const
{
  return _nvtKernelHz;
}

int NeuroVisionThermal::videoLastError(void) const
{
  return _nvtLastErr;
}

void NeuroVisionThermal::videoStats(uint32_t &halvesProduced, uint32_t &halvesConsumed,
                                    uint32_t &segments, uint32_t &frames,
                                    uint32_t &restarts) const
{
  halvesProduced = _nvtHalvesProduced;
  halvesConsumed = _nvtHalvesConsumed;
  segments       = _nvtSegsParsed;
  frames         = _nvtFramesParsed;
  restarts       = _nvtRestarts;
}

uint32_t NeuroVisionThermal::videoLastRaw(void) const
{
  return _nvtLastRaw;
}

uint32_t NeuroVisionThermal::videoDiscards(void) const
{
  return _nvtDiscards;
}

uint32_t NeuroVisionThermal::videoSpiErrors(void) const
{
  return _nvtSpiErrCount;
}

uint32_t NeuroVisionThermal::videoSpiErrCode(void) const
{
  return _nvtSpiErrCode;
}

void NeuroVisionThermal::setFlip180(bool on)
{
  _nvtFlip180 = on;
}

bool NeuroVisionThermal::flip180(void) const
{
  return _nvtFlip180;
}

/* Returns true when a complete, in-order frame is in _nvtFrameAcc. Discard
 * packets do not break the packet sequence. */
static bool _nvtParseHalf(const uint8_t *half)
{
  bool frameDone = false;
  int recognized = 0;   /* well-formed packets, discards included */

  for (int k = 0; k < NVT_DMA_PACKETS_PER_HALF; k++) {
    const uint8_t *pkt = &half[k * NVT_VOSPI_PACKET_BYTES];
    if ((pkt[0] & 0x0F) == 0x0F) {
      recognized++;                               /* discard packet */
      _nvtDiscards++;
      continue;
    }
    const int pktnum = pkt[1];
    if (pktnum >= NVT_VOSPI_PACKETS_PER_SEGMENT) {
      _nvtExpectPkt = 0;
      _nvtGotSegs = 0;
      continue;
    }
    recognized++;
    if (_nvtExpectPkt == 0) {
      if (pktnum != 0) continue;
      _nvtCurSeg = -1;
    } else if (pktnum != _nvtExpectPkt) {
      _nvtExpectPkt = 0;
      _nvtGotSegs = 0;
      continue;
    }
    if (pktnum == 20) {
      _nvtCurSeg = (pkt[0] >> 4) & 0x07;          /* segment id, ID bits [14:12] */
    }

    /* 80 big-endian pixels in bytes 4 to 163. */
    const uint8_t *pay = &pkt[4];
    uint16_t *dst = &_nvtSegPix[pktnum * NVT_VOSPI_PACKET_PIXELS];
    for (int x = 0; x < NVT_VOSPI_PACKET_PIXELS; x++) {
      dst[x] = (uint16_t)((uint16_t)pay[2 * x] << 8 | pay[2 * x + 1]);
    }

    _nvtExpectPkt = pktnum + 1;
    if (_nvtExpectPkt < NVT_VOSPI_PACKETS_PER_SEGMENT) {
      continue;
    }
    _nvtExpectPkt = 0;
    _nvtSegsParsed++;

    const int seg = _nvtCurSeg;
    if (seg < 1 || seg > NVT_VOSPI_SEGMENTS) {
      /* Segment id 0 is a filler between frames. */
      _nvtProgressTick = HAL_GetTick();
      continue;
    }
    _nvtProgressTick = HAL_GetTick();

    if (frameDone) {
      /* A frame completed earlier in this half: writing would tear it. */
      _nvtGotSegs = 0;
      continue;
    }
    if (seg == 1) {
      _nvtGotSegs = 0;
    } else if (seg != _nvtGotSegs + 1) {
      _nvtGotSegs = 0;
      continue;
    }
    /* A segment is 30 rows. */
    memcpy(&_nvtFrameAcc[(seg - 1) * 30 * NVT_VOSPI_WIDTH], _nvtSegPix,
           sizeof(_nvtSegPix));
    _nvtGotSegs = seg;
    if (seg == NVT_VOSPI_SEGMENTS) {
      _nvtGotSegs = 0;
      frameDone = true;
    }
  }

  /* A well-formed half counts as progress: the Lepton streams discards for
   * seconds at a time. */
  if (recognized >= (NVT_DMA_PACKETS_PER_HALF * 9) / 10) {
    _nvtProgressTick = HAL_GetTick();
  }
  _nvtBadHalves = (recognized < NVT_DMA_PACKETS_PER_HALF / 2) ? _nvtBadHalves + 1u : 0u;
  return frameDone;
}

bool NeuroVisionThermal::readFrame(uint16_t *frame, uint32_t timeoutMs)
{
  if (!_nvtSpiReady || frame == nullptr || _nvtHalfSem == NULL) {
    return false;
  }
  if (!_nvtStreaming && !_nvtCaptureStart()) {
    return false;
  }

  /* Elapsed time, not a deadline, so the tick may wrap. */
  const uint32_t start = HAL_GetTick();
  for (;;) {
    /* 1.5 s: a flat-field correction pauses output for about a second. */
    if (_nvtSpiError || (HAL_GetTick() - _nvtProgressTick) > 1500u ||
        _nvtBadHalves >= NVT_BAD_HALVES_MAX) {
      if (!_nvtCaptureStart()) {
        return false;
      }
    }

    /* Parser overrun: skip to the newest complete slot. */
    {
      const uint32_t produced = _nvtHalvesProduced;   /* the interrupt writes it */
      if ((produced - _nvtHalvesConsumed) > (NVT_SW_SLOTS - 2u)) {
        _nvtHalvesConsumed = produced - 1u;
        _nvtExpectPkt = 0;
        _nvtGotSegs = 0;
      }
    }

    if (_nvtHalvesProduced == _nvtHalvesConsumed) {
      const uint32_t elapsed = HAL_GetTick() - start;
      if (elapsed >= timeoutMs) {
        return false;
      }
      uint32_t waitMs = timeoutMs - elapsed;
      if (waitMs > 50u) waitMs = 50u;
      (void)xSemaphoreTake(_nvtHalfSem, pdMS_TO_TICKS(waitMs));
      continue;
    }

    uint8_t *half = _nvtSwRing[_nvtHalvesConsumed % NVT_SW_SLOTS];
    _nvtLastRaw = ((uint32_t)half[0] << 24) | ((uint32_t)half[1] << 16) |
                  ((uint32_t)half[2] << 8) | (uint32_t)half[3];
    const bool gotFrame = _nvtParseHalf(half);
    _nvtHalvesConsumed++;

    if (gotFrame) {
      _nvtFramesParsed++;
      if (_nvtFlip180) {
        for (int i = 0; i < NVT_VOSPI_PIXELS; i++) {
          frame[i] = _nvtFrameAcc[NVT_VOSPI_PIXELS - 1 - i];
        }
      } else {
        memcpy(frame, _nvtFrameAcc, sizeof(_nvtFrameAcc));
      }
      return true;
    }
    if ((HAL_GetTick() - start) >= timeoutMs) {
      return false;
    }
  }
}

/* Neuro Studio streaming */

bool NeuroVisionThermal::streamBegin(void)
{
  NeuroN6_StreamUsbOnlyStart();
  return videoBegin();
}

bool NeuroVisionThermal::streamFrame(const uint16_t *frame, bool tlinear)
{
  if (frame == nullptr) {
    return false;
  }

  const uint16_t vlen = (uint16_t)(8 + NVT_VOSPI_PIXELS * 2);   /* 38408 */
  uint8_t hdr[11];
  hdr[0] = (uint8_t)NVT_META_THERMAL_FRAME;                     /* TLV type */
  hdr[1] = (uint8_t)(vlen & 0xFF);                              /* TLV length, LE */
  hdr[2] = (uint8_t)(vlen >> 8);
  hdr[3] = (uint8_t)(NVT_VOSPI_WIDTH & 0xFF);                   /* width, LE */
  hdr[4] = (uint8_t)(NVT_VOSPI_WIDTH >> 8);
  hdr[5] = (uint8_t)(NVT_VOSPI_HEIGHT & 0xFF);                  /* height, LE */
  hdr[6] = (uint8_t)(NVT_VOSPI_HEIGHT >> 8);
  hdr[7] = (uint8_t)NVT_THERMAL_FMT_RAW16;                      /* format */
  hdr[8] = (uint8_t)(tlinear ? NVT_THERMAL_FLAG_TLINEAR : 0u);  /* flags */
  hdr[9]  = 0;                                                  /* reserved */
  hdr[10] = 0;

  vnd_meta_builder_t b;
  vnd_meta_builder_begin(&b, VND_META_SLOT_THERMAL);
  if (!vnd_meta_builder_append_bytes(&b, hdr, sizeof(hdr))) return false;
  /* The CPU is little-endian, so the frame is already RAW16 LE. */
  if (!vnd_meta_builder_append_bytes(&b, (const uint8_t *)frame,
                                     (uint32_t)(NVT_VOSPI_PIXELS * 2))) return false;
  vnd_meta_builder_finish(&b);
  return true;
}

bool NeuroVisionThermal::streamAnnounce(bool tlinear)
{
  /* The streamFrame() header with no pixels. */
  const uint16_t vlen = 8u;
  uint8_t tlv[11];
  tlv[0] = (uint8_t)NVT_META_THERMAL_FRAME;
  tlv[1] = (uint8_t)(vlen & 0xFF);
  tlv[2] = (uint8_t)(vlen >> 8);
  tlv[3] = (uint8_t)(NVT_VOSPI_WIDTH & 0xFF);
  tlv[4] = (uint8_t)(NVT_VOSPI_WIDTH >> 8);
  tlv[5] = (uint8_t)(NVT_VOSPI_HEIGHT & 0xFF);
  tlv[6] = (uint8_t)(NVT_VOSPI_HEIGHT >> 8);
  tlv[7] = (uint8_t)NVT_THERMAL_FMT_RAW16;
  tlv[8] = (uint8_t)(tlinear ? NVT_THERMAL_FLAG_TLINEAR : 0u);
  tlv[9]  = 0;
  tlv[10] = 0;

  vnd_meta_builder_t b;
  vnd_meta_builder_begin(&b, VND_META_SLOT_THERMAL);
  if (!vnd_meta_builder_append_bytes(&b, tlv, sizeof(tlv))) return false;
  vnd_meta_builder_finish(&b);
  return true;
}

/* Thermal service: capture, publish and recovery on one task */

static TaskHandle_t        s_svcTask       = NULL;
static NeuroVisionThermal *s_svcSelf       = NULL;
static bool                s_svcTlinear    = true;
static bool                s_svcVideoOk    = false;   /* VoSPI and DMA are up */
static uint32_t            s_svcOk         = 0u;      /* frames streamed */
static uint32_t            s_svcFail       = 0u;      /* readFrame timeouts */
static uint32_t            s_svcConsecFail = 0u;      /* consecutive timeouts */
static uint16_t            s_svcFrame[NVT_VOSPI_PIXELS];

static void _svcLog(const char *text)
{
  NeuroN6_DbgPrint(text);
  (void)vnd_meta_publish_debug_text(text);
}

#if !defined(NEURON6_DEBUG_CONSOLE) || (NEURON6_DEBUG_CONSOLE != 0)
/* Capture counters, every 6 s, with the debug console built in. */
static void _svcStatus(void)
{
  uint32_t hp = 0, hc = 0, segs = 0, frm = 0, rst = 0;
  s_svcSelf->videoStats(hp, hc, segs, frm, rst);
  char line[200];
  snprintf(line, sizeof(line),
           "[thermal] status vospi=%u err=%d sck=%lu dma=%lu/%lu segs=%lu disc=%lu frm=%lu rst=%lu ok=%lu to=%lu raw=%08lX spe=%lu/%02lX\r\n",
           s_svcVideoOk ? 1u : 0u,
           s_svcSelf->videoLastError(),
           (unsigned long)s_svcSelf->videoSckHz(),
           (unsigned long)hp, (unsigned long)hc,
           (unsigned long)segs, (unsigned long)s_svcSelf->videoDiscards(),
           (unsigned long)frm, (unsigned long)rst,
           (unsigned long)s_svcOk, (unsigned long)s_svcFail,
           (unsigned long)s_svcSelf->videoLastRaw(),
           (unsigned long)s_svcSelf->videoSpiErrors(),
           (unsigned long)s_svcSelf->videoSpiErrCode());
  _svcLog(line);
}
#endif

/* Uses no I2C, so it can run while a camera streams. */
static bool _svcVideoStart(const char *prefix)
{
  const bool ok = s_svcSelf->videoBegin();
  char line[140];
  snprintf(line, sizeof(line),
           "[thermal] %s vospi=%u err=%d kernel=%luHz sck=%luHz\r\n",
           prefix, ok ? 1u : 0u,
           s_svcSelf->videoLastError(),
           (unsigned long)s_svcSelf->videoKernelHz(),
           (unsigned long)s_svcSelf->videoSckHz());
  _svcLog(line);
  return ok;
}

/* The service task never uses CCI. Below the ISEG post-process priority the
 * parser is held off and the frame rate falls. */
static void _svcTaskBody(void *arg)
{
  (void)arg;
  uint32_t lastAnnounceTick = 0u;
#if !defined(NEURON6_DEBUG_CONSOLE) || (NEURON6_DEBUG_CONSOLE != 0)
  uint32_t lastStatusTick = 0u;
#endif
  for (;;) {
#if !defined(NEURON6_DEBUG_CONSOLE) || (NEURON6_DEBUG_CONSOLE != 0)
    if ((xTaskGetTickCount() - lastStatusTick) >= pdMS_TO_TICKS(6000)) {
      lastStatusTick = xTaskGetTickCount();
      _svcStatus();
    }
#endif

    if (!s_svcVideoOk) {
      s_svcVideoOk = _svcVideoStart("retry-vospi");
      if (!s_svcVideoOk) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }
      s_svcSelf->streamAnnounce(s_svcTlinear);
      lastAnnounceTick = xTaskGetTickCount();
    }

    /* A frame arrives every 115 ms. */
    const bool gotFrame = s_svcSelf->readFrame(s_svcFrame, 1000);

    if (gotFrame) {
      s_svcSelf->streamFrame(s_svcFrame, s_svcTlinear);
      s_svcOk++;
      s_svcConsecFail = 0u;
      if (s_svcOk == 1u || (s_svcOk % 256u) == 0u) {
        char tb[64];
        snprintf(tb, sizeof(tb), "[thermal] frame ok #%lu\r\n", (unsigned long)s_svcOk);
        _svcLog(tb);
      }
    } else {
      s_svcFail++;
      s_svcConsecFail++;
      if (s_svcFail == 1u || (s_svcFail % 20u) == 0u) {
        char tb[64];
        snprintf(tb, sizeof(tb), "[thermal] readFrame timeout #%lu err=%d\r\n",
                 (unsigned long)s_svcFail, s_svcSelf->videoLastError());
        _svcLog(tb);
      }
      /* Every 20 timeouts videoBegin() runs again, and every third time after a
       * power cycle with the I2C bus claimed until the first frame. */
      if (s_svcConsecFail >= 20u) {
        s_svcConsecFail = 0u;
        static uint32_t s_recoverCycles = 0u;
        s_recoverCycles++;
        if ((s_recoverCycles % 3u) == 0u) {
          _svcLog("[thermal] no frames - power-cycling Lepton (bus claimed through video-start)\r\n");
          n6_i2c_claim();
          s_svcSelf->powerOn();
          vTaskDelay(pdMS_TO_TICKS(2500));   /* boot time */
          s_svcVideoOk = _svcVideoStart("recover-vospi");
          if (s_svcVideoOk) {
            const uint32_t t0 = xTaskGetTickCount();
            while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(5000)) {
              if (s_svcSelf->readFrame(s_svcFrame, 900)) {
                s_svcSelf->streamFrame(s_svcFrame, s_svcTlinear);
                s_svcOk++;
                break;
              }
            }
          }
          n6_i2c_release();
        } else {
          s_svcVideoOk = false;          /* videoBegin() on the next pass */
        }
        continue;
      }
      if (s_svcOk == 0u &&
          (xTaskGetTickCount() - lastAnnounceTick) >= pdMS_TO_TICKS(2000)) {
        s_svcSelf->streamAnnounce(s_svcTlinear);
        lastAnnounceTick = xTaskGetTickCount();
      }
    }
  }
}

bool NeuroVisionThermal::serviceBegin(bool tlinear, uint32_t firstFrameCapMs)
{
  if (s_svcTask != NULL) {
    return true;
  }
  s_svcSelf    = this;
  s_svcTlinear = tlinear;

  /* The I2C bus is claimed from the power cycle to the first frame. */
  n6_i2c_claim();
  powerOn();
  s_svcVideoOk = _svcVideoStart("setup");
  if (s_svcVideoOk) {
    /* The first frame arrives about 2.5 s after the power cycle. */
    const uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < firstFrameCapMs) {
      if (readFrame(s_svcFrame, 900)) {
        streamFrame(s_svcFrame, tlinear);
        s_svcOk = 1u;
        streamAnnounce(tlinear);
        break;
      }
    }
    char bl[96];
    snprintf(bl, sizeof(bl), "[thermal] first frame %s after %lu ms\r\n",
             s_svcOk ? "OK" : "MISSING",
             (unsigned long)(HAL_GetTick() - t0));
    _svcLog(bl);
  }
  n6_i2c_release();

  {
    char bl[120];
    snprintf(bl, sizeof(bl),
             "[thermal] service bringup vospi=%u firstFrame=%u flip180=%u\r\n",
             s_svcVideoOk ? 1u : 0u, s_svcOk ? 1u : 0u, flip180() ? 1u : 0u);
    _svcLog(bl);
  }

  /* Stack: 2048 words, 8 KB. */
  if (xTaskCreate(_svcTaskBody, "thermal", 2048, NULL,
                  tskIDLE_PRIORITY + 3u, &s_svcTask) != pdPASS) {
    s_svcTask = NULL;
    _svcLog("[thermal] service task create FAILED\r\n");
    return false;
  }
  return s_svcOk != 0u;
}
