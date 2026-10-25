/**
 * @file    PowerSaveCamera.cpp
 * @brief   One-shot VD66GY frame capture.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "PowerSaveCamera.h"

#include "NeuroN6_app.h"
#include "NeuroN6_FrameQueue.h"
#include "stm32n6xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>
#include <string.h>

PowerSaveCameraClass PowerSaveCam;

/* Set by the core's PIPE2 frame-event callback. */
extern "C" volatile int32_t cameraFrameReceived;
/* Set by the core's PIPE1 frame-event callback. */
extern "C" volatile int32_t pipe1FrameReceived;

/* VENC JPEG encoder. It consumes the YUV422 that PIPE1 emits. */
extern "C" int      n6_jpeg_band_setup(uint32_t cam_w, uint32_t cam_h);
extern "C" uint32_t n6_jpeg_capture_one(uint8_t *out_buf);
extern "C" void     encoder_jpeg_linebuf_ensure_init(int32_t w, int32_t h, uint32_t szOut);
extern "C" void     n6_linebuf_prime_pipe(void);
extern "C" void     encoder_jpeg_end(void);
/* The core's NN-path handshake, held at 1 outside a capture. */
extern "C" volatile int32_t pipe1ready;

/* VD66GY I2C address and registers */
#define PSCAM_VD6G_ADDR8       0x20u     /* 7-bit 0x10 */
#define PSCAM_REG_SYSTEM_FSM   0x0028u
#define PSCAM_FSM_SW_STBY      0x02u
#define PSCAM_FSM_STREAMING    0x03u
#define PSCAM_REG_STBY         0x0201u   /* write 1 = start streaming */
#define PSCAM_REG_STREAMING    0x0202u   /* write 1 = stop streaming */

/* DMA buffers are aligned to the 32-byte cache line. */
static uint8_t *ps_alloc_aligned(uint32_t bytes, uint8_t **rawOut)
{
  uint32_t padded = (bytes + 31u) & ~31u;
  uint8_t *raw = (uint8_t *)malloc(padded + 32u);
  if (!raw) { *rawOut = nullptr; return nullptr; }
  *rawOut = raw;
  return (uint8_t *)(((uint32_t)raw + 31u) & ~31u);
}

static void ps_cache_invalidate(uint8_t *buf, uint32_t bytes)
{
  SCB_InvalidateDCache_by_Addr((uint32_t *)buf, (int32_t)((bytes + 31u) & ~31u));
}

static void ps_yield_ms(uint32_t ms)
{
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    vTaskDelay(pdMS_TO_TICKS(ms));
  } else {
    HAL_Delay(ms);
  }
}

/* Sensor register access */
bool PowerSaveCameraClass::vd6gWrite8(uint16_t reg, uint8_t v)
{
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, PSCAM_VD6G_ADDR8, reg,
                                           I2C_MEMADD_SIZE_16BIT, &v, 1u, 100u);
  n6_i2c_unlock();
  return st == HAL_OK;
}

bool PowerSaveCameraClass::vd6gRead8(uint16_t reg, uint8_t &v)
{
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, PSCAM_VD6G_ADDR8, reg,
                                          I2C_MEMADD_SIZE_16BIT, &v, 1u, 100u);
  n6_i2c_unlock();
  return st == HAL_OK;
}

bool PowerSaveCameraClass::vd6gPoll8(uint16_t reg, uint8_t want, uint32_t timeoutMs)
{
  uint32_t t0 = millis();
  for (;;) {
    uint8_t v = 0xFFu;
    if (vd6gRead8(reg, v) && v == want) return true;
    if ((millis() - t0) > timeoutMs) return false;
    ps_yield_ms(2);
  }
}

/* PIPE1 assist: PIPE1 runs line-wrapped into a scratch band, which keeps the
 * RAW8 demosaic stage it shares with PIPE2 fed. */
void PowerSaveCameraClass::assistStart()
{
  if (!_assist || _band == nullptr) return;
  DCMIPP_HandleTypeDef *h = (DCMIPP_HandleTypeDef *)n6_dcmipp_handle();
  (void)HAL_DCMIPP_PIPE_SetLineWrappingConfig(h, DCMIPP_PIPE1, DCMIPP_WRAP_ADDRESS_32_LINES);
  (void)HAL_DCMIPP_PIPE_EnableLineWrapping(h, DCMIPP_PIPE1);
  SET_BIT(DCMIPP->CMCR, DCMIPP_CMCR_INSEL);   /* selects the CSI input */
  n6_dcmipp_prepare_start(DCMIPP_PIPE1);
  (void)HAL_DCMIPP_CSI_PIPE_Start(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0,
                                  (uint32_t)_band, DCMIPP_MODE_CONTINUOUS);
}

void PowerSaveCameraClass::assistStop()
{
  if (!_assist || _band == nullptr) return;
  DCMIPP_HandleTypeDef *h = (DCMIPP_HandleTypeDef *)n6_dcmipp_handle();
  (void)HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE1, DCMIPP_VIRTUAL_CHANNEL0);
}

bool PowerSaveCameraClass::begin(CameraResolutionN6 res, uint16_t outW,
                                 uint16_t outH, CameraFpsN6 fps)
{
  if (_begun) return true;

  /* Without a file-scope sensor object the core does not run CSI_Init. */
  if (_neuron6_csi_lane_mapping == NEURON6_CAMERA_NONE || g_n6_active_sensor == nullptr) {
    return false;
  }

  /* init() selects the mode and reports its size. The driver starts in
   * bringUp(). */
  uint16_t w = 0, h = 0;
  if (g_n6_active_sensor->init(res, fps, CAMERA_AUTO_FLIP, w, h) != 0 || !w || !h) {
    return false;
  }
  _camW = w; _camH = h;
  _outW = outW ? outW : w;
  _outH = outH ? outH : h;

  SensorFormat sf;
  g_n6_active_sensor->getFormat(sf);
  n6_apply_sensor_format((int)sf.pixel_format, (int)sf.color_space, (int)sf.bayer_order);
  n6_csi_set_phy_bitrate(g_n6_active_sensor->csiLaneRateMbps());

  /* Cleared before DCMIPP_USB_Init, which otherwise initialises the JPEG
   * encoder and halts in Error_Handler(). */
  g_n6_usb_enabled = 0u;

  DCMIPP_USB_Init(_camW, _camH);
  DCMIPP_NN_InitWithCrop(_outW, _outH, 0u, 0u, 0u, 0u);

  /* Clears P2PPCR.SWAPRB, which the core sets. Written directly as well: the
   * HAL call has no effect unless DCMIPP is READY. */
  {
    DCMIPP_HandleTypeDef *hd = (DCMIPP_HandleTypeDef *)n6_dcmipp_handle();
    (void)HAL_DCMIPP_PIPE_DisableRedBlueSwap(hd, DCMIPP_PIPE2);
    if ((DCMIPP->P2PPCR & DCMIPP_P2PPCR_SWAPRB) != 0u) {
      CLEAR_BIT(DCMIPP->P2PPCR, DCMIPP_P2PPCR_SWAPRB);
    }
  }

  if (!_jpegMode) {
    /* Heap first, then PSRAM. A full-resolution capture into PSRAM overruns
     * and capture() times out. */
    const uint32_t rgbBytes = (uint32_t)_outW * _outH * 3u;
    _buf = ps_alloc_aligned(rgbBytes, &_bufRaw);
    if (_buf == nullptr) {
      _buf = (uint8_t *)n6_psram_alloc(rgbBytes);
      _bufPsram = (_buf != nullptr);
      if (_buf == nullptr) return false;
    }
  }
  if (_assist && !_jpegMode) {
    /* 32 wrapped lines at 2 bytes per pixel. */
    _band = ps_alloc_aligned((uint32_t)_camW * 2u * 32u, &_bandRaw);
    if (_band == nullptr) { _assist = false; }
  }

  /* VD6G_Init and VD6G_Start need the USB pump task. Starting it arms the
   * core's 8 s IWDG. */
  NeuroN6_StreamUsbOnlyStart();
  ps_yield_ms(20);   /* lets the pump task run */

  /* bringUp() starts auto-exposure and AWB only if g_isp_tuning_mode is 2. */
  extern volatile uint8_t g_isp_tuning_mode;
  g_isp_tuning_mode = 2u;
  g_n6_active_sensor->bringUp();

  _streaming = true;

  if (_jpegMode) {
    /* PSRAM, not malloc: the VENC's DMA master cannot reach the AXISRAM heap.
     * JPEG mode therefore excludes PowerSave.psramSleep(). */
    _jpegMax = ((uint32_t)_camW * _camH * 2u) / 4u;
    _jpeg    = (uint8_t *)n6_psram_alloc(_jpegMax);
    if (_jpeg == nullptr) { return false; }

    if (!n6_jpeg_band_setup(_camW, _camH)) { return false; }

    encoder_jpeg_linebuf_ensure_init((int32_t)_camW, (int32_t)_camH, _jpegMax);
    /* PIPE1 is started once and stopped here. Starting a PIPE1 that has never
     * run resets the encoder. */
    n6_linebuf_prime_pipe();
    _encoderUp = true;
  } else {
    assistStart();
  }

  _begun = true;
  return true;
}

uint32_t PowerSaveCameraClass::captureJpeg(uint32_t timeoutMs)
{
  (void)timeoutMs;   /* unused: the encoder blocks for about one frame */
  if (!_begun || !_streaming || !_encoderUp || _jpeg == nullptr) return 0;

  _jpegLen = n6_jpeg_capture_one(_jpeg);
  return _jpegLen;
}

bool PowerSaveCameraClass::capture(uint32_t timeoutMs)
{
  if (!_begun || !_streaming || _buf == nullptr) return false;

  DCMIPP_HandleTypeDef *h = (DCMIPP_HandleTypeDef *)n6_dcmipp_handle();
  const uint32_t bytes = frameBytes();

  /* Invalidated before the DMA writes. PSRAM is noncacheable. */
  if (!_bufPsram) ps_cache_invalidate(_buf, bytes);

  cameraFrameReceived = 0;
  pipe1ready = 0;
  n6_dcmipp_prepare_start(DCMIPP_PIPE2);
  if (HAL_DCMIPP_CSI_PIPE_Start(h, DCMIPP_PIPE2, DCMIPP_VIRTUAL_CHANNEL0,
                                (uint32_t)_buf, DCMIPP_MODE_SNAPSHOT) != HAL_OK) {
    /* A capture that timed out can leave the pipe BUSY. */
    (void)HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE2, DCMIPP_VIRTUAL_CHANNEL0);
    n6_dcmipp_prepare_start(DCMIPP_PIPE2);
    if (HAL_DCMIPP_CSI_PIPE_Start(h, DCMIPP_PIPE2, DCMIPP_VIRTUAL_CHANNEL0,
                                  (uint32_t)_buf, DCMIPP_MODE_SNAPSHOT) != HAL_OK) {
      pipe1ready = 1;
      return false;
    }
  }

  uint32_t t0 = millis();
  while (cameraFrameReceived == 0) {
    if ((millis() - t0) > timeoutMs) {
      (void)HAL_DCMIPP_CSI_PIPE_Stop(h, DCMIPP_PIPE2, DCMIPP_VIRTUAL_CHANNEL0);
      pipe1ready = 1;
      return false;
    }
    ps_yield_ms(1);
  }
  cameraFrameReceived = 0;
  pipe1ready = 1;

  /* The DMA wrote behind the D-cache. */
  if (!_bufPsram) ps_cache_invalidate(_buf, bytes);
  return true;
}

/* Sensor sleep and wake */
void PowerSaveCameraClass::sensorSleep()
{
  if (!_begun || !_streaming) return;
  assistStop();
  if (vd6gWrite8(PSCAM_REG_STREAMING, 1u)) {
    (void)vd6gPoll8(PSCAM_REG_STREAMING, 0u, 100u);
    (void)vd6gPoll8(PSCAM_REG_SYSTEM_FSM, PSCAM_FSM_SW_STBY, 100u);
  }
  _streaming = false;
}

bool PowerSaveCameraClass::sensorWake(uint32_t timeoutMs)
{
  if (!_begun) return false;
  if (_streaming) return true;

  busWake();

  if (!vd6gWrite8(PSCAM_REG_STBY, 1u)) return false;
  if (!vd6gPoll8(PSCAM_REG_STBY, 0u, timeoutMs)) return false;
  if (!vd6gPoll8(PSCAM_REG_SYSTEM_FSM, PSCAM_FSM_STREAMING, timeoutMs)) return false;

  _streaming = true;
  assistStart();
  return true;
}

/* Camera bus clock gating */
void PowerSaveCameraClass::busSleep()
{
  /* Interface off, the D-PHY regulator off through PRCR.PEN, then the clocks.
   * busWake() reverses the order. */
  CLEAR_BIT(CSI->CR, CSI_CR_CSIEN);
  CLEAR_BIT(CSI->PRCR, CSI_PRCR_PEN);
  /* Read back so both writes retire before the clock stops. A write still in
   * flight would stall APB5. */
  (void)READ_REG(CSI->PRCR);
  __DSB();
  __HAL_RCC_DCMIPP_CLK_DISABLE();
  __HAL_RCC_CSI_CLK_DISABLE();
}

void PowerSaveCameraClass::busWake()
{
  __HAL_RCC_CSI_CLK_ENABLE();
  __HAL_RCC_DCMIPP_CLK_ENABLE();
  SET_BIT(CSI->PRCR, CSI_PRCR_PEN);
  HAL_Delay(1);                     /* PHY regulator settle */
  SET_BIT(CSI->CR, CSI_CR_CSIEN);
}
