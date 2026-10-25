/**
  ******************************************************************************
  * @file    imageEncode.c
  * @brief   Hardware JPEG encoder wrapper using STM32N6 VENC peripheral
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025-2026 Ohm Lab Ltd.
  * Portions Copyright (c) STMicroelectronics: made from imageEncode.c of
  * the VENC_JPEG_Encoding application in STM32CubeN6, licensed under
  * SLA0044. See AI_Runtime/LICENSE.txt.
  * SPDX-License-Identifier: MIT AND LicenseRef-ST-SLA0044
  *
  ******************************************************************************
  */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "jpegencapi.h"
#include "stm32n6xx_ll_venc.h"
#include "stm32n6xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ewl.h"

typedef struct jpegInfo_t
{
  uint32_t  frameSeqNum;
  uint32_t  frameHeader;
} jpegInfo_t;

typedef struct venc_jpeg_cfg_t
{
  JpegEncCfg   cfgJpeg;
  jpegInfo_t   cfgJpegInfo;
} venc_jpeg_cfg_t;

venc_jpeg_cfg_t hVencJpegPluginInstance = {
  .cfgJpeg.qLevel            = 2,                               /* 0-9 */
  .cfgJpeg.frameType         = JPEGENC_YUV422_INTERLEAVED_YUYV,
  .cfgJpeg.xOffset           = 0,
  .cfgJpeg.yOffset           = 0,
  .cfgJpeg.restartInterval   = 0,
  .cfgJpegInfo.frameHeader   = 1,
  .cfgJpegInfo.frameSeqNum   = 1,
  .cfgJpeg.rotation          = JPEGENC_ROTATE_0,
  .cfgJpeg.codingType        = JPEGENC_WHOLE_FRAME,
  .cfgJpeg.codingMode        = JPEGENC_422_MODE,
  .cfgJpeg.unitsType         = JPEGENC_DOTS_PER_INCH,
  .cfgJpeg.markerType        = JPEGENC_SINGLE_MARKER,
  .cfgJpeg.xDensity          = 72,
  .cfgJpeg.yDensity          = 72,
};

static JpegEncInst jpegInstance;
JpegEncIn   jpegEncIn  = {0};
static uint32_t *pOutput;
static uint32_t  szOutput;
extern volatile int32_t g_dbg_ff_init_ret;   /* full-frame init return */
extern volatile int32_t  g_dbg_ff_jeinit, g_dbg_ff_jesps;          /* JpegEncInit, SetPictureSize returns */
extern volatile uint32_t g_dbg_swreg_lo[64], g_dbg_swreg_hi[16];   /* VENC register snapshot */
extern volatile uint8_t  g_dbg_venc_arm, g_dbg_venc_print;

/* Line-buffer encoder instance. The EWL is a singleton, so it and the
 * full-frame instance are mutually exclusive. */
static JpegEncInst lbInstance;
static int lb_inited = 0;

uint8_t encoder_jpeg_init(uint32_t type, int32_t width, int32_t height, void *pOut, uint32_t szOut)
{
  int32_t ret;
  /* A sensor that defers streaming (VD66GY) opens the line-buffer encoder
   * first. It keeps the VENC; a second instance would take over the EWL. */
  if (lb_inited)
    return 1;
  LL_VENC_Init();
  pOutput  = pOut;
  szOutput = szOut;
  /* The 4:2:2 MCU is 16x8, so the coded height must be a multiple of 8. */
  height = (int32_t)((uint32_t)height & ~7u);
  hVencJpegPluginInstance.cfgJpeg.inputWidth  = width;
  hVencJpegPluginInstance.cfgJpeg.inputHeight = height;
  hVencJpegPluginInstance.cfgJpeg.codingWidth  = width;
  hVencJpegPluginInstance.cfgJpeg.codingHeight = height;
  hVencJpegPluginInstance.cfgJpeg.frameType    = (JpegEncFrameType)type;
  /* The encoder rejects 4:2:2 coding for RGB input. */
  hVencJpegPluginInstance.cfgJpeg.codingMode   =
      ((type == (uint32_t)JPEGENC_YUV422_INTERLEAVED_YUYV) ||
       (type == (uint32_t)JPEGENC_YUV422_INTERLEAVED_UYVY))
      ? JPEGENC_422_MODE : JPEGENC_420_MODE;

  if (pOutput == NULL)
    return 0;

  ret = JpegEncInit(&hVencJpegPluginInstance.cfgJpeg, &jpegInstance);
  g_dbg_ff_jeinit   = (int32_t)ret;
  g_dbg_ff_init_ret = (int32_t)ret;
  if (ret != JPEGENC_OK)
    return 0;

  ret = JpegEncSetPictureSize(jpegInstance, &hVencJpegPluginInstance.cfgJpeg);
  g_dbg_ff_jesps    = (int32_t)ret;
  g_dbg_ff_init_ret = (int32_t)ret;
  if (ret != JPEGENC_OK)
    return 0;

  memset(&jpegEncIn, 0, sizeof(jpegEncIn));
  jpegEncIn.pOutBuf    = (u8 *)pOutput;
  jpegEncIn.busOutBuf  = (uint32_t)pOutput;
  jpegEncIn.outBufSize = szOutput;
  return 1;
}

/* Sets the output buffer for the next encode. Its size stays szOutput. */
void encoder_jpeg_set_output(void *pOut)
{
  pOutput             = (uint32_t *)pOut;
  jpegEncIn.pOutBuf   = (u8 *)pOut;
  jpegEncIn.busOutBuf = (uint32_t)pOut;
}

void encoder_jpeg_end(void)
{
  LL_VENC_DeInit();
  /* By value: JpegEncRelease(&jpegInstance) frees nothing. */
  JpegEncRelease(jpegInstance);
  jpegInstance = NULL;
}

/* VENC capability probes */

/* Fills out[0..7] with the VENC ASIC fuse bits. The line-buffer RGB565 path
 * needs [0] and [2] non-zero. */
void encoder_jpeg_caps(uint32_t out[8])
{
  LL_VENC_Init();
  EWLHwConfig_t c = EWLReadAsicConfig();
  out[0] = c.rgbEnabled;          /* RGB input */
  out[1] = c.jpegEnabled;
  out[2] = c.instantSupport;      /* input line-buffer interface */
  out[3] = c.inLoopbackSupport;   /* input ping-pong */
  out[4] = c.irqEnhanceSupport;   /* IRQ cleared by writing 1 */
  out[5] = c.maxEncodedWidth;
  out[6] = c.busWidth;            /* AXI bus width in 32-bit words */
  out[7] = EWLReadAsicID();       /* VENC register 0 */
}

/* Returns 1 when a malloc of the band size succeeds. *band_out gets the size
 * and *addr_out the freed address. A successful probe raises the heap
 * high-water mark permanently: _sbrk never shrinks. */
int encoder_jpeg_band_probe(uint32_t width, uint32_t wrap_lines, uint32_t *band_out, void **addr_out)
{
  uint32_t pitch16 = ((width * 2u) + 15u) & ~15u;
  uint32_t band    = pitch16 * wrap_lines;
  void *p = malloc(band);
  if (band_out) *band_out = band;
  if (addr_out) *addr_out = p;
  if (p) { free(p); return 1; }
  return 0;
}

/* Line-buffer JPEG encoder, fed from a band the DCMIPP fills. */

extern volatile uint32_t g_dcmipp_wr_mblines;
extern uint32_t g_venc_total_mblines;
extern uint8_t *g_venc_band;
extern uint32_t g_venc_band_size, g_venc_band_depth;
extern volatile uint32_t g_dbg_lb_cb_calls;
extern volatile int32_t  g_dbg_lb_init_ret;
/* Registers the band as the encoder's input line buffer. size is in bytes. */
extern void EWLSetInputLineBufferBase(uint32_t *virt, uint32_t bus, uint32_t size);
extern volatile uint8_t g_venc_lb_hw;   /* 1 = use the hardware handshake */
extern volatile int32_t g_dbg_lb_enc_rv;   /* raw line-buffer encode return */
extern volatile uint32_t g_dbg_lb_instant;  /* VENC register 0x7C4 */

/* Geometry of the last successful init, for the ES0620 reset. */
static int32_t  s_lb_w     = 0;
static int32_t  s_lb_h     = 0;
static uint32_t s_lb_szOut = 0;

/* Idempotent. Releases the full-frame instance and initialises lbInstance. */
void encoder_jpeg_linebuf_ensure_init(int32_t width, int32_t height, uint32_t szOut)
{
  if (lb_inited)
    return;

  /* The band is a DMA target from the non-cacheable pool, never from malloc. */
  if (g_venc_band == NULL)
    return;

  LL_VENC_Init();

  /* By value, as in encoder_jpeg_end(). */
  if (jpegInstance != NULL) {
    JpegEncRelease(jpegInstance);
    jpegInstance = NULL;
  }

  szOutput = szOut;

  /* The band is fed as YUV422: RGB565 would hit the VENC's RGB-preprocessor
   * width limit at 1080p and above. */
  JpegEncCfg cfg = hVencJpegPluginInstance.cfgJpeg;
  height = (int32_t)((uint32_t)height & ~7u);
  cfg.inputWidth=cfg.codingWidth=width; cfg.inputHeight=cfg.codingHeight=height;
  cfg.frameType = JPEGENC_YUV422_INTERLEAVED_YUYV; cfg.codingMode = JPEGENC_422_MODE;
  cfg.inputLineBufEn = 1; cfg.inputLineBufLoopBackEn = 1;
  cfg.inputLineBufDepth = g_venc_band_depth;
  cfg.inputLineBufHwModeEn = g_venc_lb_hw ? 1u : 0u;   /* sets 0x7C4 bit29 and skips the software cbFunc */
  EWLSetInputLineBufferBase((uint32_t*)g_venc_band, (uint32_t)g_venc_band, g_venc_band_size);
  /* On rejection the full-frame encoder is restored. The band is not from
   * malloc, so it is cleared and not freed. */
  g_dbg_lb_init_ret = (int32_t)JpegEncInit(&cfg, &lbInstance);
  if (g_dbg_lb_init_ret == JPEGENC_OK) {
    g_dbg_lb_init_ret = (int32_t)JpegEncSetPictureSize(lbInstance, &cfg);
    if (g_dbg_lb_init_ret != JPEGENC_OK) {
      JpegEncRelease(lbInstance);   /* by value */
      lbInstance = NULL;
    }
  }
  if (g_dbg_lb_init_ret != JPEGENC_OK) {
    g_venc_band = NULL; g_venc_band_size = 0u;
    (void)encoder_jpeg_init((uint32_t)hVencJpegPluginInstance.cfgJpeg.frameType,
                            width, height, pOutput, szOut);
    return;
  }

  s_lb_w = width; s_lb_h = height; s_lb_szOut = szOut;
  lb_inited = 1;
}

/* Erratum ES0620, VENC and DCMIPP desynchronisation. The caller stops the
 * DCMIPP first, then re-arms the VENC and restarts the DCMIPP afterwards. */
void encoder_jpeg_venc_errata_reset(void)
{
  if (g_venc_band == NULL || !lb_inited)
    return;

  /* Lets a trailing AXI burst drain, so the reset does not land mid-access. */
  vTaskDelay(pdMS_TO_TICKS(2));

  __HAL_RCC_VENC_FORCE_RESET();   __DSB(); __ISB();
  __HAL_RCC_VENC_RELEASE_RESET(); __DSB(); __ISB();
  LL_VENC_Init();

  /* The reset cleared every register, so the instance is re-initialised. */
  JpegEncRelease(lbInstance);   /* by value */
  lbInstance = NULL;
  lb_inited  = 0;
  encoder_jpeg_linebuf_ensure_init(s_lb_w, s_lb_h, s_lb_szOut);
}

/* Per-frame hook, empty. */
void encoder_jpeg_linebuf_begin_frame(void)
{
}

/* Called inside JpegEncEncode, in the capture task. Grants the encoder the
 * MB-lines the DCMIPP has produced, waiting up to 80 ms for more. */
static void venc_linebuf_cbFunc(void *pAppData) {
  (void)pAppData;
  g_dbg_lb_cb_calls++;
  uint32_t rd = JpegEncGetEncodedMbLines(lbInstance);          /* MB-lines consumed */
  uint32_t total = g_venc_total_mblines;
  uint32_t tickstart = HAL_GetTick();
  uint32_t target;
  for (;;) {
    uint32_t prod = g_dcmipp_wr_mblines;                       /* written by the LINE IRQ */
    target = (prod < total) ? prod : total;
    if (target > rd) break;
    if (rd >= total) { target = total; break; }
    if ((HAL_GetTick() - tickstart) > 80u) { target = (rd < total ? rd+1u : total); break; } /* stall guard */
    vTaskDelay(1);
  }
  (void)JpegEncSetInputMBLines(lbInstance, target);
}

/* Encodes one frame, blocking for about one frame time. Returns the JFIF size,
 * or 0 on failure. */
uint32_t encode_jpeg_frame_linebuf(void) {
  if (!lb_inited) return 0u;
  JpegEncIn in; memset(&in,0,sizeof(in));
  in.pOutBuf=(u8*)pOutput; in.busOutBuf=(uint32_t)pOutput; in.outBufSize=szOutput;
  in.frameHeader=1; in.busLum=(uint32_t)g_venc_band; in.busCb=0; in.busCr=0;
  /* Must be 0: the DCMIPP starts after the VENC is armed, and the venc_rdy line
   * events then advance the count. RM0486 section 45.4.7. */
  in.lineBufWrCnt = 0u;
  JpegEncOut out; memset(&out,0,sizeof(out));
  int rv = JpegEncEncode(lbInstance, &in, &out, venc_linebuf_cbFunc, NULL);
  g_dbg_lb_enc_rv = (int32_t)rv;
  g_dbg_lb_instant = LL_VENC_ReadRegister(497u);   /* 0x7C4>>2; bit27 is line-buffer mode */
  /* Register snapshot on the 'e' command and on the first rejected encode. */
  {
    static uint8_t s_lb_cap = 0u;
    uint8_t failed = (rv != JPEGENC_FRAME_READY && rv != JPEGENC_RESTART_INTERVAL);
    if (g_dbg_venc_arm || (failed && !s_lb_cap)) {
      if (failed) s_lb_cap = 1u;
      g_dbg_venc_arm = 0u;
      for (uint32_t i = 0; i < 64u; i++)  g_dbg_swreg_lo[i] = LL_VENC_ReadRegister(i);
      for (uint32_t i = 0; i < 16u; i++)  g_dbg_swreg_hi[i] = LL_VENC_ReadRegister(290u + i);
      g_dbg_venc_print = 1u;
    }
  }
  return (rv==JPEGENC_FRAME_READY || rv==JPEGENC_RESTART_INTERVAL) ? out.jfifSize : 0u;
}
