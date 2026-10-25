/**
 ******************************************************************************
 * @file    neuron6_audio.c
 * @brief   PDM microphone capture on MDF1_Filter1, streamed as PCM TLVs.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * Portions Copyright (c) 2023 STMicroelectronics: the GPDMA set-up, from
 * the audio driver of ST's STM32N6570-DK BSP. See ../LICENSE.txt.
 * SPDX-License-Identifier: MIT AND BSD-3-Clause
 ******************************************************************************
 */
#include "stm32n6xx_hal.h"
#include "NeuroN6_FrameQueue.h"
#include "Microphone.h"
#include <string.h>
#include <stdio.h>

/* CDC console writer. */
extern void NeuroN6_DbgPrint(const char *s);

#define MDF_BUFFER_SIZE       320u   /* int32 samples, two DMA halves of 160 */
#define PCM_RING_SIZE         16384u /* int16 samples, about 0.5 s; a power of two */
#define AUDIO_CHUNK_SAMPLES   320u   /* samples per TLV */
#define N6_META_AUDIO_PCM     0x50u  /* META_AUDIO_PCM in vnd_meta.h */
/* Follows from the dividers in mdf_init(). */
#define AUDIO_SAMPLE_RATE_HZ  32552u
/* MDF output to int16. Scales with decimation^4, the SINC4 gain. */
#define MDF_TO_PCM_DIVISOR    648

MDF_HandleTypeDef        haudio_in_mdf;
DMA_HandleTypeDef        hDmaMdf;
static MDF_FilterConfigTypeDef s_filt;

/* GPDMA list node. Non-cacheable: the DMA fetches it from memory. */
__attribute__((section(".axisram_nc"), aligned(32), used))
static DMA_NodeTypeDef   RxNode;

/* DMA target, 32-aligned for D-cache invalidation. */
__attribute__((aligned(32)))
static int32_t  mdf_rec_buf[MDF_BUFFER_SIZE];

static int16_t  pcm_buf[MDF_BUFFER_SIZE / 2];

/* PCM ring in PSRAM: one producer, the ISR, and one consumer task. */
__attribute__((section(".ext_psram")))
static int16_t  pcm_ring[PCM_RING_SIZE];
static volatile uint32_t pcm_ring_wr = 0;   /* free-running; written by ISR  */
static volatile uint32_t pcm_ring_rd = 0;   /* free-running; written by task */
static volatile uint32_t pcm_ring_overflow = 0;

static volatile bool s_audio_ready  = false;
static volatile bool s_audio_stream = false;
static volatile bool s_audio_init_err = false;
static uint32_t      s_audio_sr     = AUDIO_SAMPLE_RATE_HZ;
static volatile uint32_t s_audio_peak = 0;   /* largest |sample| since the last n6_audio_peak() */

/* HAL return codes for the [MICDBG] dump. -999 means the step never ran. */
static int32_t s_dbg_clkcfg_ret   = -999;
static int32_t s_dbg_mdf_init_ret = -999;
static int32_t s_dbg_acq_ret      = -999;

static inline uint32_t ring_avail(void)
{
  return pcm_ring_wr - pcm_ring_rd;
}

static void ring_push(const int16_t *src, uint32_t n)
{
  uint32_t wr = pcm_ring_wr;
  const uint32_t rd = pcm_ring_rd;
  for (uint32_t i = 0u; i < n; i++) {
    if ((wr - rd) >= PCM_RING_SIZE) { pcm_ring_overflow++; break; }
    pcm_ring[wr & (PCM_RING_SIZE - 1u)] = src[i];
    wr++;
  }
  pcm_ring_wr = wr;
}

static uint32_t ring_pop(void *dst, uint32_t nsamples)
{
  uint32_t avail = ring_avail();
  if (nsamples > avail) nsamples = avail;
  const uint32_t rd = pcm_ring_rd & (PCM_RING_SIZE - 1u);
  uint32_t first = PCM_RING_SIZE - rd;
  if (first > nsamples) first = nsamples;
  memcpy(dst, &pcm_ring[rd], (size_t)first * 2u);
  if (nsamples > first) {
    memcpy((uint8_t *)dst + (size_t)first * 2u, &pcm_ring[0], (size_t)(nsamples - first) * 2u);
  }
  pcm_ring_rd += nsamples;
  return nsamples;
}

/* MDF1 kernel clock: IC8 from PLL4. PLL4 is left alone: it also clocks XSPI2,
 * the PSRAM interface. */
static void mdf_clock_config(void)
{
  RCC_PeriphCLKInitTypeDef p = {0};
  p.PeriphClockSelection                 = RCC_PERIPHCLK_MDF1;
  p.Mdf1ClockSelection                   = RCC_MDF1CLKSOURCE_IC8;
  p.ICSelection[RCC_IC8].ClockSelection  = RCC_ICCLKSOURCE_PLL4;
  p.ICSelection[RCC_IC8].ClockDivider    = 4;   /* 400 MHz / 4 = 100 MHz */
  s_dbg_clkcfg_ret = (int32_t)HAL_RCCEx_PeriphCLKConfig(&p);
  if (s_dbg_clkcfg_ret != HAL_OK) {
    s_audio_init_err = true;
  }
}

/* The MP34DT05-A accepts a clock between 1.2 and 3.25 MHz. */
static void mdf_init(void)
{
  haudio_in_mdf.Instance = MDF1_Filter1;

  haudio_in_mdf.Init.CommonParam.ProcClockDivider              = 2;    /* 50 MHz */
  haudio_in_mdf.Init.CommonParam.OutputClock.Activation        = ENABLE;
  haudio_in_mdf.Init.CommonParam.OutputClock.Pins              = MDF_OUTPUT_CLOCK_0;
  haudio_in_mdf.Init.CommonParam.OutputClock.Divider           = 16;   /* CCK 3.125 MHz */
  haudio_in_mdf.Init.CommonParam.OutputClock.Trigger.Activation = DISABLE;
  haudio_in_mdf.Init.SerialInterface.Activation                = ENABLE;
  haudio_in_mdf.Init.SerialInterface.Mode                      = MDF_SITF_NORMAL_SPI_MODE;
  haudio_in_mdf.Init.SerialInterface.ClockSource               = MDF_SITF_CCK0_SOURCE;
  haudio_in_mdf.Init.SerialInterface.Threshold                 = 31U;
  haudio_in_mdf.Init.FilterBistream                            = MDF_BITSTREAM1_FALLING; /* PB2 = SDI1 */

  s_dbg_mdf_init_ret = (int32_t)HAL_MDF_Init(&haudio_in_mdf);
  if (s_dbg_mdf_init_ret != HAL_OK) {
    s_audio_init_err = true;
    return;
  }

  s_filt.DataSource                     = MDF_DATA_SOURCE_BSMX;
  s_filt.Delay                          = 0U;
  s_filt.CicMode                        = MDF_ONE_FILTER_SINC4;
  s_filt.DecimationRatio                = 24;   /* with the reshape /4, 32552 Hz */
  s_filt.Offset                         = 0;
  s_filt.Gain                           = 1;
  s_filt.ReshapeFilter.Activation       = ENABLE;
  s_filt.ReshapeFilter.DecimationRatio  = MDF_RSF_DECIMATION_RATIO_4;
  s_filt.HighPassFilter.Activation      = ENABLE;
  s_filt.HighPassFilter.CutOffFrequency = MDF_HPF_CUTOFF_0_000625FPCM;
  s_filt.Integrator.Activation          = DISABLE;
  s_filt.AcquisitionMode                = MDF_MODE_ASYNC_CONT;
  s_filt.FifoThreshold                  = MDF_FIFO_THRESHOLD_NOT_EMPTY;
  s_filt.DiscardSamples                 = 0U;
}

/* Called from HAL_MDF_Init. */
void HAL_MDF_MspInit(MDF_HandleTypeDef *hmdf)
{
  GPIO_InitTypeDef           GPIO_InitStruct;
  static DMA_NodeConfTypeDef dmaNodeConfig;
  static DMA_QListTypeDef    MdfRxQueue;

  __HAL_RCC_MDF1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* PB2: MDF1_SDI1, PDM data. */
  GPIO_InitStruct.Pin       = GPIO_PIN_2;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_MDF1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PE2: MDF1_CCK0, PDM clock out. */
  GPIO_InitStruct.Pin       = GPIO_PIN_2;
  GPIO_InitStruct.Alternate = GPIO_AF4_MDF1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  if (MdfRxQueue.Head == NULL) {
    __HAL_RCC_GPDMA1_CLK_ENABLE();

    hDmaMdf.Instance = GPDMA1_Channel0;

    dmaNodeConfig.NodeType                         = DMA_GPDMA_LINEAR_NODE;
    dmaNodeConfig.Init.Request                     = GPDMA1_REQUEST_MDF1_FLT1;
    dmaNodeConfig.Init.BlkHWRequest                = DMA_BREQ_SINGLE_BURST;
    dmaNodeConfig.Init.Direction                   = DMA_PERIPH_TO_MEMORY;
    dmaNodeConfig.Init.SrcInc                      = DMA_SINC_FIXED;
    dmaNodeConfig.Init.DestInc                     = DMA_DINC_INCREMENTED;
    dmaNodeConfig.Init.SrcDataWidth                = DMA_SRC_DATAWIDTH_WORD;
    dmaNodeConfig.Init.DestDataWidth               = DMA_DEST_DATAWIDTH_WORD;
    dmaNodeConfig.Init.SrcBurstLength              = 1;
    dmaNodeConfig.Init.DestBurstLength             = 1;
    dmaNodeConfig.Init.Priority                    = DMA_HIGH_PRIORITY;
    dmaNodeConfig.Init.TransferEventMode           = DMA_TCEM_BLOCK_TRANSFER;
    dmaNodeConfig.Init.TransferAllocatedPort       = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
    dmaNodeConfig.DataHandlingConfig.DataExchange  = DMA_EXCHANGE_NONE;
    dmaNodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
    dmaNodeConfig.TriggerConfig.TriggerPolarity    = DMA_TRIG_POLARITY_MASKED;
    dmaNodeConfig.SrcSecure                        = DMA_CHANNEL_SRC_SEC;
    dmaNodeConfig.DestSecure                       = DMA_CHANNEL_DEST_SEC;

    if (HAL_DMAEx_List_BuildNode(&dmaNodeConfig, &RxNode) != HAL_OK)        { s_audio_init_err = true; return; }
    if (HAL_DMAEx_List_InsertNode_Tail(&MdfRxQueue, &RxNode) != HAL_OK)     { s_audio_init_err = true; return; }
    if (HAL_DMAEx_List_SetCircularMode(&MdfRxQueue) != HAL_OK)              { s_audio_init_err = true; return; }

    hDmaMdf.InitLinkedList.Priority          = DMA_HIGH_PRIORITY;
    hDmaMdf.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
    hDmaMdf.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
    hDmaMdf.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
    hDmaMdf.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;

    if (HAL_DMAEx_List_Init(&hDmaMdf) != HAL_OK)                            { s_audio_init_err = true; return; }

    __HAL_RCC_RIFSC_CLK_ENABLE();
    if (HAL_DMA_ConfigChannelAttributes(&hDmaMdf,
        DMA_CHANNEL_SEC | DMA_CHANNEL_PRIV | DMA_CHANNEL_SRC_SEC | DMA_CHANNEL_DEST_SEC) != HAL_OK) {
      s_audio_init_err = true; return;
    }

    /* MDF1 must be secure, or RIF blocks the secure GPDMA channel's reads. */
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_MDF1,
                                          RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);

    if (HAL_DMAEx_List_LinkQ(&hDmaMdf, &MdfRxQueue) != HAL_OK)              { s_audio_init_err = true; return; }

    __HAL_LINKDMA(hmdf, hdma, hDmaMdf);

    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
  }
}

void GPDMA1_Channel0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDmaMdf);
}

/* Runs in the DMA interrupt for each half of mdf_rec_buf. */
static void process_half(uint32_t start)
{
  SCB_InvalidateDCache_by_Addr((void *)&mdf_rec_buf[start], (int32_t)((MDF_BUFFER_SIZE / 2u) * sizeof(int32_t)));

  uint32_t peak = s_audio_peak;
  for (uint32_t i = 0u; i < MDF_BUFFER_SIZE / 2u; i++) {
    int32_t tmp = mdf_rec_buf[start + i] / MDF_TO_PCM_DIVISOR;
    if (tmp >  32767)  tmp =  32767;
    if (tmp < -32768)  tmp = -32768;
    pcm_buf[i] = (int16_t)tmp;
    uint32_t a = (uint32_t)(tmp < 0 ? -tmp : tmp);
    if (a > peak) peak = a;
  }
  s_audio_peak = peak;
  ring_push(pcm_buf, MDF_BUFFER_SIZE / 2u);
}

void HAL_MDF_AcqHalfCpltCallback(MDF_HandleTypeDef *hmdf) { (void)hmdf; process_half(0u); }
void HAL_MDF_AcqCpltCallback(MDF_HandleTypeDef *hmdf)     { (void)hmdf; process_half(MDF_BUFFER_SIZE / 2u); }
void HAL_MDF_ErrorCallback(MDF_HandleTypeDef *hmdf)       { (void)hmdf; }

int n6_audio_begin(void)
{
  if (s_audio_ready) return 0;

  pcm_ring_wr = 0; pcm_ring_rd = 0; pcm_ring_overflow = 0;
  s_audio_init_err = false;
  s_audio_sr = AUDIO_SAMPLE_RATE_HZ;

  /* The USB task starts first, so the debug console can report a failed
   * bring-up. */
  NeuroN6_StreamUsbOnlyStart();

  /* A begin() that failed part way is undone, as the HAL sets a filter up
   * only once, and the MDF starts again from reset: its active bits clear
   * only while its clock runs. */
  __HAL_RCC_MDF1_CLK_ENABLE();
  if (haudio_in_mdf.State != HAL_MDF_STATE_RESET) {
    (void)HAL_MDF_DeInit(&haudio_in_mdf);
  }
  __HAL_RCC_MDF1_FORCE_RESET();
  __HAL_RCC_MDF1_RELEASE_RESET();

  mdf_clock_config();
  mdf_init();
  if (s_audio_init_err) return -1;

  MDF_DmaConfigTypeDef dmaConfig;
  dmaConfig.Address    = (uint32_t)mdf_rec_buf;
  dmaConfig.DataLength = MDF_BUFFER_SIZE * sizeof(int32_t);
  dmaConfig.MsbOnly    = DISABLE;
  s_dbg_acq_ret = (int32_t)HAL_MDF_AcqStart_DMA(&haudio_in_mdf, &s_filt, &dmaConfig);
  if (s_dbg_acq_ret != HAL_OK) return -1;

  s_audio_stream = true;
  s_audio_ready  = true;
  return 0;
}

bool     n6_audio_ready(void)        { return s_audio_ready; }
uint32_t n6_audio_sample_rate(void)  { return s_audio_sr; }
uint32_t n6_audio_available(void)    { return ring_avail(); }
uint32_t n6_audio_read(int16_t *dst, uint32_t max_samples) { return dst ? ring_pop(dst, max_samples) : 0u; }
void     n6_audio_stream_enable(bool on) { s_audio_stream = on; }
bool     n6_audio_streaming(void)    { return s_audio_stream; }
uint32_t n6_audio_peak(void)         { uint32_t p = s_audio_peak; s_audio_peak = 0u; return p; }

/* One TLV. The core calls n6_audio_drain() only while the link is idle. */
static uint8_t s_audio_tlv[3u + 8u + AUDIO_CHUNK_SAMPLES * 2u];

bool n6_audio_drain(const uint8_t **out_tlv, uint32_t *out_len)
{
  if (!s_audio_stream || !s_audio_ready) return false;
  if (ring_avail() < AUDIO_CHUNK_SAMPLES) return false;

  const uint16_t n    = (uint16_t)AUDIO_CHUNK_SAMPLES;
  const uint16_t vlen = (uint16_t)(8u + (uint32_t)n * 2u);   /* 8-byte header + PCM */
  uint8_t *p = s_audio_tlv;

  p[0] = (uint8_t)N6_META_AUDIO_PCM;          /* u8 type */
  p[1] = (uint8_t)(vlen & 0xFFu);             /* u16 length LE */
  p[2] = (uint8_t)(vlen >> 8);
  p[3] = (uint8_t)(s_audio_sr & 0xFFu);       /* u32 sample_rate LE */
  p[4] = (uint8_t)((s_audio_sr >> 8)  & 0xFFu);
  p[5] = (uint8_t)((s_audio_sr >> 16) & 0xFFu);
  p[6] = (uint8_t)((s_audio_sr >> 24) & 0xFFu);
  p[7] = 1u;                                   /* u8 channels */
  p[8] = 16u;                                  /* u8 bits per sample */
  p[9]  = (uint8_t)(n & 0xFFu);                /* u16 sample_count LE */
  p[10] = (uint8_t)(n >> 8);
  ring_pop(&p[11], n);                         /* s16 PCM LE */

  *out_tlv = s_audio_tlv;
  *out_len = (uint32_t)(3u + vlen);
  return true;
}

/* Prints [MICDBG] to the CDC console. n6_usb_task calls it at about 1 Hz when
 * the debug console is on. */
void n6_audio_debug_dump(void)
{
  char b[256];

  snprintf(b, sizeof b,
    "[MICDBG] ready=%u err=%u stream=%u sr=%lu | clkcfg=%ld mdfinit=%ld acq=%ld | PLL4CFGR1=%08lX IC8CFGR=%08lX\r\n",
    (unsigned)s_audio_ready, (unsigned)s_audio_init_err, (unsigned)s_audio_stream,
    (unsigned long)s_audio_sr,
    (long)s_dbg_clkcfg_ret, (long)s_dbg_mdf_init_ret, (long)s_dbg_acq_ret,
    (unsigned long)RCC->PLL4CFGR1, (unsigned long)RCC->IC8CFGR);
  NeuroN6_DbgPrint(b);

  /* The registers below are clocked by MspInit. An unclocked read faults. */
  if (!s_audio_ready) {
    NeuroN6_DbgPrint("[MICDBG] not ready - MDF/DMA/GPIO regs skipped (see clkcfg/mdfinit/acq)\r\n");
    return;
  }

  SCB_InvalidateDCache_by_Addr((void *)mdf_rec_buf, (int32_t)sizeof(mdf_rec_buf));

  snprintf(b, sizeof b,
    "[MICDBG] MDF GCR=%08lX CKGCR=%08lX | F0 SITFCR=%08lX DFLTCR=%08lX DFLTISR=%08lX | F1 SITFCR=%08lX BSMXCR=%08lX DFLTCR=%08lX DFLTCICR=%08lX DFLTRSFR=%08lX DFLTISR=%08lX\r\n",
    (unsigned long)MDF1->GCR, (unsigned long)MDF1->CKGCR,
    (unsigned long)MDF1_Filter0->SITFCR, (unsigned long)MDF1_Filter0->DFLTCR, (unsigned long)MDF1_Filter0->DFLTISR,
    (unsigned long)MDF1_Filter1->SITFCR, (unsigned long)MDF1_Filter1->BSMXCR, (unsigned long)MDF1_Filter1->DFLTCR,
    (unsigned long)MDF1_Filter1->DFLTCICR, (unsigned long)MDF1_Filter1->DFLTRSFR, (unsigned long)MDF1_Filter1->DFLTISR);
  NeuroN6_DbgPrint(b);

  snprintf(b, sizeof b,
    "[MICDBG] DMA CCR=%08lX CSR=%08lX CBR1=%08lX CSAR=%08lX CDAR=%08lX CTR1=%08lX CTR2=%08lX CLLR=%08lX CCID=%08lX\r\n",
    (unsigned long)GPDMA1_Channel0->CCR,  (unsigned long)GPDMA1_Channel0->CSR,
    (unsigned long)GPDMA1_Channel0->CBR1, (unsigned long)GPDMA1_Channel0->CSAR,
    (unsigned long)GPDMA1_Channel0->CDAR, (unsigned long)GPDMA1_Channel0->CTR1,
    (unsigned long)GPDMA1_Channel0->CTR2, (unsigned long)GPDMA1_Channel0->CLLR,
    (unsigned long)GPDMA1_Channel0->CCIDCFGR);
  NeuroN6_DbgPrint(b);

  snprintf(b, sizeof b,
    "[MICDBG] GPDMA SEC=%08lX PRIV=%08lX | wr=%lu rd=%lu ovf=%lu peak=%lu | buf[0]=%ld [80]=%ld [160]=%ld [240]=%ld\r\n",
    (unsigned long)GPDMA1->SECCFGR, (unsigned long)GPDMA1->PRIVCFGR,
    (unsigned long)pcm_ring_wr, (unsigned long)pcm_ring_rd, (unsigned long)pcm_ring_overflow,
    (unsigned long)s_audio_peak,
    (long)mdf_rec_buf[0], (long)mdf_rec_buf[80], (long)mdf_rec_buf[160], (long)mdf_rec_buf[240]);
  NeuroN6_DbgPrint(b);

  snprintf(b, sizeof b,
    "[MICDBG] GPIOB MODER=%08lX AFRL=%08lX | GPIOE MODER=%08lX AFRL=%08lX\r\n",
    (unsigned long)GPIOB->MODER, (unsigned long)GPIOB->AFR[0],
    (unsigned long)GPIOE->MODER, (unsigned long)GPIOE->AFR[0]);
  NeuroN6_DbgPrint(b);
}
