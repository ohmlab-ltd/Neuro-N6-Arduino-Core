/**
 ******************************************************************************
 * @file    neuron6_speaker.c
 * @brief   Speaker output: SAI1 Block B as I2S master TX into a MAX98357A amp.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * Portions Copyright (c) 2023 STMicroelectronics: the GPDMA set-up, from
 * the audio driver of ST's STM32N6570-DK BSP. See ../LICENSE.txt.
 * SPDX-License-Identifier: MIT AND BSD-3-Clause
 ******************************************************************************
 */
#include "stm32n6xx_hal.h"
#include "NeuroN6_FrameQueue.h"
#include "Speaker.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* CDC console writer. */
extern void NeuroN6_DbgPrint(const char *s);

#define SPK_FRAMES_HALF     512u    /* stereo frames per DMA half */
#define SPK_CH              2u       /* I2S slots, L and R; mono is duplicated */
#define SPK_TX_HALFWORDS    (SPK_FRAMES_HALF * SPK_CH * 2u)   /* int16 in the whole buffer */
#define SPK_RING_SIZE       16384u  /* int16 mono samples; a power of two */
#define SPK_LUT_SIZE        256u    /* sine table entries, one period */

/* SAI1 kernel clock: PLL4 through IC7 / 16. fs = kernel / (MCKDIV * 32),
 * MCKDIV 1 to 63. */
#define SPK_IC7_DIV         16u
#define SPK_KER_HZ          25000000u                     /* 400 MHz / 16 */
#define SPK_FRAME_BITS      32u                           /* 2 slots * 16 bit */
#define SPK_DEFAULT_FS      48000u

#define SPK_AF_SAI1         GPIO_AF6_SAI1                  /* PG1, PG2, PE3 */
#define SPK_TONE_INF        0xFFFFFFFFu                    /* tone() with no duration */

/* MAX98357A SD_MODE on PB6: low is shutdown, high selects the left channel. */
#define SPK_SD_PORT         GPIOB
#define SPK_SD_PIN          GPIO_PIN_6
#define SPK_SD_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

SAI_HandleTypeDef        hSpkSai;
DMA_HandleTypeDef        hDmaSai;

/* GPDMA list node. Non-cacheable: the DMA fetches it from memory. */
__attribute__((section(".axisram_nc"), aligned(32), used))
static DMA_NodeTypeDef   TxNode;

/* DMA source, 32-aligned for the D-cache clean. Interleaved L, R. */
__attribute__((aligned(32)))
static int16_t  spk_tx_buf[SPK_TX_HALFWORDS];

static int16_t  spk_mono_tmp[SPK_FRAMES_HALF];

/* PCM ring in PSRAM: one producer, write(), and one consumer, the DMA ISR. */
__attribute__((section(".ext_psram")))
static int16_t  spk_ring[SPK_RING_SIZE];
static volatile uint32_t spk_ring_wr = 0;
static volatile uint32_t spk_ring_rd = 0;
static volatile uint32_t spk_ring_underrun = 0;

static int16_t  spk_sine_lut[SPK_LUT_SIZE];

/* Tone generator, shared by the ISR and tone()/noTone(). */
static volatile uint32_t s_tone_phase    = 0;
static volatile uint32_t s_tone_inc      = 0;
static volatile uint32_t s_tone_frames   = 0;     /* frames left, or SPK_TONE_INF */
static volatile bool     s_tone_active   = false;

static volatile int32_t  s_vol_q15       = 32768; /* 0..32768 */

static volatile bool s_ready    = false;
static volatile bool s_init_err = false;
static uint32_t      s_fs       = SPK_DEFAULT_FS;
static uint32_t      s_mckdiv   = 16u;
static uint32_t      s_ker_hz   = SPK_KER_HZ;   /* SAI1 kernel clock, read back at begin() */

/* HAL return codes for the [SPKDBG] dump. -999 means the step never ran. */
static int32_t s_dbg_clkcfg_ret = -999;
static int32_t s_dbg_sai_ret    = -999;
static int32_t s_dbg_tx_ret     = -999;
static volatile uint32_t s_dbg_err_cb = 0;

static inline uint32_t ring_avail(void) { return spk_ring_wr - spk_ring_rd; }
static inline uint32_t ring_free(void)  { return SPK_RING_SIZE - (spk_ring_wr - spk_ring_rd); }

static uint32_t ring_push(const int16_t *src, uint32_t n)
{
  uint32_t wr = spk_ring_wr;
  const uint32_t rd = spk_ring_rd;
  uint32_t pushed = 0;
  for (uint32_t i = 0u; i < n; i++) {
    if ((wr - rd) >= SPK_RING_SIZE) break;
    spk_ring[wr & (SPK_RING_SIZE - 1u)] = src[i];
    wr++; pushed++;
  }
  spk_ring_wr = wr;
  return pushed;
}

static uint32_t ring_pop(int16_t *dst, uint32_t n)
{
  uint32_t avail = ring_avail();
  if (n > avail) n = avail;
  uint32_t rd = spk_ring_rd;
  for (uint32_t i = 0u; i < n; i++) dst[i] = spk_ring[(rd + i) & (SPK_RING_SIZE - 1u)];
  spk_ring_rd = rd + n;
  return n;
}

/* Fills one DMA half. Runs in the DMA interrupt. */
static void spk_fill(int16_t *dst, uint32_t frames)
{
  const int32_t vol = s_vol_q15;

  if (s_tone_active) {
    for (uint32_t i = 0u; i < frames; i++) {
      int16_t s = 0;
      if (s_tone_active) {
        s_tone_phase += s_tone_inc;
        s = (int16_t)(((int32_t)spk_sine_lut[s_tone_phase >> 24] * vol) >> 15);
        if (s_tone_frames != SPK_TONE_INF) {
          uint32_t left = s_tone_frames - 1u;
          s_tone_frames = left;
          if (left == 0u) s_tone_active = false;
        }
      }
      dst[2u * i] = s;
      dst[2u * i + 1u] = s;
    }
  } else {
    uint32_t got = ring_pop(spk_mono_tmp, frames);
    if (got < frames) spk_ring_underrun++;
    for (uint32_t i = 0u; i < got; i++) {
      int16_t s = (int16_t)(((int32_t)spk_mono_tmp[i] * vol) >> 15);
      dst[2u * i] = s;
      dst[2u * i + 1u] = s;
    }
    for (uint32_t i = got; i < frames; i++) { dst[2u * i] = 0; dst[2u * i + 1u] = 0; }
  }

  SCB_CleanDCache_by_Addr((void *)dst, (int32_t)(frames * SPK_CH * sizeof(int16_t)));
}

/* PLL4 is left alone: it also clocks XSPI2, the PSRAM interface. */
static void spk_clock_config(void)
{
  RCC_PeriphCLKInitTypeDef p = {0};
  p.PeriphClockSelection                = RCC_PERIPHCLK_SAI1;
  p.Sai1ClockSelection                  = RCC_SAI1CLKSOURCE_IC7;
  p.ICSelection[RCC_IC7].ClockSelection = RCC_ICCLKSOURCE_PLL4;
  p.ICSelection[RCC_IC7].ClockDivider   = SPK_IC7_DIV;
  s_dbg_clkcfg_ret = (int32_t)HAL_RCCEx_PeriphCLKConfig(&p);
  if (s_dbg_clkcfg_ret != HAL_OK) {
    s_init_err = true;
    return;
  }

  /* Every rate is computed from the kernel clock read back here. */
  const uint32_t ker = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SAI1);
  s_ker_hz = (ker != 0u) ? ker : SPK_KER_HZ;
}

static void spk_sai_init(void)
{
  hSpkSai.Instance            = SAI1_Block_B;
  hSpkSai.Init.AudioMode      = SAI_MODEMASTER_TX;
  hSpkSai.Init.Synchro        = SAI_ASYNCHRONOUS;
  hSpkSai.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
  hSpkSai.Init.NoDivider      = SAI_MASTERDIVIDER_DISABLE;   /* NODIV=1: SCK = kernel / MCKDIV */
  hSpkSai.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
  hSpkSai.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;  /* uses the Mckdiv set below */
  hSpkSai.Init.Mckdiv         = s_mckdiv;
  hSpkSai.Init.MckOutput      = SAI_MCK_OUTPUT_DISABLE;      /* MAX98357A needs no MCLK */
  hSpkSai.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hSpkSai.Init.MonoStereoMode = SAI_STEREOMODE;
  hSpkSai.Init.CompandingMode = SAI_NOCOMPANDING;
  hSpkSai.Init.TriState       = SAI_OUTPUT_NOTRELEASED;

  s_dbg_sai_ret = (int32_t)HAL_SAI_InitProtocol(&hSpkSai, SAI_I2S_STANDARD,
                                                SAI_PROTOCOL_DATASIZE_16BIT, 2u);
  if (s_dbg_sai_ret != HAL_OK) s_init_err = true;
}

/* Called from HAL_SAI_Init. */
void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance != SAI1_Block_B) return;

  GPIO_InitTypeDef           GPIO_InitStruct = {0};
  static DMA_NodeConfTypeDef dmaNodeConfig;
  static DMA_QListTypeDef    SaiTxQueue;

  __HAL_RCC_SAI1_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* PG1: SAI1_SCK_B, BCLK. PG2: SAI1_FS_B, LRCLK. */
  GPIO_InitStruct.Pin       = GPIO_PIN_1 | GPIO_PIN_2;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = SPK_AF_SAI1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* PE3: SAI1_SD_B, data. */
  GPIO_InitStruct.Pin       = GPIO_PIN_3;
  GPIO_InitStruct.Alternate = SPK_AF_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  if (SaiTxQueue.Head == NULL) {
    __HAL_RCC_GPDMA1_CLK_ENABLE();

    /* GPDMA1 channels: 0 Microphone, 1 NeuroVisionThermal, 2 Speaker. */
    hDmaSai.Instance = GPDMA1_Channel2;

    dmaNodeConfig.NodeType                         = DMA_GPDMA_LINEAR_NODE;
    dmaNodeConfig.Init.Request                     = GPDMA1_REQUEST_SAI1_B;
    dmaNodeConfig.Init.BlkHWRequest                = DMA_BREQ_SINGLE_BURST;
    dmaNodeConfig.Init.Direction                   = DMA_MEMORY_TO_PERIPH;
    dmaNodeConfig.Init.SrcInc                      = DMA_SINC_INCREMENTED;
    dmaNodeConfig.Init.DestInc                     = DMA_DINC_FIXED;
    dmaNodeConfig.Init.SrcDataWidth                = DMA_SRC_DATAWIDTH_HALFWORD;
    dmaNodeConfig.Init.DestDataWidth               = DMA_DEST_DATAWIDTH_HALFWORD;
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

    if (HAL_DMAEx_List_BuildNode(&dmaNodeConfig, &TxNode) != HAL_OK)        { s_init_err = true; return; }
    if (HAL_DMAEx_List_InsertNode_Tail(&SaiTxQueue, &TxNode) != HAL_OK)     { s_init_err = true; return; }
    if (HAL_DMAEx_List_SetCircularMode(&SaiTxQueue) != HAL_OK)              { s_init_err = true; return; }

    hDmaSai.InitLinkedList.Priority          = DMA_HIGH_PRIORITY;
    hDmaSai.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
    hDmaSai.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
    hDmaSai.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
    hDmaSai.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;

    if (HAL_DMAEx_List_Init(&hDmaSai) != HAL_OK)                            { s_init_err = true; return; }

    __HAL_RCC_RIFSC_CLK_ENABLE();
    if (HAL_DMA_ConfigChannelAttributes(&hDmaSai,
        DMA_CHANNEL_SEC | DMA_CHANNEL_PRIV | DMA_CHANNEL_SRC_SEC | DMA_CHANNEL_DEST_SEC) != HAL_OK) {
      s_init_err = true; return;
    }

    /* SAI1 must be secure, or RIF blocks the secure GPDMA channel's writes. */
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_SAI1,
                                          RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);

    if (HAL_DMAEx_List_LinkQ(&hDmaSai, &SaiTxQueue) != HAL_OK)              { s_init_err = true; return; }

    __HAL_LINKDMA(hsai, hdmatx, hDmaSai);

    HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);
  }
}

void GPDMA1_Channel2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hDmaSai);
}

static void spk_amp_enable(bool on);

/* Interrupt context. Each refills the half last played. */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) { (void)hsai; spk_fill(&spk_tx_buf[0],                     SPK_FRAMES_HALF); }
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)     { (void)hsai; spk_fill(&spk_tx_buf[SPK_FRAMES_HALF * SPK_CH], SPK_FRAMES_HALF); }

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
  s_dbg_err_cb++;
  /* A DMA error stops the transfer and the SAI and leaves the handle READY.
   * The amp goes off as n6_spk_end() turns it off, and begin() starts again. */
  if ((hsai->Instance == SAI1_Block_B) && (hsai->State == HAL_SAI_STATE_READY)) {
    spk_amp_enable(false);
    s_tone_active = false;
    s_ready = false;
  }
}

static void spk_amp_enable(bool on)
{
  static bool configured = false;
  if (!configured) {
    GPIO_InitTypeDef g = {0};
    SPK_SD_CLK_ENABLE();
    g.Pin   = SPK_SD_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SPK_SD_PORT, &g);
    configured = true;
  }
  HAL_GPIO_WritePin(SPK_SD_PORT, SPK_SD_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint32_t spk_pick_mckdiv(uint32_t want_fs)
{
  if (want_fs == 0u) want_fs = SPK_DEFAULT_FS;
  uint32_t denom = want_fs * SPK_FRAME_BITS;
  uint32_t d = (s_ker_hz + denom / 2u) / denom;          /* nearest divider */
  if (d < 1u)  d = 1u;
  if (d > 63u) d = 63u;
  return d;
}

int n6_spk_begin(uint32_t want_fs)
{
  if (s_ready) return 0;

  s_init_err = false;
  spk_ring_wr = 0; spk_ring_rd = 0; spk_ring_underrun = 0;
  s_tone_active = false; s_tone_phase = 0; s_tone_inc = 0; s_tone_frames = 0;

  for (uint32_t i = 0u; i < SPK_LUT_SIZE; i++) {
    double a = (2.0 * 3.14159265358979323846 * (double)i) / (double)SPK_LUT_SIZE;
    spk_sine_lut[i] = (int16_t)lround(32767.0 * sin(a));
  }

  memset(spk_tx_buf, 0, sizeof(spk_tx_buf));
  SCB_CleanDCache_by_Addr((void *)spk_tx_buf, (int32_t)sizeof(spk_tx_buf));

  /* The CDC console starts first, so [SPKDBG] shows when the bring-up fails. */
  NeuroN6_StreamUsbOnlyStart();

  /* The amp stays in shutdown until the stream runs, so it does not pop. */
  spk_amp_enable(false);

  spk_clock_config();
  if (s_init_err) return -1;

  s_mckdiv = spk_pick_mckdiv(want_fs);
  s_fs     = s_ker_hz / (s_mckdiv * SPK_FRAME_BITS);

  spk_sai_init();
  if (s_init_err) return -1;

  /* A DMA set-up that failed in an earlier begin() left the SAI with no DMA
   * handle, and HAL_SAI_Init does not run HAL_SAI_MspInit again. */
  if (hSpkSai.hdmatx == NULL) return -1;

  /* The size counts int16 elements. */
  s_dbg_tx_ret = (int32_t)HAL_SAI_Transmit_DMA(&hSpkSai, (uint8_t *)spk_tx_buf, (uint16_t)SPK_TX_HALFWORDS);
  if (s_dbg_tx_ret != HAL_OK) return -1;

  spk_amp_enable(true);

  s_ready = true;
  return 0;
}

bool     n6_spk_ready(void)        { return s_ready; }
uint32_t n6_spk_sample_rate(void)  { return s_fs; }

void n6_spk_tone(uint32_t freq, uint32_t ms)
{
  if (!s_ready || freq == 0u) { s_tone_active = false; return; }
  uint32_t inc    = (uint32_t)(((uint64_t)freq << 32) / s_fs);
  uint32_t frames = (ms == 0u) ? SPK_TONE_INF
                               : (uint32_t)(((uint64_t)ms * s_fs) / 1000u);
  if (frames == 0u) frames = 1u;
  s_tone_inc    = inc;
  s_tone_frames = frames;
  s_tone_phase  = 0;
  s_tone_active = true;
}

void n6_spk_no_tone(void) { s_tone_active = false; }

uint32_t n6_spk_write(const int16_t *samples, uint32_t n)
{
  if (!s_ready || samples == NULL) return 0;
  return ring_push(samples, n);
}

uint32_t n6_spk_available_for_write(void) { return s_ready ? ring_free() : 0u; }
bool     n6_spk_playing(void) { return s_ready && (s_tone_active || ring_avail() > 0u); }

void n6_spk_set_volume(uint16_t vol_q15)
{
  if (vol_q15 > 32768u) vol_q15 = 32768u;
  s_vol_q15 = (int32_t)vol_q15;
}

void n6_spk_end(void)
{
  if (!s_ready) return;
  s_tone_active = false;
  spk_amp_enable(false);       /* first, to avoid a pop */
  HAL_SAI_DMAStop(&hSpkSai);
  __HAL_SAI_DISABLE(&hSpkSai);
  s_ready = false;
}

/* Prints [SPKDBG] to the CDC console. */
void n6_spk_debug_dump(void)
{
  char b[256];

  snprintf(b, sizeof b,
    "[SPKDBG] ready=%u err=%u fs=%lu mckdiv=%lu ker=%lu | clkcfg=%ld saiinit=%ld tx=%ld errcb=%lu | PLL4CFGR1=%08lX IC7CFGR=%08lX\r\n",
    (unsigned)s_ready, (unsigned)s_init_err, (unsigned long)s_fs, (unsigned long)s_mckdiv,
    (unsigned long)s_ker_hz,
    (long)s_dbg_clkcfg_ret, (long)s_dbg_sai_ret, (long)s_dbg_tx_ret, (unsigned long)s_dbg_err_cb,
    (unsigned long)RCC->PLL4CFGR1, (unsigned long)RCC->IC7CFGR);
  NeuroN6_DbgPrint(b);

  if (!s_ready) {
    NeuroN6_DbgPrint("[SPKDBG] not ready - SAI/DMA/GPIO regs skipped (see clkcfg/saiinit/tx)\r\n");
    return;
  }

  snprintf(b, sizeof b,
    "[SPKDBG] SAI CR1=%08lX CR2=%08lX FRCR=%08lX SLOTR=%08lX SR=%08lX IMR=%08lX\r\n",
    (unsigned long)SAI1_Block_B->CR1, (unsigned long)SAI1_Block_B->CR2,
    (unsigned long)SAI1_Block_B->FRCR, (unsigned long)SAI1_Block_B->SLOTR,
    (unsigned long)SAI1_Block_B->SR,  (unsigned long)SAI1_Block_B->IMR);
  NeuroN6_DbgPrint(b);

  snprintf(b, sizeof b,
    "[SPKDBG] DMA(ch2) CCR=%08lX CSR=%08lX CBR1=%08lX CSAR=%08lX CDAR=%08lX CLLR=%08lX\r\n",
    (unsigned long)GPDMA1_Channel2->CCR,  (unsigned long)GPDMA1_Channel2->CSR,
    (unsigned long)GPDMA1_Channel2->CBR1, (unsigned long)GPDMA1_Channel2->CSAR,
    (unsigned long)GPDMA1_Channel2->CDAR, (unsigned long)GPDMA1_Channel2->CLLR);
  NeuroN6_DbgPrint(b);

  snprintf(b, sizeof b,
    "[SPKDBG] ring wr=%lu rd=%lu avail=%lu underrun=%lu vol=%ld tone=%u\r\n",
    (unsigned long)spk_ring_wr, (unsigned long)spk_ring_rd, (unsigned long)ring_avail(),
    (unsigned long)spk_ring_underrun, (long)s_vol_q15, (unsigned)s_tone_active);
  NeuroN6_DbgPrint(b);

  snprintf(b, sizeof b,
    "[SPKDBG] GPIOG MODER=%08lX AFRL=%08lX | GPIOE MODER=%08lX AFRL=%08lX | SD(PB6) ODR=%u (1=amp on)\r\n",
    (unsigned long)GPIOG->MODER, (unsigned long)GPIOG->AFR[0],
    (unsigned long)GPIOE->MODER, (unsigned long)GPIOE->AFR[0],
    (unsigned)((GPIOB->ODR >> 6) & 1u));
  NeuroN6_DbgPrint(b);
}
