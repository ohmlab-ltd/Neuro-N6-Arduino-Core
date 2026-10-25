/**
 ******************************************************************************
 * @file    n6_aed.c
 * @brief   Audio Event Detection: resampler, log-mel frontend, NPU inference.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "n6_aed.h"

#include <math.h>
#include <string.h>
#include <stddef.h>

#include "arm_math.h"
#include "feature_extraction.h"
#include "NeuroN6.h"
#include "NeuroN6_app.h"

static NN_Instance_TypeDef *s_model = NULL;
static bool     s_ok = false;
static float    s_threshold = 0.5f;

/* Resampler step: mic rate over model rate, in Q16. */
static uint32_t s_step_q16 = 0;
static uint32_t s_phase_q16 = 0;
static int16_t  s_prev_in = 0;

/* One analysis window of resampled audio. */
static int16_t  s_win[N6_AED_WIN_LEN];
static uint32_t s_win_fill = 0;

/* The model's input: 96 frames x 64 mel, int8. The newest frame is last. */
static int8_t   s_spec[N6_AED_NFRAMES * N6_AED_NMEL];
static uint32_t s_frames_since_infer = 0;
static bool     s_ready = false;
static bool     s_primed = false;      /* the spectrogram has filled once */

/* Copy of the input. The NPU overwrites s_in_buf during inference. */
static uint8_t  s_img[N6_AED_NFRAMES * N6_AED_NMEL];

static float    s_scores[N6_AED_NUM_CLASSES];
static uint16_t s_level = 0, s_level_run = 0;

static float    s_in_inv_scale = 1.0f;
static int8_t   s_in_offset = 0;
static int8_t  *s_in_buf = NULL;
static uint32_t s_in_bytes = 0;

static arm_rfft_fast_instance_f32 s_rfft;
static float32_t   s_scratch1[N6_AED_NFFT];
static float32_t   s_scratch2[N6_AED_NFFT];
static MelFilterTypeDef       s_mel;
static SpectrogramTypeDef     s_spectro;
static MelSpectrogramTypeDef  s_melspec;
static LogMelSpectrogramTypeDef s_logmel;

static const char *const s_labels[N6_AED_NUM_CLASSES] = {
  "chainsaw", "clock tick", "crackling fire", "crying baby", "dog",
  "helicopter", "rain", "rooster", "sea waves", "sneezing",
};

const char *n6_aed_label(int cls)
{
  if (cls < 0 || cls >= (int)N6_AED_NUM_CLASSES) return "?";
  return s_labels[cls];
}

void  n6_aed_set_threshold(float t) { s_threshold = t; }
const float *n6_aed_scores(void) { return s_scores; }
uint16_t n6_aed_level(void) { return s_level; }
const uint8_t *n6_aed_input_image(void) { return s_img; }

/* Setup */

bool n6_aed_begin(NN_Instance_TypeDef *model, uint32_t mic_sample_rate)
{
  s_ok = false;
  s_model = model;
  if (model == NULL) return false;

  const LL_Buffer_InfoTypeDef *in = LL_ATON_Input_Buffers_Info(model);
  if (in == NULL || in[0].name == NULL) return false;
  uint8_t *start = LL_Buffer_addr_start(&in[0]);
  uint8_t *end   = LL_Buffer_addr_end(&in[0]);
  if (start == NULL || end == NULL || end <= start) return false;

  s_in_buf   = (int8_t *)start;
  s_in_bytes = (uint32_t)(end - start);
  if (s_in_bytes != N6_AED_NFRAMES * N6_AED_NMEL) return false;

  /* The uint8 interface reports the int8 zero point plus 128. The frontend
   * saturates to int8, so it takes the int8 offset. */
  if (in[0].scale == NULL || in[0].offset == NULL) return false;
  const float scale = in[0].scale[0];
  if (!(scale > 0.0f)) return false;

  s_in_inv_scale = 1.0f / scale;
  s_in_offset    = (int8_t)((int32_t)in[0].offset[0] - 128);

  if (mic_sample_rate == 0u) mic_sample_rate = N6_AED_SAMPLE_RATE;
  s_step_q16  = (uint32_t)(((uint64_t)mic_sample_rate << 16) / N6_AED_SAMPLE_RATE);
  s_phase_q16 = 0;
  s_prev_in   = 0;

  if (arm_rfft_fast_init_f32(&s_rfft, N6_AED_NFFT) != ARM_MATH_SUCCESS) return false;

  s_mel.pStartIndices = (uint32_t *)n6_aed_mel_start;
  s_mel.pStopIndices  = (uint32_t *)n6_aed_mel_stop;
  s_mel.pCoefficients = (float32_t *)n6_aed_mel_lut;
  s_mel.NumMels       = N6_AED_NMEL;
  s_mel.FFTLen        = N6_AED_NFFT;
  s_mel.SampRate      = N6_AED_SAMPLE_RATE;
  s_mel.FMin          = 125.0f;
  s_mel.FMax          = 7500.0f;
  s_mel.Formula       = MEL_HTK;
  s_mel.Normalize     = 0;
  s_mel.Mel2F         = 0;

  s_spectro.pRfft     = &s_rfft;
  s_spectro.Type      = SPECTRUM_TYPE_MAGNITUDE;
  s_spectro.pWindow   = (float32_t *)n6_aed_window;
  s_spectro.SampRate  = N6_AED_SAMPLE_RATE;
  s_spectro.FrameLen  = N6_AED_WIN_LEN;
  s_spectro.FFTLen    = N6_AED_NFFT;
  s_spectro.pad_left  = 0;
  s_spectro.pad_right = 0;
  s_spectro.pScratch1 = s_scratch1;
  s_spectro.pScratch2 = s_scratch2;

  s_melspec.SpectrogramConf = &s_spectro;
  s_melspec.MelFilter       = &s_mel;

  s_logmel.MelSpectrogramConf = &s_melspec;
  s_logmel.LogFormula         = LOGMELSPECTROGRAM_SCALE_LOG;
  s_logmel.Ref                = 1.0f;
  s_logmel.TopdB              = 80.0f;

  memset(s_spec, 0, sizeof(s_spec));
  memset(s_img, 0, sizeof(s_img));
  memset(s_scores, 0, sizeof(s_scores));
  s_win_fill = s_frames_since_infer = 0;
  s_ready = s_primed = false;
  s_level = s_level_run = 0;

  s_ok = true;
  return true;
}

/* Feeding */

/* Appends one mel column computed from the full window. */
static void emit_frame(void)
{
  memmove(s_spec, s_spec + N6_AED_NMEL,
          (N6_AED_NFRAMES - 1u) * N6_AED_NMEL * sizeof(int8_t));

  LogMelSpectrogramColumn_q15_Q8(&s_logmel, s_win,
                                 &s_spec[(N6_AED_NFRAMES - 1u) * N6_AED_NMEL],
                                 s_in_offset, s_in_inv_scale);

  if (s_frames_since_infer < 0xFFFFu) s_frames_since_infer++;
  if (!s_primed && s_frames_since_infer >= N6_AED_NFRAMES) s_primed = true;

  if (s_primed && s_frames_since_infer >= N6_AED_INFER_HOP) s_ready = true;

  memmove(s_win, s_win + N6_AED_HOP,
          (N6_AED_WIN_LEN - N6_AED_HOP) * sizeof(int16_t));
  s_win_fill = N6_AED_WIN_LEN - N6_AED_HOP;
}

void n6_aed_push(const int16_t *pcm, uint32_t n)
{
  if (!s_ok || pcm == NULL) return;

  for (uint32_t i = 0; i < n; i++) {
    const int16_t cur = pcm[i];

    int32_t a = (cur < 0) ? -(int32_t)cur : (int32_t)cur;
    if (a > 32767) a = 32767;
    if (a > (int32_t)s_level_run) s_level_run = (uint16_t)a;

    /* Linear resample. The phase counts input samples in Q16. */
    while (s_phase_q16 < (1u << 16)) {
      const uint32_t frac = s_phase_q16;
      /* 64-bit: a full-scale step times frac exceeds int32. */
      const int32_t  out  = (int32_t)s_prev_in +
                            (int32_t)((((int64_t)cur - (int64_t)s_prev_in) * (int64_t)frac) >> 16);

      if (s_win_fill < N6_AED_WIN_LEN) s_win[s_win_fill++] = (int16_t)out;
      if (s_win_fill >= N6_AED_WIN_LEN) {
        emit_frame();
      }
      s_phase_q16 += s_step_q16;
    }
    s_phase_q16 -= (1u << 16);
    s_prev_in = cur;
  }
}

bool n6_aed_ready(void) { return s_ready; }

/* Inference */

void n6_aed_run(void)
{
  s_ready = false;
  s_frames_since_infer = 0;
  if (!s_ok || s_model == NULL || s_in_buf == NULL) return;

  s_level = s_level_run;
  s_level_run = 0;

  /* Back to the uint8 grid the interface reads. */
  for (uint32_t i = 0; i < s_in_bytes; i++) {
    const uint8_t b = (uint8_t)((int32_t)s_spec[i] + 128);
    s_in_buf[i] = (int8_t)b;
    s_img[i]    = b;
  }

  /* The NPU is a separate bus master and does not see the CPU's data cache. */
  SCB_CleanDCache_by_Addr((uint32_t *)s_in_buf, (int32_t)((s_in_bytes + 31u) & ~31u));

  NPU_Run_Inference_Direct(s_model);

  const LL_Buffer_InfoTypeDef *out = LL_ATON_Output_Buffers_Info(s_model);
  if (out == NULL || out[0].name == NULL) return;
  const uint8_t *ostart = LL_Buffer_addr_start(&out[0]);
  const uint8_t *oend   = LL_Buffer_addr_end(&out[0]);
  if (ostart == NULL || oend == NULL) return;
  if ((uint32_t)(oend - ostart) != N6_AED_NUM_CLASSES * sizeof(float)) return;

  memcpy(s_scores, ostart, sizeof(s_scores));
}

int n6_aed_top(void)
{
  int   best = -1;
  float bv   = s_threshold;
  for (uint32_t i = 0; i < N6_AED_NUM_CLASSES; i++) {
    if (s_scores[i] > bv) { bv = s_scores[i]; best = (int)i; }
  }
  return best;
}
