/**
 ******************************************************************************
 * @file    n6_fft.c
 * @brief   Iterative radix-2 Cooley-Tukey FFT, decimation in time.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "n6_fft.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* exp(-2*pi*i*k/N). Stage len uses entry k * (N/len). */
static float s_tw_cos[N6_FFT_SIZE / 2u];
static float s_tw_sin[N6_FFT_SIZE / 2u];
static float s_win[N6_FFT_SIZE];
static bool  s_ready = false;

/* Scratch, too large for a task stack. One caller at a time. */
static float s_re[N6_FFT_SIZE];
static float s_im[N6_FFT_SIZE];

void n6_fft_init(void)
{
  if (s_ready) return;

  for (uint32_t k = 0; k < N6_FFT_SIZE / 2u; k++) {
    const double a = -2.0 * M_PI * (double)k / (double)N6_FFT_SIZE;
    s_tw_cos[k] = (float)cos(a);
    s_tw_sin[k] = (float)sin(a);
  }

  /* Hann window. */
  for (uint32_t n = 0; n < N6_FFT_SIZE; n++) {
    s_win[n] = (float)(0.5 - 0.5 * cos(2.0 * M_PI * (double)n /
                                       (double)(N6_FFT_SIZE - 1u)));
  }

  s_ready = true;
}

static inline uint32_t bit_reverse(uint32_t i)
{
  uint32_t r = 0;
  for (uint32_t b = 0; b < N6_FFT_LOG2; b++) {
    r = (r << 1) | (i & 1u);
    i >>= 1;
  }
  return r;
}

void n6_fft_magnitude(const int16_t *pcm, float *mag)
{
  if (pcm == NULL || mag == NULL) return;
  if (!s_ready) n6_fft_init();

  float mean = 0.0f;
  for (uint32_t i = 0; i < N6_FFT_SIZE; i++) mean += (float)pcm[i];
  mean /= (float)N6_FFT_SIZE;

  for (uint32_t i = 0; i < N6_FFT_SIZE; i++) {
    const float v = (((float)pcm[i] - mean) * (1.0f / 32768.0f)) * s_win[i];
    const uint32_t j = bit_reverse(i);
    s_re[j] = v;
    s_im[j] = 0.0f;
  }

  for (uint32_t len = 2; len <= N6_FFT_SIZE; len <<= 1) {
    const uint32_t half = len >> 1;
    const uint32_t step = N6_FFT_SIZE / len;      /* twiddle stride */

    for (uint32_t base = 0; base < N6_FFT_SIZE; base += len) {
      for (uint32_t k = 0; k < half; k++) {
        const uint32_t tw = k * step;
        const float c = s_tw_cos[tw];
        const float s = s_tw_sin[tw];

        const uint32_t a = base + k;
        const uint32_t b = a + half;

        const float vr = s_re[b] * c - s_im[b] * s;
        const float vi = s_re[b] * s + s_im[b] * c;

        s_re[b] = s_re[a] - vr;
        s_im[b] = s_im[a] - vi;
        s_re[a] = s_re[a] + vr;
        s_im[a] = s_im[a] + vi;
      }
    }
  }

  /* A full-scale sine reads 1.0: N/2 in its bin, halved by the Hann window. */
  const float norm = 4.0f / (float)N6_FFT_SIZE;
  for (uint32_t k = 0; k < N6_FFT_BINS; k++) {
    mag[k] = sqrtf(s_re[k] * s_re[k] + s_im[k] * s_im[k]) * norm;
  }
}

/* Sliding analysis window */

void n6_frame_init(n6_frame_t *f)
{
  if (f == NULL) return;
  memset(f->buf, 0, sizeof(f->buf));
  f->filled = 0;
}

bool n6_frame_push(n6_frame_t *f, const int16_t *pcm, uint32_t n)
{
  if (f == NULL || pcm == NULL || n == 0) return false;

  if (n >= N6_FFT_SIZE) {
    memcpy(f->buf, pcm + (n - N6_FFT_SIZE), sizeof(f->buf));
    f->filled = N6_FFT_SIZE;
    return true;
  }

  const uint32_t keep = N6_FFT_SIZE - n;
  memmove(f->buf, f->buf + n, keep * sizeof(int16_t));
  memcpy(f->buf + keep, pcm, n * sizeof(int16_t));

  if (f->filled < N6_FFT_SIZE) {
    f->filled += n;
    if (f->filled > N6_FFT_SIZE) f->filled = N6_FFT_SIZE;
  }
  return f->filled >= N6_FFT_SIZE;
}
