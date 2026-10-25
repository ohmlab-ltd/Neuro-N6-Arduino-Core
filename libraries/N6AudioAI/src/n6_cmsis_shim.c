/**
 ******************************************************************************
 * @file    n6_cmsis_shim.c
 * @brief   The CMSIS-DSP functions ST's audio preprocessing library calls.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "arm_math.h"

/* Elementwise helpers */

void arm_mult_f32(const float32_t *pSrcA, const float32_t *pSrcB,
                  float32_t *pDst, uint32_t blockSize)
{
  for (uint32_t i = 0; i < blockSize; i++) pDst[i] = pSrcA[i] * pSrcB[i];
}

void arm_scale_f32(const float32_t *pSrc, float32_t scale,
                   float32_t *pDst, uint32_t blockSize)
{
  for (uint32_t i = 0; i < blockSize; i++) pDst[i] = pSrc[i] * scale;
}

void arm_dot_prod_f32(const float32_t *pSrcA, const float32_t *pSrcB,
                      uint32_t blockSize, float32_t *result)
{
  float32_t acc = 0.0f;
  for (uint32_t i = 0; i < blockSize; i++) acc += pSrcA[i] * pSrcB[i];
  *result = acc;
}

void arm_cmplx_mag_f32(const float32_t *pSrc, float32_t *pDst,
                       uint32_t numSamples)
{
  for (uint32_t i = 0; i < numSamples; i++) {
    const float32_t re = pSrc[2 * i], im = pSrc[2 * i + 1];
    pDst[i] = sqrtf(re * re + im * im);
  }
}

void arm_cmplx_mag_squared_f32(const float32_t *pSrc, float32_t *pDst,
                               uint32_t numSamples)
{
  for (uint32_t i = 0; i < numSamples; i++) {
    const float32_t re = pSrc[2 * i], im = pSrc[2 * i + 1];
    pDst[i] = re * re + im * im;
  }
}

/* Q15 conversions */

void arm_float_to_q15(const float32_t *pSrc, q15_t *pDst, uint32_t blockSize)
{
  for (uint32_t i = 0; i < blockSize; i++) {
    float32_t v = pSrc[i] * 32768.0f;
    v += (v >= 0.0f) ? 0.5f : -0.5f;              /* round to nearest */
    if (v > 32767.0f)  v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    pDst[i] = (q15_t)v;
  }
}

void arm_q15_to_float(const q15_t *pSrc, float32_t *pDst, uint32_t blockSize)
{
  for (uint32_t i = 0; i < blockSize; i++)
    pDst[i] = (float32_t)pSrc[i] / 32768.0f;
}

void arm_absmax_q15(const q15_t *pSrc, uint32_t blockSize,
                    q15_t *pResult, uint32_t *pIndex)
{
  q15_t    best = 0;
  uint32_t at   = 0;

  for (uint32_t i = 0; i < blockSize; i++) {
    /* -32768 clamps to 32767, as in CMSIS. */
    q15_t a = (pSrc[i] < 0) ? (q15_t)((pSrc[i] == (q15_t)0x8000) ? 0x7FFF : -pSrc[i])
                            : pSrc[i];
    if (a > best) { best = a; at = i; }
  }
  *pResult = best;
  *pIndex  = at;
}

/* Real FFT. Output packed as SpectrogramColumn() expects:
 *     pOut[0]     = X[0].real
 *     pOut[1]     = X[N/2].real
 *     pOut[2i]    = X[i].real        for i = 1 .. N/2-1
 *     pOut[2i+1]  = X[i].imag */

#ifndef N6_SHIM_FFT_MAX
#define N6_SHIM_FFT_MAX 1024u
#endif

static float32_t s_re[N6_SHIM_FFT_MAX];
static float32_t s_im[N6_SHIM_FFT_MAX];
/* Twiddles for N6_SHIM_FFT_MAX points. Length N uses every (MAX / N)th entry,
 * so instances of different lengths work side by side. */
static float32_t s_cos[N6_SHIM_FFT_MAX / 2];
static float32_t s_sin[N6_SHIM_FFT_MAX / 2];
static uint8_t   s_tw_ready = 0u;

arm_status arm_rfft_fast_init_f32(arm_rfft_fast_instance_f32 *S, uint16_t fftLen)
{
  if (S == NULL) return ARM_MATH_ARGUMENT_ERROR;
  if (fftLen < 2u || fftLen > N6_SHIM_FFT_MAX) return ARM_MATH_ARGUMENT_ERROR;
  if ((fftLen & (fftLen - 1u)) != 0u) return ARM_MATH_ARGUMENT_ERROR;  /* pow2 */

  /* Only fftLenRFFT is used. */
  S->fftLenRFFT   = fftLen;
  S->pTwiddleRFFT = NULL;

  if (!s_tw_ready) {
    for (uint32_t k = 0; k < N6_SHIM_FFT_MAX / 2u; k++) {
      const double a = -2.0 * 3.14159265358979323846 * (double)k / (double)N6_SHIM_FFT_MAX;
      s_cos[k] = (float32_t)cos(a);
      s_sin[k] = (float32_t)sin(a);
    }
    s_tw_ready = 1u;
  }
  return ARM_MATH_SUCCESS;
}

static uint32_t bit_reverse(uint32_t i, uint32_t log2n)
{
  uint32_t r = 0;
  for (uint32_t b = 0; b < log2n; b++) { r = (r << 1) | (i & 1u); i >>= 1; }
  return r;
}

/* Radix-2 FFT of s_re and s_im in place, loaded in bit-reversed order. sign
 * -1 gives the inverse, unscaled. */
static void fft_core(uint32_t N, float32_t sign)
{
  const uint32_t scale = N6_SHIM_FFT_MAX / N;
  for (uint32_t len = 2; len <= N; len <<= 1) {
    const uint32_t half = len >> 1;
    const uint32_t step = (N / len) * scale;
    for (uint32_t base = 0; base < N; base += len) {
      for (uint32_t k = 0; k < half; k++) {
        const float32_t c = s_cos[k * step];
        const float32_t s = sign * s_sin[k * step];
        const uint32_t a = base + k, b = a + half;
        const float32_t vr = s_re[b] * c - s_im[b] * s;
        const float32_t vi = s_re[b] * s + s_im[b] * c;
        s_re[b] = s_re[a] - vr;  s_im[b] = s_im[a] - vi;
        s_re[a] = s_re[a] + vr;  s_im[a] = s_im[a] + vi;
      }
    }
  }
}

void arm_rfft_fast_f32(const arm_rfft_fast_instance_f32 *S,
                       float32_t *p, float32_t *pOut, uint8_t ifftFlag)
{
  if (S == NULL || p == NULL || pOut == NULL || !s_tw_ready) return;

  const uint32_t N = S->fftLenRFFT;
  if (N < 2u || N > N6_SHIM_FFT_MAX || (N & (N - 1u)) != 0u) return;

  uint32_t log2n = 0;
  while ((1u << log2n) < N) log2n++;

  if (ifftFlag) {
    /* p is a packed spectrum. The full one is conjugate-symmetric. */
    for (uint32_t k = 0; k < N; k++) {
      float32_t re, im;
      if (k == 0u)          { re = p[0];             im = 0.0f; }
      else if (k == N / 2u) { re = p[1];             im = 0.0f; }
      else if (k < N / 2u)  { re = p[2u * k];        im = p[2u * k + 1u]; }
      else                  { re = p[2u * (N - k)];  im = -p[2u * (N - k) + 1u]; }
      const uint32_t r = bit_reverse(k, log2n);
      s_re[r] = re;
      s_im[r] = im;
    }
    fft_core(N, -1.0f);
    const float32_t inv = 1.0f / (float32_t)N;
    for (uint32_t i = 0; i < N; i++) pOut[i] = s_re[i] * inv;
    return;
  }

  for (uint32_t i = 0; i < N; i++) {
    const uint32_t r = bit_reverse(i, log2n);
    s_re[r] = p[i];
    s_im[r] = 0.0f;
  }
  fft_core(N, 1.0f);

  pOut[0] = s_re[0];                          /* DC */
  pOut[1] = s_re[N / 2u];                     /* Nyquist */
  for (uint32_t k = 1; k < N / 2u; k++) {
    pOut[2u * k]      = s_re[k];
    pOut[2u * k + 1u] = s_im[k];
  }
}
