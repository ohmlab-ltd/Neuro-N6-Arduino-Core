/**
 ******************************************************************************
 * @file    n6_fft.h
 * @brief   Real-input FFT and magnitude spectrum for 16-bit PCM.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6_FFT_H
#define N6_FFT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define N6_FFT_LOG2   12u
#define N6_FFT_SIZE   (1u << N6_FFT_LOG2)      /* 4096 samples in */
#define N6_FFT_BINS   (N6_FFT_SIZE / 2u)       /* 2048 bins out */

/* New samples per analysis. */
#define N6_FFT_HOP    1024u

/* Builds the tables. Safe to call more than once. */
void n6_fft_init(void);

/* Applies a Hann window to N6_FFT_SIZE samples of pcm and writes N6_FFT_BINS
 * magnitudes to mag. A full-scale sine reads 1.0. */
void n6_fft_magnitude(const int16_t *pcm, float *mag);

/* Sliding window of the newest N6_FFT_SIZE samples. */
typedef struct {
  int16_t  buf[N6_FFT_SIZE];
  uint32_t filled;            /* counts up to N6_FFT_SIZE */
} n6_frame_t;

void n6_frame_init(n6_frame_t *f);

/* Appends n samples, discarding the oldest. True once the window is full. */
bool n6_frame_push(n6_frame_t *f, const int16_t *pcm, uint32_t n);

static inline const int16_t *n6_frame_data(const n6_frame_t *f)
{
  return f->buf;
}

static inline float n6_fft_bin_hz(uint32_t k, float sample_rate)
{
  return ((float)k * sample_rate) / (float)N6_FFT_SIZE;
}

#ifdef __cplusplus
}
#endif

#endif /* N6_FFT_H */
