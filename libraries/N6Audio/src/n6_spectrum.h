/**
 ******************************************************************************
 * @file    n6_spectrum.h
 * @brief   Display bars from FFT magnitudes: log bands, dB scale, peak hold.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6_SPECTRUM_H
#define N6_SPECTRUM_H

#include <stdint.h>
#include "n6_fft.h"

#ifdef __cplusplus
extern "C" {
#endif

#define N6_SPECTRUM_MAX_BARS  96u

typedef struct {
  int      nbars;
  uint16_t lo[N6_SPECTRUM_MAX_BARS];    /* first bin of each band */
  uint16_t hi[N6_SPECTRUM_MAX_BARS];    /* one past the last */
  float    centre_hz[N6_SPECTRUM_MAX_BARS];

  float    bars[N6_SPECTRUM_MAX_BARS];  /* 0..1, smoothed */
  float    peaks[N6_SPECTRUM_MAX_BARS]; /* 0..1, peak hold */

  /* Tunable after init. */
  float    db_floor;    /* dB that maps to 0.0, default -70 */
  float    attack;      /* 0..1 rise per frame, default 0.55 */
  float    release;     /* 0..1 fall per frame, default 0.12 */
  float    peak_fall;   /* peak decay per frame, default 0.010 */
} n6_spectrum_t;

/* Lays out nbars log-spaced bands from f_lo to f_hi, clamped to Nyquist. */
void n6_spectrum_init(n6_spectrum_t *s, int nbars, float sample_rate,
                      float f_lo, float f_hi);

/* Folds one frame of magnitudes into the bars, the loudest bin per band. */
void n6_spectrum_update(n6_spectrum_t *s, const float *mag);

/* Loudest band, or -1 when none is above the floor. */
int  n6_spectrum_loudest(const n6_spectrum_t *s);

#ifdef __cplusplus
}
#endif

#endif /* N6_SPECTRUM_H */
