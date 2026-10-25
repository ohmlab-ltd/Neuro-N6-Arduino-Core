/**
 ******************************************************************************
 * @file    n6_spectrum.c
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "n6_spectrum.h"

#include <math.h>
#include <string.h>
#include <stddef.h>

void n6_spectrum_init(n6_spectrum_t *s, int nbars, float sample_rate,
                      float f_lo, float f_hi)
{
  if (s == NULL) return;

  memset(s, 0, sizeof(*s));

  if (nbars < 1) nbars = 1;
  if (nbars > (int)N6_SPECTRUM_MAX_BARS) nbars = (int)N6_SPECTRUM_MAX_BARS;
  s->nbars = nbars;

  s->db_floor  = -70.0f;
  s->attack    = 0.55f;
  s->release   = 0.12f;
  s->peak_fall = 0.010f;

  if (!(sample_rate > 0.0f)) sample_rate = 32552.0f;

  const float nyquist = sample_rate * 0.5f;
  if (!(f_hi > 0.0f) || f_hi > nyquist) f_hi = nyquist;
  if (!(f_lo > 0.0f)) f_lo = 40.0f;
  if (f_lo >= f_hi)   f_lo = f_hi * 0.01f;

  const float hz_per_bin = sample_rate / (float)N6_FFT_SIZE;
  const float ratio      = f_hi / f_lo;

  for (int b = 0; b < nbars; b++) {
    const float f0 = f_lo * powf(ratio, (float)b       / (float)nbars);
    const float f1 = f_lo * powf(ratio, (float)(b + 1) / (float)nbars);

    int lo = (int)(f0 / hz_per_bin + 0.5f);
    int hi = (int)(f1 / hz_per_bin + 0.5f);

    /* Bin 0 is DC and is left out. */
    if (lo < 1) lo = 1;
    if (hi <= lo) hi = lo + 1;                 /* at least one bin per bar */
    if (hi > (int)N6_FFT_BINS) hi = (int)N6_FFT_BINS;
    if (lo >= hi) lo = hi - 1;

    s->lo[b] = (uint16_t)lo;
    s->hi[b] = (uint16_t)hi;
    s->centre_hz[b] = sqrtf(f0 * f1);          /* geometric mean */
  }
}

void n6_spectrum_update(n6_spectrum_t *s, const float *mag)
{
  if (s == NULL || mag == NULL) return;

  for (int b = 0; b < s->nbars; b++) {
    float m = 0.0f;
    for (uint16_t k = s->lo[b]; k < s->hi[b]; k++) {
      if (mag[k] > m) m = mag[k];
    }

    /* dBFS: full scale is magnitude 1.0. */
    const float db = 20.0f * log10f(m + 1e-9f);

    float v = (db - s->db_floor) / (0.0f - s->db_floor);
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;

    const float k = (v > s->bars[b]) ? s->attack : s->release;
    s->bars[b] += (v - s->bars[b]) * k;

    if (s->bars[b] > s->peaks[b]) {
      s->peaks[b] = s->bars[b];
    } else {
      s->peaks[b] -= s->peak_fall;
      if (s->peaks[b] < s->bars[b]) s->peaks[b] = s->bars[b];
      if (s->peaks[b] < 0.0f)       s->peaks[b] = 0.0f;
    }
  }
}

int n6_spectrum_loudest(const n6_spectrum_t *s)
{
  if (s == NULL) return -1;

  int   best = -1;
  float bv   = 0.02f;                 /* noise floor */
  for (int b = 0; b < s->nbars; b++) {
    if (s->bars[b] > bv) { bv = s->bars[b]; best = b; }
  }
  return best;
}
