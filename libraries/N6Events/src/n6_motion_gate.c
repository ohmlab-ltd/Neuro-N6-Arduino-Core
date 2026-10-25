/**
 ******************************************************************************
 * @file    n6_motion_gate.c
 * @brief   Strided grey thumbnail difference with hysteresis.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "n6_motion_gate.h"
#include <string.h>

extern void *n6_psram_alloc(uint32_t bytes);

bool n6_motion_gate_begin(n6_motion_gate_t *g, int w, int h, uint8_t *mem)
{
  if (g == NULL || w <= 0 || h <= 0) return false;
  memset(g, 0, sizeof(*g));
  const uint32_t plane = (uint32_t)w * (uint32_t)h;
  if (mem == NULL) mem = (uint8_t *)n6_psram_alloc(2u * plane);
  if (mem == NULL) return false;
  g->prev = mem;
  g->cur  = mem + plane;
  g->w = w; g->h = h;
  g->delta        = 24u;
  g->on_permille  = 30u;
  g->off_permille = 10u;
  return true;
}

void n6_motion_gate_reset(n6_motion_gate_t *g)
{
  if (g == NULL) return;
  g->have_prev  = false;
  g->active     = false;
  g->last_score = 0u;
}

uint16_t n6_motion_gate_score(n6_motion_gate_t *g, const uint8_t *rgb888,
                              int src_w, int src_h)
{
  if (g == NULL || g->cur == NULL || rgb888 == NULL || src_w <= 0 || src_h <= 0) return 0u;

  /* Strided sample as integer luma, close to BT.601. */
  uint8_t *dst = g->cur;
  for (int y = 0; y < g->h; y++) {
    const int sy = (int)(((int64_t)y * src_h) / g->h);
    const uint8_t *row = rgb888 + (size_t)sy * (size_t)src_w * 3u;
    for (int x = 0; x < g->w; x++) {
      const int sx = (int)(((int64_t)x * src_w) / g->w);
      const uint8_t *p = row + (size_t)sx * 3u;
      *dst++ = (uint8_t)((77u * p[0] + 150u * p[1] + 29u * p[2]) >> 8);
    }
  }

  uint16_t score = 0u;
  if (g->have_prev) {
    const uint32_t n = (uint32_t)g->w * (uint32_t)g->h;
    uint32_t changed = 0u;
    for (uint32_t i = 0u; i < n; i++) {
      const int d = (int)g->cur[i] - (int)g->prev[i];
      if (d > (int)g->delta || d < -(int)g->delta) changed++;
    }
    score = (uint16_t)((changed * 1000u) / n);
  }

  uint8_t *t = g->prev; g->prev = g->cur; g->cur = t;
  g->have_prev  = true;
  g->last_score = score;
  return score;
}

bool n6_motion_gate_update(n6_motion_gate_t *g, const uint8_t *rgb888,
                           int src_w, int src_h)
{
  const uint16_t s = n6_motion_gate_score(g, rgb888, src_w, src_h);
  if (g == NULL) return false;
  if (!g->active && s >= g->on_permille)  g->active = true;
  else if (g->active && s <= g->off_permille) g->active = false;
  return g->active;
}
