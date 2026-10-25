/**
 ******************************************************************************
 * @file    n6_blob.c
 * @brief   Colour blob finder.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "n6_imgproc.h"
#include <string.h>

/* Density grid */
#define GRID_W 32
#define GRID_H 32
#define GRID_N (GRID_W * GRID_H)

/* Static scratch: n6_blob_find_rgb888 is not reentrant. */
static uint16_t s_grid[GRID_N];
static uint16_t s_queue[GRID_N];
static uint8_t  s_visited[GRID_N];

/* Hue in degrees. mx is max(r,g,b) and chroma is max-min. */
static int32_t rgb_hue(int r, int g, int b, int mx, int chroma)
{
  int32_t h;
  if (mx == r)      h = (60 * (g - b)) / chroma;
  else if (mx == g) h = 120 + (60 * (b - r)) / chroma;
  else              h = 240 + (60 * (r - g)) / chroma;
  if (h < 0) h += 360;
  return h;
}

int32_t n6_hue_at_rgb888(const uint8_t *rgb, int w, int x, int y)
{
  if (rgb == NULL || w <= 0) return 0;
  const uint8_t *p = rgb + ((size_t)y * (size_t)w + (size_t)x) * 3u;
  const int r = p[0], g = p[1], b = p[2];
  const int mx = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
  const int mn = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
  const int c  = mx - mn;
  return (c == 0) ? 0 : rgb_hue(r, g, b, mx, c);
}

bool n6_blob_find_rgb888(const uint8_t *rgb, int w, int h,
                         const n6_blob_cfg_t *cfg, n6_blob_t *out)
{
  if (rgb == NULL || cfg == NULL || out == NULL) return false;
  if (w < GRID_W || h < GRID_H) return false;

  memset(s_grid, 0, sizeof(s_grid));
  {
    const uint8_t *src = rgb;
    for (int y = 0; y < h; y++) {
      const int gy = y * GRID_H / h;
      for (int x = 0; x < w; x++) {
        const int r = src[0], g = src[1], b = src[2];
        src += 3;

        const int mx = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
        const int mn = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
        const int chroma = mx - mn;

        if (mx < cfg->val_min || chroma < cfg->sat_min) continue;

        if (cfg->gate_hue) {
          const int32_t hh = rgb_hue(r, g, b, mx, chroma);
          int32_t dh = hh - cfg->target_hue;
          if (dh < 0) dh = -dh;
          if (dh > 180) dh = 360 - dh;      /* shorter way round the circle */
          if (dh > cfg->hue_tol) continue;
        }

        s_grid[gy * GRID_W + (x * GRID_W / w)]++;
      }
    }
  }

  int peak = 0, peak_i = -1;
  for (int i = 0; i < GRID_N; i++) {
    if (s_grid[i] > peak) { peak = s_grid[i]; peak_i = i; }
  }
  if (peak_i < 0 || peak < cfg->min_cell_hits) return false;

  /* A neighbour joins if it is at least a third as dense as the peak. */
  const int thr = (peak / 3 > cfg->min_cell_hits) ? peak / 3 : cfg->min_cell_hits;

  memset(s_visited, 0, sizeof(s_visited));
  int head = 0, tail = 0;
  s_queue[tail++] = (uint16_t)peak_i;
  s_visited[peak_i] = 1u;

  int cminx = GRID_W, cminy = GRID_H, cmaxx = -1, cmaxy = -1;
  uint32_t sumc = 0u, sumcx = 0u, sumcy = 0u;

  while (head < tail) {
    const int ci = s_queue[head++];
    const int gx = ci % GRID_W, gy = ci / GRID_W;
    const uint32_t c = s_grid[ci];

    /* Density-weighted centroid from each cell's centre, in source pixels. */
    sumc  += c;
    sumcx += c * (uint32_t)(((2 * gx + 1) * w) / (2 * GRID_W));
    sumcy += c * (uint32_t)(((2 * gy + 1) * h) / (2 * GRID_H));

    if (gx < cminx) cminx = gx;
    if (gx > cmaxx) cmaxx = gx;
    if (gy < cminy) cminy = gy;
    if (gy > cmaxy) cmaxy = gy;

    const int  nb[4] = { ci - 1, ci + 1, ci - GRID_W, ci + GRID_W };
    const bool ok[4] = { gx > 0, gx < GRID_W - 1, gy > 0, gy < GRID_H - 1 };
    for (int k = 0; k < 4; k++) {
      if (!ok[k]) continue;
      const int ni = nb[k];
      if (!s_visited[ni] && s_grid[ni] >= (uint16_t)thr) {
        s_visited[ni] = 1u;
        s_queue[tail++] = (uint16_t)ni;
      }
    }
  }

  if (sumc < (uint32_t)cfg->min_blob_px) return false;

  const float minx = (float)(cminx * w) / (float)GRID_W;
  const float maxx = (float)((cmaxx + 1) * w) / (float)GRID_W;
  const float miny = (float)(cminy * h) / (float)GRID_H;
  const float maxy = (float)((cmaxy + 1) * h) / (float)GRID_H;

  out->cx = ((minx + maxx) * 0.5f) / (float)w;
  out->cy = ((miny + maxy) * 0.5f) / (float)h;
  out->w  = (maxx - minx) / (float)w;
  out->h  = (maxy - miny) / (float)h;

  int ccx = (int)(sumcx / sumc);
  int ccy = (int)(sumcy / sumc);
  if (ccx < 0) ccx = 0; else if (ccx >= w) ccx = w - 1;
  if (ccy < 0) ccy = 0; else if (ccy >= h) ccy = h - 1;

  out->hue    = n6_hue_at_rgb888(rgb, w, ccx, ccy);
  out->pixels = sumc;
  out->area   = (float)sumc / ((float)w * (float)h);
  return true;
}
