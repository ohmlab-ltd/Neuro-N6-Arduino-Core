/**
 ******************************************************************************
 * @file    n6_primitives.c
 * @brief   Grayscale image primitives.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "n6_imgproc.h"
#include <string.h>

static void clip(const n6img_rect_t *roi, int w, int h,
                 int *x0, int *y0, int *x1, int *y1)
{
  if (roi == NULL) { *x0 = 0; *y0 = 0; *x1 = w; *y1 = h; return; }
  int ax = roi->x, ay = roi->y;
  int bx = roi->x + roi->w, by = roi->y + roi->h;
  if (ax < 0) ax = 0;  if (ay < 0) ay = 0;
  if (bx > w) bx = w;  if (by > h) by = h;
  if (bx < ax) bx = ax;
  if (by < ay) by = ay;
  *x0 = ax; *y0 = ay; *x1 = bx; *y1 = by;
}

void n6img_rgb888_to_gray(const uint8_t *src, uint8_t *dst, int w, int h)
{
  if (src == NULL || dst == NULL || w <= 0 || h <= 0) return;
  const uint32_t n = (uint32_t)w * (uint32_t)h;
  /* BT.601 luma: 0.299, 0.587 and 0.114 scaled by 1024. */
  for (uint32_t i = 0; i < n; i++) {
    const uint32_t r = src[i * 3u + 0u];
    const uint32_t g = src[i * 3u + 1u];
    const uint32_t b = src[i * 3u + 2u];
    dst[i] = (uint8_t)((306u * r + 601u * g + 117u * b) >> 10);
  }
}

void n6img_histogram(const uint8_t *gray, int w, int h,
                     const n6img_rect_t *roi, uint32_t bins[256])
{
  if (bins == NULL) return;
  memset(bins, 0, 256u * sizeof(uint32_t));
  if (gray == NULL || w <= 0 || h <= 0) return;

  int x0, y0, x1, y1;
  clip(roi, w, h, &x0, &y0, &x1, &y1);
  for (int y = y0; y < y1; y++) {
    const uint8_t *row = gray + (size_t)y * (size_t)w;
    for (int x = x0; x < x1; x++) bins[row[x]]++;
  }
}

void n6img_statistics(const uint8_t *gray, int w, int h,
                      const n6img_rect_t *roi, n6img_stats_t *out)
{
  if (out == NULL) return;
  memset(out, 0, sizeof(*out));
  if (gray == NULL || w <= 0 || h <= 0) return;

  uint32_t bins[256];
  n6img_histogram(gray, w, h, roi, bins);

  uint32_t n = 0u;
  uint64_t sum = 0u, sumsq = 0u;
  int lo = -1, hi = -1;
  for (int v = 0; v < 256; v++) {
    const uint32_t c = bins[v];
    if (c == 0u) continue;
    if (lo < 0) lo = v;
    hi = v;
    n += c;
    sum   += (uint64_t)c * (uint64_t)v;
    sumsq += (uint64_t)c * (uint64_t)v * (uint64_t)v;
  }
  if (n == 0u) return;

  out->pixels = n;
  out->min = (uint8_t)lo;
  out->max = (uint8_t)hi;
  out->mean = (uint8_t)(sum / n);

  uint32_t half = n / 2u, run = 0u;
  for (int v = 0; v < 256; v++) {
    run += bins[v];
    if (run > half) { out->median = (uint8_t)v; break; }
  }

  /* variance = E[x^2] - E[x]^2, kept in x256 fixed point. */
  const uint64_t mean_q = (sum * 256u) / n;
  const uint64_t ex2_q  = (sumsq * 65536u) / n;
  const uint64_t m2_q   = mean_q * mean_q;
  uint64_t var_q = (ex2_q > m2_q) ? (ex2_q - m2_q) : 0u;
  uint32_t r = 0u, bit = 1u << 30;
  while (bit > var_q) bit >>= 2;
  while (bit != 0u) {                       /* integer sqrt */
    if (var_q >= r + bit) { var_q -= r + bit; r = (r >> 1) + bit; }
    else r >>= 1;
    bit >>= 2;
  }
  out->stdev = r;
}

uint8_t n6img_otsu(const uint32_t bins[256])
{
  if (bins == NULL) return 128u;
  uint64_t total = 0u, sum = 0u;
  for (int v = 0; v < 256; v++) { total += bins[v]; sum += (uint64_t)v * bins[v]; }
  if (total == 0u) return 128u;

  /* Every threshold between two modes scores the same, so the optimum is a
   * plateau. Its middle is returned. */
  uint64_t sum_b = 0u, w_b = 0u;
  uint64_t best = 0u;
  int best_lo = 0, best_hi = 0;
  for (int t = 0; t < 256; t++) {
    w_b += bins[t];
    if (w_b == 0u) continue;
    const uint64_t w_f = total - w_b;
    if (w_f == 0u) break;
    sum_b += (uint64_t)t * bins[t];
    const uint64_t m_b = sum_b / w_b;
    const uint64_t m_f = (sum - sum_b) / w_f;
    const int64_t  d   = (int64_t)m_b - (int64_t)m_f;
    const uint64_t var = w_b * w_f * (uint64_t)(d * d);
    if (var > best)      { best = var; best_lo = best_hi = t; }
    else if (var == best && best > 0u) { best_hi = t; }
  }
  return (uint8_t)((best_lo + best_hi) / 2);
}

void n6img_threshold(const uint8_t *gray, uint8_t *dst, int w, int h,
                     uint8_t lo, uint8_t hi)
{
  if (gray == NULL || dst == NULL || w <= 0 || h <= 0) return;
  const uint32_t n = (uint32_t)w * (uint32_t)h;
  for (uint32_t i = 0; i < n; i++) {
    const uint8_t v = gray[i];
    dst[i] = (v >= lo && v <= hi) ? 255u : 0u;
  }
}

/* One 3x3 pass. Out-of-bounds neighbours count as background. */
static void morph_pass(const uint8_t *src, uint8_t *dst, int w, int h,
                       bool erode)
{
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      bool any = false, all = true;
      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          const int nx = x + dx, ny = y + dy;
          const bool set = (nx >= 0 && nx < w && ny >= 0 && ny < h)
                         ? (src[(size_t)ny * (size_t)w + (size_t)nx] != 0u)
                         : false;
          if (set) any = true; else all = false;
        }
      }
      dst[(size_t)y * (size_t)w + (size_t)x] =
          (uint8_t)((erode ? all : any) ? 255 : 0);
    }
  }
}

static void morph(uint8_t *bin, uint8_t *scratch, int w, int h,
                  int iterations, bool erode)
{
  if (bin == NULL || scratch == NULL || w <= 0 || h <= 0) return;
  const size_t n = (size_t)w * (size_t)h;
  for (int i = 0; i < iterations; i++) {
    morph_pass(bin, scratch, w, h, erode);
    memcpy(bin, scratch, n);
  }
}

void n6img_erode(uint8_t *bin, uint8_t *scratch, int w, int h, int iterations)
{ morph(bin, scratch, w, h, iterations, true); }

void n6img_dilate(uint8_t *bin, uint8_t *scratch, int w, int h, int iterations)
{ morph(bin, scratch, w, h, iterations, false); }

void n6img_absdiff(const uint8_t *a, const uint8_t *b, uint8_t *dst,
                   int w, int h)
{
  if (a == NULL || b == NULL || dst == NULL || w <= 0 || h <= 0) return;
  const uint32_t n = (uint32_t)w * (uint32_t)h;
  for (uint32_t i = 0; i < n; i++) {
    dst[i] = (a[i] > b[i]) ? (uint8_t)(a[i] - b[i]) : (uint8_t)(b[i] - a[i]);
  }
}

/* Connected components */

#define N6IMG_MAX_LABELS 2048u

/* Two-pass labelling with union-find, bounded by the label table. */
static uint16_t uf_find(uint16_t *parent, uint16_t a)
{
  while (parent[a] != a) { parent[a] = parent[parent[a]]; a = parent[a]; }
  return a;
}

static void uf_union(uint16_t *parent, uint16_t a, uint16_t b)
{
  a = uf_find(parent, a); b = uf_find(parent, b);
  if (a == b) return;
  if (a < b) parent[b] = a; else parent[a] = b;
}

/* Relabels pixels 0..upto-1 with their roots, numbered from 1, so labels that
 * were merged can be used again. Returns the next free label. */
static uint16_t compact_labels(uint16_t *parent, uint16_t next, uint16_t *labels, size_t upto)
{
  static uint16_t remap[N6IMG_MAX_LABELS];
  uint16_t n = 1u;
  for (uint16_t l = 1u; l < next; l++) remap[l] = 0u;
  for (uint16_t l = 1u; l < next; l++) {
    const uint16_t r = uf_find(parent, l);
    if (remap[r] == 0u) remap[r] = n++;
  }
  for (size_t i = 0; i < upto; i++) {
    if (labels[i] != 0u) labels[i] = remap[uf_find(parent, labels[i])];
  }
  for (uint16_t l = 1u; l < n; l++) parent[l] = l;
  return n;
}

int n6img_find_components(const uint8_t *bin, uint16_t *labels, int w, int h,
                          uint32_t min_px,
                          n6img_component_t *out, int max_out)
{
  if (bin == NULL || labels == NULL || out == NULL || max_out <= 0) return -1;
  if (w <= 0 || h <= 0) return -1;

  static uint16_t parent[N6IMG_MAX_LABELS];
  uint16_t next = 1u;
  memset(labels, 0, (size_t)w * (size_t)h * sizeof(uint16_t));
  parent[0] = 0u;

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      const size_t i = (size_t)y * (size_t)w + (size_t)x;
      if (bin[i] == 0u) continue;

      /* Neighbours already visited: W, NW, N, NE. */
      uint16_t best = 0u;
      const int dx[4] = { -1, -1, 0, 1 };
      const int dy[4] = {  0, -1, -1, -1 };
      for (int k = 0; k < 4; k++) {
        const int nx = x + dx[k], ny = y + dy[k];
        if (nx < 0 || nx >= w || ny < 0) continue;
        const uint16_t l = labels[(size_t)ny * (size_t)w + (size_t)nx];
        if (l == 0u) continue;
        if (best == 0u) best = l; else uf_union(parent, best, l);
      }
      if (best == 0u) {
        if (next >= N6IMG_MAX_LABELS) {
          next = compact_labels(parent, next, labels, i);
          if (next >= N6IMG_MAX_LABELS) return -2;   /* too many components */
        }
        best = next; parent[next] = next; next++;
      }
      labels[i] = best;
    }
  }

  static uint32_t px[N6IMG_MAX_LABELS];
  static uint32_t sx[N6IMG_MAX_LABELS], sy[N6IMG_MAX_LABELS];
  static uint16_t bx0[N6IMG_MAX_LABELS], by0[N6IMG_MAX_LABELS];
  static uint16_t bx1[N6IMG_MAX_LABELS], by1[N6IMG_MAX_LABELS];
  for (uint16_t i = 0; i < next; i++) {
    px[i] = 0u; sx[i] = 0u; sy[i] = 0u;
    bx0[i] = 0xFFFFu; by0[i] = 0xFFFFu; bx1[i] = 0u; by1[i] = 0u;
  }
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      const uint16_t l = labels[(size_t)y * (size_t)w + (size_t)x];
      if (l == 0u) continue;
      const uint16_t r = uf_find(parent, l);
      px[r]++; sx[r] += (uint32_t)x; sy[r] += (uint32_t)y;
      if ((uint16_t)x < bx0[r]) bx0[r] = (uint16_t)x;
      if ((uint16_t)y < by0[r]) by0[r] = (uint16_t)y;
      if ((uint16_t)x > bx1[r]) bx1[r] = (uint16_t)x;
      if ((uint16_t)y > by1[r]) by1[r] = (uint16_t)y;
    }
  }

  int n = 0;
  for (uint16_t r = 1u; r < next; r++) {
    if (px[r] < min_px || uf_find(parent, r) != r) continue;
    n6img_component_t c;
    c.pixels = px[r];
    c.cx = ((float)sx[r] / (float)px[r]) / (float)w;
    c.cy = ((float)sy[r] / (float)px[r]) / (float)h;
    c.w  = (float)(bx1[r] - bx0[r] + 1u) / (float)w;
    c.h  = (float)(by1[r] - by0[r] + 1u) / (float)h;

    /* Insertion sort, largest first. */
    int pos = n;
    while (pos > 0 && out[pos - 1].pixels < c.pixels) pos--;
    if (pos >= max_out) continue;
    for (int k = (n < max_out ? n : max_out - 1); k > pos; k--) out[k] = out[k - 1];
    out[pos] = c;
    if (n < max_out) n++;
  }
  return n;
}
