/**
 ******************************************************************************
 * @file    n6_apriltag.c
 * @brief   tag36h11 detection.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "n6_apriltag.h"
#include "n6_tag36h11_codes.h"
#include <string.h>
#include <math.h>

#define MAXD            N6_APRILTAG_MAX_DIM
#define BITMAP_BYTES    ((MAXD * MAXD + 7) / 8)
#define MIN_BLOB_PX     64        /* smallest blob kept, pixels */

/* A blob that overflows either limit is dropped. */
#define MAX_BOUNDARY    2048      /* boundary points */
#define MAX_STACK       4096      /* flood-fill frontier */

static uint8_t  s_dark[BITMAP_BYTES];      /* 1 = below local threshold */
static uint8_t  s_seen[BITMAP_BYTES];      /* 1 = flood-filled */
static int16_t  s_bx[MAX_BOUNDARY];
static int16_t  s_by[MAX_BOUNDARY];
static int32_t  s_stack[MAX_STACK];

static n6_apriltag_stats_t s_stats;
static uint8_t s_dbg_cells[8][8];   /* last grid sampled */

void n6_apriltag_last_stats(n6_apriltag_stats_t *out)
{
  if (out != NULL) *out = s_stats;
}

static inline void bm_set(uint8_t *bm, int i) { bm[i >> 3] |= (uint8_t)(1u << (i & 7)); }
static inline int  bm_get(const uint8_t *bm, int i) { return (bm[i >> 3] >> (i & 7)) & 1; }

/* Adaptive threshold: each 8x8 tile against the min and max of its 3x3
 * neighbourhood. Holds up to a border ring about 24 px thick, a tag of about
 * 192 px. */
#define TILE 8
#define MAXT ((MAXD + TILE - 1) / TILE)

static uint8_t s_tmin[MAXT * MAXT];
static uint8_t s_tmax[MAXT * MAXT];

static void threshold(const uint8_t *g, int w, int h, int min_contrast)
{
  memset(s_dark, 0, (size_t)((w * h + 7) / 8));

  const int ntx = (w + TILE - 1) / TILE;
  const int nty = (h + TILE - 1) / TILE;

  for (int t = 0; t < nty; t++) {
    const int ty = t * TILE, y1 = (ty + TILE < h) ? ty + TILE : h;
    for (int u = 0; u < ntx; u++) {
      const int tx = u * TILE, x1 = (tx + TILE < w) ? tx + TILE : w;
      int mn = 255, mx = 0;
      for (int y = ty; y < y1; y++) {
        const uint8_t *row = g + (size_t)y * (size_t)w;
        for (int x = tx; x < x1; x++) {
          const int v = row[x];
          if (v < mn) mn = v;
          if (v > mx) mx = v;
        }
      }
      s_tmin[t * ntx + u] = (uint8_t)mn;
      s_tmax[t * ntx + u] = (uint8_t)mx;
    }
  }

  for (int t = 0; t < nty; t++) {
    const int ty = t * TILE, y1 = (ty + TILE < h) ? ty + TILE : h;
    for (int u = 0; u < ntx; u++) {
      const int tx = u * TILE, x1 = (tx + TILE < w) ? tx + TILE : w;

      int mn = 255, mx = 0;
      for (int dt = -1; dt <= 1; dt++) {
        const int tt = t + dt;
        if (tt < 0 || tt >= nty) continue;
        for (int du = -1; du <= 1; du++) {
          const int uu = u + du;
          if (uu < 0 || uu >= ntx) continue;
          const int k = tt * ntx + uu;
          if (s_tmin[k] < mn) mn = s_tmin[k];
          if (s_tmax[k] > mx) mx = s_tmax[k];
        }
      }
      if ((mx - mn) < min_contrast) continue;

      const int mid = (mn + mx) / 2;
      for (int y = ty; y < y1; y++) {
        const uint8_t *row = g + (size_t)y * (size_t)w;
        const int base = y * w;
        for (int x = tx; x < x1; x++) {
          if (row[x] < mid) bm_set(s_dark, base + x);
        }
      }
    }
  }
}

/* Flood-fills one dark blob and collects its boundary points. Returns the
 * pixel count, or 0 when MAX_STACK or MAX_BOUNDARY overflows. */
static int fill_blob(int seed, int w, int h, int *nb_out)
{
  int sp = 0, npx = 0, nb = 0;
  s_stack[sp++] = seed;
  bm_set(s_seen, seed);

  while (sp > 0) {
    const int i = s_stack[--sp];
    const int x = i % w, y = i / w;
    npx++;

    int is_boundary = 0;
    const int nx[4] = { x - 1, x + 1, x, x };
    const int ny[4] = { y, y, y - 1, y + 1 };
    for (int k = 0; k < 4; k++) {
      const int a = nx[k], b = ny[k];
      if (a < 0 || a >= w || b < 0 || b >= h) { is_boundary = 1; continue; }
      const int j = b * w + a;
      if (!bm_get(s_dark, j)) { is_boundary = 1; continue; }
      if (!bm_get(s_seen, j)) {
        if (sp >= MAX_STACK) { s_stats.drop_stack++;
                               if (npx > s_stats.drop_npx) s_stats.drop_npx = npx;
                               return 0; }
        bm_set(s_seen, j);
        s_stack[sp++] = j;
      }
    }
    if (is_boundary) {
      if (nb >= MAX_BOUNDARY) { s_stats.drop_boundary++;
                                if (npx > s_stats.drop_npx) s_stats.drop_npx = npx;
                                return 0; }
      s_bx[nb] = (int16_t)x;
      s_by[nb] = (int16_t)y;
      nb++;
    }
  }
  *nb_out = nb;
  return npx;
}

/* Fits a quad to the boundary points by the farthest-point method. */
static bool fit_quad(int nb, float q[4][2], int min_side)
{
  if (nb < 8) return false;

  float mx = 0.f, my = 0.f;
  for (int i = 0; i < nb; i++) { mx += s_bx[i]; my += s_by[i]; }
  mx /= (float)nb; my /= (float)nb;

  int i0 = 0; float best = -1.f;
  for (int i = 0; i < nb; i++) {
    const float dx = s_bx[i] - mx, dy = s_by[i] - my;
    const float d = dx * dx + dy * dy;
    if (d > best) { best = d; i0 = i; }
  }
  int i2 = 0; best = -1.f;
  for (int i = 0; i < nb; i++) {
    const float dx = s_bx[i] - s_bx[i0], dy = s_by[i] - s_by[i0];
    const float d = dx * dx + dy * dy;
    if (d > best) { best = d; i2 = i; }
  }

  const float ax = s_bx[i0], ay = s_by[i0];
  const float bx = s_bx[i2], by = s_by[i2];
  const float ex = bx - ax, ey = by - ay;

  int i1 = -1, i3 = -1;
  float bp = 0.f, bn = 0.f;
  for (int i = 0; i < nb; i++) {
    const float cr = ex * (s_by[i] - ay) - ey * (s_bx[i] - ax);
    if (cr > bp) { bp = cr; i1 = i; }
    if (cr < bn) { bn = cr; i3 = i; }
  }
  if (i1 < 0 || i3 < 0) return false;      /* all collinear */

  q[0][0] = s_bx[i0]; q[0][1] = s_by[i0];
  q[1][0] = s_bx[i1]; q[1][1] = s_by[i1];
  q[2][0] = s_bx[i2]; q[2][1] = s_by[i2];
  q[3][0] = s_bx[i3]; q[3][1] = s_by[i3];

  /* Normalises the winding. A quad wound the other way samples the tag
   * mirrored, which the decoder's rotation search cannot recover. */
  float area2 = 0.0f;
  for (int i = 0; i < 4; i++) {
    const int j = (i + 1) & 3;
    area2 += q[i][0] * q[j][1] - q[j][0] * q[i][1];
  }
  if (area2 < 0.0f) {
    float t;
    t = q[1][0]; q[1][0] = q[3][0]; q[3][0] = t;
    t = q[1][1]; q[1][1] = q[3][1]; q[3][1] = t;
  }

  for (int i = 0; i < 4; i++) {
    const float dx = q[(i + 1) & 3][0] - q[i][0];
    const float dy = q[(i + 1) & 3][1] - q[i][1];
    if (sqrtf(dx * dx + dy * dy) < (float)min_side) return false;
  }
  return true;
}

/* Homography from the unit square to the quad. Heckbert's closed form. */
typedef struct { float m[8]; } homog_t;

static bool homography(const float q[4][2], homog_t *H)
{
  const float x0 = q[0][0], y0 = q[0][1];
  const float x1 = q[1][0], y1 = q[1][1];
  const float x2 = q[2][0], y2 = q[2][1];
  const float x3 = q[3][0], y3 = q[3][1];

  const float dx1 = x1 - x2, dx2 = x3 - x2, sx = x0 - x1 + x2 - x3;
  const float dy1 = y1 - y2, dy2 = y3 - y2, sy = y0 - y1 + y2 - y3;

  const float den = dx1 * dy2 - dx2 * dy1;
  if (fabsf(den) < 1e-9f) return false;

  const float g = (sx * dy2 - dx2 * sy) / den;
  const float hh = (dx1 * sy - sx * dy1) / den;

  H->m[0] = x1 - x0 + g * x1;   H->m[1] = x3 - x0 + hh * x3;   H->m[2] = x0;
  H->m[3] = y1 - y0 + g * y1;   H->m[4] = y3 - y0 + hh * y3;   H->m[5] = y0;
  H->m[6] = g;                  H->m[7] = hh;
  return true;
}

static inline void hmap(const homog_t *H, float u, float v, float *x, float *y)
{
  const float d = H->m[6] * u + H->m[7] * v + 1.0f;
  const float s = (fabsf(d) < 1e-9f) ? 1e-9f : d;
  *x = (H->m[0] * u + H->m[1] * v + H->m[2]) / s;
  *y = (H->m[3] * u + H->m[4] * v + H->m[5]) / s;
}

/* Cell sampling and decoding */
#define WB N6_TAG36H11_WIDTH_AT_BORDER      /* 8 */

/* Mean of a 3x3 patch around the cell centre. */
static float cell_value(const uint8_t *g, int w, int h, const homog_t *H,
                        float cx_cell, float cy_cell)
{
  float sum = 0.f; int n = 0;
  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      float px, py;
      hmap(H, (cx_cell + dx * 0.18f) / (float)WB,
              (cy_cell + dy * 0.18f) / (float)WB, &px, &py);
      const int xi = (int)(px + 0.5f), yi = (int)(py + 0.5f);
      if (xi < 0 || xi >= w || yi < 0 || yi >= h) continue;
      sum += g[(size_t)yi * (size_t)w + (size_t)xi];
      n++;
    }
  }
  return (n > 0) ? (sum / (float)n) : 255.f;
}

static inline int popcount64(uint64_t v)
{
  v = v - ((v >> 1) & UINT64_C(0x5555555555555555));
  v = (v & UINT64_C(0x3333333333333333)) + ((v >> 2) & UINT64_C(0x3333333333333333));
  v = (v + (v >> 4)) & UINT64_C(0x0f0f0f0f0f0f0f0f);
  return (int)((v * UINT64_C(0x0101010101010101)) >> 56);
}

/* Rotates a cell coordinate by rot quarter turns within the WB x WB square. */
static inline void rot_cell(int rot, int x, int y, int *ox, int *oy)
{
  switch (rot & 3) {
    case 0:  *ox = x;              *oy = y;              break;
    case 1:  *ox = y;              *oy = WB - 1 - x;     break;
    case 2:  *ox = WB - 1 - x;     *oy = WB - 1 - y;     break;
    default: *ox = WB - 1 - y;     *oy = x;              break;
  }
}

/* Decodes the sampled grid. Returns the id, or -1. */
static int decode(const float cells[WB][WB], float black, float white,
                  int max_hamming, int *hamming_out, int *rot_out,
                  float *margin_out)
{
  const float mid = (black + white) * 0.5f;
  const float span = (white - black);

  for (int y = 0; y < WB; y++)
    for (int x = 0; x < WB; x++)
      s_dbg_cells[y][x] = (cells[y][x] > mid) ? 1u : 0u;
  if (span < 8.0f) { s_stats.rej_contrast++; return -1; }

  for (int i = 0; i < WB; i++) {
    if (cells[0][i]      > mid ||
        cells[WB - 1][i] > mid ||
        cells[i][0]      > mid ||
        cells[i][WB - 1] > mid) { s_stats.rej_border++; return -1; }
  }

  int best_id = -1, best_h = 1 << 30, best_rot = 0;
  float best_margin = 0.f;

  for (int rot = 0; rot < 4; rot++) {
    uint64_t code = 0;
    float worst = 1e9f;
    for (int b = 0; b < N6_TAG36H11_NBITS; b++) {
      int cx, cy;
      rot_cell(rot, n6_tag36h11_bit_x[b], n6_tag36h11_bit_y[b], &cx, &cy);
      const float v = cells[cy][cx];
      /* Bits are MSB first, and a white cell is a 1. */
      code = (code << 1) | ((v > mid) ? 1u : 0u);
      const float sep = fabsf(v - mid) / (span * 0.5f);
      if (sep < worst) worst = sep;
    }
    for (int c = 0; c < N6_TAG36H11_NCODES; c++) {
      const int d = popcount64(code ^ n6_tag36h11_codes[c]);
      if (d < best_h) {
        best_h = d; best_id = c; best_rot = rot; best_margin = worst;
        if (d == 0) break;
      }
    }
    if (best_h == 0) break;
  }

  if (best_id >= 0 && best_h < s_stats.best_hamming) s_stats.best_hamming = best_h;
  if (best_id < 0 || best_h > max_hamming) { s_stats.rej_code++; return -1; }
  *hamming_out = best_h;
  *rot_out = best_rot;
  *margin_out = (best_margin > 1.f) ? 1.f : best_margin;
  return best_id;
}

int n6_apriltag_find_gray(const uint8_t *gray, int w, int h,
                          const n6_apriltag_cfg_t *cfg,
                          n6_apriltag_t *out, int max_out)
{
  if (gray == NULL || cfg == NULL || out == NULL || max_out <= 0) return -1;
  if (w <= 0 || h <= 0 || w > MAXD || h > MAXD) return -1;

  threshold(gray, w, h, cfg->min_contrast);
  memset(s_seen, 0, (size_t)((w * h + 7) / 8));

  memset(&s_stats, 0, sizeof(s_stats));
  s_stats.best_hamming = 99;

  int found = 0;

  for (int i = 0; i < w * h && found < max_out; i++) {
    if (!bm_get(s_dark, i) || bm_get(s_seen, i)) continue;

    int nb = 0;
    const int npx = fill_blob(i, w, h, &nb);
    if (npx < MIN_BLOB_PX) continue;
    s_stats.blobs++;

    float q[4][2];
    if (!fit_quad(nb, q, cfg->min_side_px)) continue;
    s_stats.quads++;

    homog_t H;
    if (!homography(q, &H)) continue;

    float cells[WB][WB];
    for (int cy = 0; cy < WB; cy++) {
      for (int cx = 0; cx < WB; cx++) {
        cells[cy][cx] = cell_value(gray, w, h, &H, cx + 0.5f, cy + 0.5f);
      }
    }

    /* Black is the ring's mean, white the brightest payload cell. */
    float black = 0.f; int nblack = 0; float white = 0.f;
    for (int k = 0; k < WB; k++) {
      black += cells[0][k] + cells[WB - 1][k] + cells[k][0] + cells[k][WB - 1];
      nblack += 4;
    }
    black /= (float)nblack;
    for (int cy = 1; cy < WB - 1; cy++)
      for (int cx = 1; cx < WB - 1; cx++)
        if (cells[cy][cx] > white) white = cells[cy][cx];

    int hd = 0, rot = 0; float margin = 0.f;
    const int id = decode(cells, black, white, cfg->max_hamming, &hd, &rot, &margin);
    if (id < 0) continue;

    n6_apriltag_t *t = &out[found];
    t->id = id;
    t->hamming = hd;
    t->margin = margin;

    /* Corner 0 is the tag's own top-left. rot is subtracted: decode() applies
     * it from canonical to observed. */
    for (int k = 0; k < 4; k++) {
      const float *p = q[(k + 4 - rot) & 3];
      t->corners[k][0] = p[0] / (float)w;
      t->corners[k][1] = p[1] / (float)h;
    }

    float minx = q[0][0], maxx = q[0][0], miny = q[0][1], maxy = q[0][1];
    for (int k = 1; k < 4; k++) {
      if (q[k][0] < minx) minx = q[k][0];
      if (q[k][0] > maxx) maxx = q[k][0];
      if (q[k][1] < miny) miny = q[k][1];
      if (q[k][1] > maxy) maxy = q[k][1];
    }
    t->cx = ((minx + maxx) * 0.5f) / (float)w;
    t->cy = ((miny + maxy) * 0.5f) / (float)h;
    t->w  = (maxx - minx) / (float)w;
    t->h  = (maxy - miny) / (float)h;

    /* Angle of the top edge in pixels, negated because image y grows downward. */
    const float ex = (t->corners[1][0] - t->corners[0][0]) * (float)w;
    const float ey = (t->corners[1][1] - t->corners[0][1]) * (float)h;
    t->angle = -atan2f(ey, ex) * (180.0f / 3.14159265f);

    s_stats.decoded++;
    found++;
  }

  for (int a = 0; a < found - 1; a++)
    for (int b = a + 1; b < found; b++)
      if (out[b].margin > out[a].margin) {
        const n6_apriltag_t tmp = out[a]; out[a] = out[b]; out[b] = tmp;
      }

  return found;
}

int n6_apriltag_find_rgb888(const uint8_t *rgb, int w, int h,
                            uint8_t *gray_scratch,
                            const n6_apriltag_cfg_t *cfg,
                            n6_apriltag_t *out, int max_out)
{
  if (rgb == NULL || gray_scratch == NULL) return -1;
  const uint8_t *s = rgb;
  const int n = w * h;
  for (int i = 0; i < n; i++) {
    /* BT.601 luma. */
    gray_scratch[i] = (uint8_t)((s[0] * 77u + s[1] * 151u + s[2] * 28u) >> 8);
    s += 3;
  }
  return n6_apriltag_find_gray(gray_scratch, w, h, cfg, out, max_out);
}

void n6_apriltag_debug_dark(uint8_t *out, int w, int h)
{
  if (out == NULL || w <= 0 || h <= 0 || w > MAXD || h > MAXD) return;
  for (int i = 0; i < w * h; i++) out[i] = bm_get(s_dark, i) ? 255u : 0u;
}

void n6_apriltag_debug_cells(uint8_t *out64)
{
  if (out64 == NULL) return;
  for (int y = 0; y < WB; y++)
    for (int x = 0; x < WB; x++) out64[y * WB + x] = s_dbg_cells[y][x];
}
