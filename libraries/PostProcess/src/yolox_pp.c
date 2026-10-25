/**
 * @file    yolox_pp.c
 * @brief   Post-processing for PixelKit-trained YOLOX detectors.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "yolox_pp.h"

#include <math.h>
#include <string.h>

typedef struct {
  float   cx, cy, w, h, score;
  int32_t cls;
} cand_t;
static cand_t s_cand[YOLOX_PP_MAX_CAND] __attribute__((section(".ext_psram")));

static float iou_cxcywh(const cand_t *a, const cand_t *b)
{
  const float ax0 = a->cx - a->w * 0.5f, ax1 = a->cx + a->w * 0.5f;
  const float ay0 = a->cy - a->h * 0.5f, ay1 = a->cy + a->h * 0.5f;
  const float bx0 = b->cx - b->w * 0.5f, bx1 = b->cx + b->w * 0.5f;
  const float by0 = b->cy - b->h * 0.5f, by1 = b->cy + b->h * 0.5f;
  const float ix0 = ax0 > bx0 ? ax0 : bx0;
  const float iy0 = ay0 > by0 ? ay0 : by0;
  const float ix1 = ax1 < bx1 ? ax1 : bx1;
  const float iy1 = ay1 < by1 ? ay1 : by1;
  const float iw = ix1 - ix0, ih = iy1 - iy0;
  if (iw <= 0.f || ih <= 0.f) return 0.f;
  const float inter = iw * ih;
  const float uni = a->w * a->h + b->w * b->h - inter;
  return uni > 0.f ? inter / uni : 0.f;
}

int32_t yolox_pp_process(const LL_Buffer_InfoTypeDef *outs,
                         yolox_pp_out_t *out,
                         const yolox_pp_params_t *params)
{
  if (!outs || !out || !out->pOutBuff || !params) return -1;
  const int32_t nc = params->nb_classes;
  if (nc <= 0 || params->input_dim <= 0.f) return -1;
  const uint32_t row_len = (uint32_t)(5 + nc);

  const uint32_t dim = (uint32_t)(params->input_dim + 0.5f);
  const uint32_t exp_rows =
      (dim / 8u) * (dim / 8u) + (dim / 16u) * (dim / 16u) + (dim / 32u) * (dim / 32u);

  /* The output is {1, N, 5+nc}, matched by shape. */
  const LL_Buffer_InfoTypeDef *buf = NULL;
  uint32_t rows = 0;
  bool shape_seen = false;
  for (int i = 0; outs[i].name != NULL; i++) {
    const LL_Buffer_InfoTypeDef *b = &outs[i];
    if (!b->mem_shape || b->mem_ndims < 2) continue;
    uint32_t total = 1;
    for (int d = 0; d < b->mem_ndims; d++) total *= (uint32_t)b->mem_shape[d];
    if (total == 0u) continue;
    shape_seen = true;
    const uint32_t last = (uint32_t)b->mem_shape[b->mem_ndims - 1];
    if (last == row_len && total == exp_rows * row_len) { buf = b; rows = exp_rows; break; }
  }
  if (!buf) return shape_seen ? -3 : -2;   /* -3: model and labels.h disagree */

  /* A buffer with a scale is int8, one without is float32. */
  const bool    q     = (buf->scale != NULL);
  const float   sc    = q ? buf->scale[0] : 1.0f;
  const int     zp    = (q && buf->offset) ? (int)buf->offset[0] : 0;
  const int8_t *p8    = (const int8_t *)LL_Buffer_addr_start(buf);
  const float  *pf    = (const float *)LL_Buffer_addr_start(buf);

  const float conf_thr = params->conf_threshold;
  const float inv_dim  = 1.0f / params->input_dim;
  int ncand = 0;
  int min_i = 0;

  /* Rows are ordered stride 8, then 16, then 32. */
  static const int kStrides[3] = { 8, 16, 32 };
  uint32_t r = 0;
  for (int lvl = 0; lvl < 3 && r < rows; lvl++) {
    const int   st  = kStrides[lvl];
    const int   g   = (int)(dim / (uint32_t)st);
    const float stf = (float)st;
    for (int gy = 0; gy < g && r < rows; gy++) {
      for (int gx = 0; gx < g && r < rows; gx++, r++) {
        float row[5];
        float best_cls = -1.f;
        int32_t best_idx = 0;
        if (q) {
          const int8_t *rp = p8 + (size_t)r * row_len;
          for (int k = 0; k < 5; k++) row[k] = sc * (float)((int)rp[k] - zp);
          for (int32_t c = 0; c < nc; c++) {
            const float v = sc * (float)((int)rp[5 + c] - zp);
            if (v > best_cls) { best_cls = v; best_idx = c; }
          }
        } else {
          const float *rp = pf + (size_t)r * row_len;
          memcpy(row, rp, sizeof row);
          for (int32_t c = 0; c < nc; c++) {
            const float v = rp[5 + c];
            if (v > best_cls) { best_cls = v; best_idx = c; }
          }
        }
        /* obj and cls are already sigmoided. */
        const float score = row[4] * best_cls;
        if (!(score >= conf_thr)) continue;       /* also rejects NaN */

        /* The exponent is clamped so a corrupt row cannot produce inf. */
        float ew = row[2], eh = row[3];
        if (ew > 16.f) ew = 16.f; else if (ew < -16.f) ew = -16.f;
        if (eh > 16.f) eh = 16.f; else if (eh < -16.f) eh = -16.f;
        cand_t c;
        c.cx    = (row[0] + (float)gx) * stf;
        c.cy    = (row[1] + (float)gy) * stf;
        c.w     = expf(ew) * stf;
        c.h     = expf(eh) * stf;
        c.score = score;
        c.cls   = best_idx;
        if (!(c.w > 0.f && c.h > 0.f)) continue;

        if (ncand < YOLOX_PP_MAX_CAND) {
          s_cand[ncand++] = c;
        } else {
          float mn = s_cand[0].score; min_i = 0;
          for (int i = 1; i < YOLOX_PP_MAX_CAND; i++)
            if (s_cand[i].score < mn) { mn = s_cand[i].score; min_i = i; }
          if (score > mn) s_cand[min_i] = c;
        }
      }
    }
  }

  for (int i = 1; i < ncand; i++) {
    cand_t key = s_cand[i];
    int j = i - 1;
    while (j >= 0 && s_cand[j].score < key.score) { s_cand[j + 1] = s_cand[j]; j--; }
    s_cand[j + 1] = key;
  }

  const float iou_thr = params->iou_threshold;
  int32_t kept = 0;
  for (int i = 0; i < ncand && kept < params->max_det; i++) {
    bool drop = false;
    for (int32_t k = 0; k < kept && !drop; k++) {
      const yolox_det_t *d = &out->pOutBuff[k];
      if (d->class_index != s_cand[i].cls) continue;
      cand_t kd = { d->x_center * params->input_dim, d->y_center * params->input_dim,
                    d->width * params->input_dim, d->height * params->input_dim, 0.f, 0 };
      if (iou_cxcywh(&kd, &s_cand[i]) > iou_thr) drop = true;
    }
    if (drop) continue;
    yolox_det_t *d = &out->pOutBuff[kept++];
    d->x_center    = s_cand[i].cx * inv_dim;
    d->y_center    = s_cand[i].cy * inv_dim;
    d->width       = s_cand[i].w * inv_dim;
    d->height      = s_cand[i].h * inv_dim;
    d->conf        = s_cand[i].score;
    d->class_index = s_cand[i].cls;
  }
  out->nb_detect = kept;
  return 0;
}
