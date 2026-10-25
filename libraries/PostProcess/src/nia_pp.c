/**
 * @file    nia_pp.c
 * @brief   Nia detector post-processing.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "nia_pp.h"

#include <math.h>

#if defined(__ARM_FEATURE_MVE) && ((__ARM_FEATURE_MVE) & 1)
#  include <arm_mve.h>
#  define NIA_PP_USE_MVE 1
#else
#  define NIA_PP_USE_MVE 0
#endif
#include <stdlib.h>
#include <string.h>

static const int   NIA_STRIDES[3] = { 8, 16, 32 };
static const int   NIA_GRID[3]    = { 32, 16, 8 };
/* Trained per-level B-Reg scales, specific to one model. */
static const float NIA_SCALES[3]  = { 13.60580825805664f,
                                      11.974827766418457f,
                                      8.355365753173828f };

typedef struct {
  float x1, y1, x2, y2;   /* corners in 256-px NN space */
  float score;
  int   label;
} nia_cand_t;

__attribute__((section(".ext_psram"))) static nia_cand_t s_cand[NIA_PP_MAX_CAND];

/* Maximum raw int8 logit over classes and its class, per cell y*G + x. In
 * AXISRAM, not .ext_psram: the gate reads each cell up to nine times. */
static int16_t s_objq[NIA_PP_MAX_GRID_CELLS];
static uint8_t s_objc[NIA_PP_MAX_GRID_CELLS];

static int8_t  s_best8[NIA_PP_MAX_GRID_CELLS];
static uint8_t s_lbl8[NIA_PP_MAX_GRID_CELLS];

static inline float nia_sigmoidf(float v) { return 1.0f / (1.0f + expf(-v)); }

static int nia_cand_cmp_desc(const void *pa, const void *pb)
{
  const nia_cand_t *a = (const nia_cand_t *)pa;
  const nia_cand_t *b = (const nia_cand_t *)pb;
  if (a->score < b->score) return  1;
  if (a->score > b->score) return -1;
  return 0;
}

int32_t nia_pp_process(const LL_Buffer_InfoTypeDef *outs,
                       nia_pp_out_t *out,
                       const nia_pp_params_t *params)
{
  if (!outs || !out || !out->pOutBuff || !params) return -1;

  /* Six int8 outputs with mem_shape {1,C,H,W}. C == NB_CLASSES is cls, C == 4
   * is reg, and H of 32, 16 or 8 the level. */
  const int8_t *cls_ptr[3] = { 0, 0, 0 };
  const int8_t *reg_ptr[3] = { 0, 0, 0 };
  float cls_scale[3] = { 0 }, reg_scale[3] = { 0 };
  int   cls_zp[3]    = { 0 }, reg_zp[3]    = { 0 };

  for (int i = 0; outs[i].name != NULL; i++) {
    const LL_Buffer_InfoTypeDef *b = &outs[i];
    if (!b->mem_shape || b->mem_ndims < 4) continue;
    const uint32_t C = b->mem_shape[1];
    const uint32_t G = b->mem_shape[2];
    const int lvl = (G == 32u) ? 0 : (G == 16u) ? 1 : (G == 8u) ? 2 : -1;
    if (lvl < 0) continue;

    const int8_t *p  = (const int8_t *)LL_Buffer_addr_start(b);
    const float   sc = b->scale  ? b->scale[0]        : 1.0f;
    const int     zp = b->offset ? (int)b->offset[0]  : 0;

    if (C == (uint32_t)NIA_PP_NB_CLASSES) {
      cls_ptr[lvl] = p; cls_scale[lvl] = sc; cls_zp[lvl] = zp;
    } else if (C == 4u) {
      reg_ptr[lvl] = p; reg_scale[lvl] = sc; reg_zp[lvl] = zp;
    }
  }

  const float conf_thr = params->conf_threshold;
  int ncand = 0;

  for (int lvl = 0; lvl < 3; lvl++) {
    if (!cls_ptr[lvl] || !reg_ptr[lvl]) continue;

    const int     s   = NIA_STRIDES[lvl];
    const int     G   = NIA_GRID[lvl];
    const float   sca = NIA_SCALES[lvl];
    const int     HW  = G * G;
    const int8_t *cls = cls_ptr[lvl];
    const int8_t *reg = reg_ptr[lvl];
    const float   cs  = cls_scale[lvl]; const int cz = cls_zp[lvl];
    const float   rs  = reg_scale[lvl]; const int rz = reg_zp[lvl];

    /* Objectness is the maximum over classes of the raw int8 logit
     * cls[c*HW + cell], dequant being monotonic. Ties keep the lowest class. */
    memcpy(s_best8, cls, (size_t)HW);        /* class 0 row */
    memset(s_lbl8, 0, (size_t)HW);

    for (int c = 1; c < NIA_PP_NB_CLASSES; c++) {
      const int8_t *row = cls + (size_t)c * (size_t)HW;
      int cell = 0;
#if NIA_PP_USE_MVE
      {
        const int8x16_t vc = vdupq_n_s8((int8_t)c);
        for (; cell + 16 <= HW; cell += 16) {
          const int8x16_t v = vld1q_s8(&row[cell]);
          const int8x16_t b = vld1q_s8(&s_best8[cell]);
          const mve_pred16_t p = vcmpgtq_s8(v, b);
          vst1q_s8(&s_best8[cell], vpselq_s8(v, b, p));
          const int8x16_t l = vld1q_s8((const int8_t *)&s_lbl8[cell]);
          vst1q_s8((int8_t *)&s_lbl8[cell], vpselq_s8(vc, l, p));
        }
      }
#endif
      for (; cell < HW; cell++) {
        const int q = (int)row[cell];
        if (q > (int)s_best8[cell]) {
          s_best8[cell] = (int8_t)q;
          s_lbl8[cell]  = (uint8_t)c;
        }
      }
    }
    for (int cell = 0; cell < HW; cell++) {
      s_objq[cell] = (int16_t)s_best8[cell];
      s_objc[cell] = s_lbl8[cell];
    }

    /* EQ-gate: a cell passes when its objectness equals the maximum of its
     * KxK window, clamped to the grid. Tied cells all pass. */
    const int half = NIA_PP_GATE_KERNEL / 2;
    for (int y = 0; y < G; y++) {
      for (int x = 0; x < G; x++) {
        const int cell = y * G + x;
        const int q    = (int)s_objq[cell];

        int is_local_max = 1;
        for (int dy = -half; dy <= half && is_local_max; dy++) {
          const int ny = y + dy;
          if (ny < 0 || ny >= G) continue;
          for (int dx = -half; dx <= half; dx++) {
            const int nx = x + dx;
            if (nx < 0 || nx >= G) continue;
            if ((int)s_objq[ny * G + nx] > q) { is_local_max = 0; break; }
          }
        }
        if (!is_local_max) continue;

        const float conf = nia_sigmoidf(((float)q - (float)cz) * cs);
        if (conf <= conf_thr) continue;
        if (ncand >= NIA_PP_MAX_CAND) continue;

        /* B-Reg: dist[l,t,r,b] = sigmoid(reg) * SCALES[lvl] * stride. */
        const float dl = nia_sigmoidf(((float)reg[0 * HW + cell] - (float)rz) * rs) * sca * (float)s;
        const float dt = nia_sigmoidf(((float)reg[1 * HW + cell] - (float)rz) * rs) * sca * (float)s;
        const float dr = nia_sigmoidf(((float)reg[2 * HW + cell] - (float)rz) * rs) * sca * (float)s;
        const float db = nia_sigmoidf(((float)reg[3 * HW + cell] - (float)rz) * rs) * sca * (float)s;

        const float cx = ((float)x + 0.5f) * (float)s;
        const float cy = ((float)y + 0.5f) * (float)s;

        float x1 = cx - dl, y1 = cy - dt, x2 = cx + dr, y2 = cy + db;
        if (x1 < 0.f)               x1 = 0.f;
        if (y1 < 0.f)               y1 = 0.f;
        if (x2 > NIA_PP_INPUT_DIM)  x2 = NIA_PP_INPUT_DIM;
        if (y2 > NIA_PP_INPUT_DIM)  y2 = NIA_PP_INPUT_DIM;

        nia_cand_t *cd = &s_cand[ncand++];
        cd->x1 = x1; cd->y1 = y1; cd->x2 = x2; cd->y2 = y2;
        cd->score = conf; cd->label = (int)s_objc[cell];
      }
    }
  }

  if (ncand > 1) qsort(s_cand, (size_t)ncand, sizeof(nia_cand_t), nia_cand_cmp_desc);

  int max_det = params->max_det;
  if (max_det < 0)              max_det = 0;
  if (max_det > NIA_PP_MAX_DET) max_det = NIA_PP_MAX_DET;

  int kept = (ncand < max_det) ? ncand : max_det;
  for (int i = 0; i < kept; i++) {
    const nia_cand_t *a = &s_cand[i];
    nia_det_t *d = &out->pOutBuff[i];
    d->x_center    = ((a->x1 + a->x2) * 0.5f) / NIA_PP_INPUT_DIM;
    d->y_center    = ((a->y1 + a->y2) * 0.5f) / NIA_PP_INPUT_DIM;
    d->width       = (a->x2 - a->x1)          / NIA_PP_INPUT_DIM;
    d->height      = (a->y2 - a->y1)          / NIA_PP_INPUT_DIM;
    d->conf        = a->score;
    d->class_index = a->label;
  }

  out->nb_detect = kept;
  return 0;
}
