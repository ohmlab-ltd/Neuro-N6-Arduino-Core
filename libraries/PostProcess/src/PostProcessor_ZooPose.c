/**
 ******************************************************************************
 * @file    PostProcessor_ZooPose.c
 * @brief   MoveNet pose post-processor of PostProcessor_Zoo.h.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "PostProcessor_Zoo.h"
#include "zoo_pp.h"

#include "spe_movenet_pp_if.h"

#include <string.h>

#define ZOO_KPS 17

typedef struct {
  zoo_common_t      c;
  od_pp_outBuffer_t det[1];
  zoo_kp_t          kp[ZOO_KPS];
} zoo_pose_state_t;

static zoo_pose_state_t g_mn = {
  .c = { .magic = ZOO_PP_MAGIC, .name = "MoveNet", .conf = 0.3f,
         .slot = VND_META_SLOT_MODEL_0, .nb_classes = 1, .status = ZOO_PP_OK },
};

/* Keypoints at or above the threshold needed to report a person. */
#define ZOO_POSE_MIN_KPS 3

static int32_t mn_run(void *ctx, NN_Instance_TypeDef *model)
{
  zoo_pose_state_t *s = (zoo_pose_state_t *)ctx;
  zoo_common_t *c = &s->c;
  c->n = 0;
  zoo_scratch_reset();

  zoo_tensor_t t[ZOO_MAX_OUTPUTS];
  const int32_t nt = zoo_outputs(model, t, ZOO_MAX_OUTPUTS);
  const zoo_tensor_t *o = NULL;
  for (int32_t i = 0; i < nt; i++) {
    if (t[i].ndims == 3u && (o == NULL || t[i].count > o->count)) o = &t[i];
  }
  if (o == NULL) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected one output of heatmaps");
  if (o->type != DataType_INT8 && o->type != DataType_FLOAT) {
    return zoo_fail(c, ZOO_PP_ERR_TYPE, "expected an int8 or float output");
  }

  /* ST's decoder reads [H][W][keypoints]. */
  const uint32_t a = o->dims[0], b = o->dims[1], d = o->dims[2];
  uint32_t hh, ww;
  const void *hm = o->data;
  if (a == b && d == ZOO_KPS) {
    hh = a; ww = b;
  } else if (b == d && a == ZOO_KPS) {
    hh = b; ww = d;
    void *tr = zoo_scratch_alloc(o->count * zoo_esize(o));
    if (tr == NULL) return zoo_fail(c, ZOO_PP_ERR_SCRATCH, "output too large to reorder");
    zoo_chw_to_hwc(o->data, tr, ZOO_KPS, hh, ww, zoo_esize(o));
    hm = tr;
  } else {
    return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected 17 square heatmaps");
  }

  spe_movenet_pp_static_param_t p;
  memset(&p, 0, sizeof(p));
  p.heatmap_width  = ww;
  p.heatmap_height = hh;
  p.nb_keypoints   = ZOO_KPS;
  p.raw_scale      = o->scale;
  p.raw_zero_point = (int8_t)o->zp;
  spe_pp_outBuffer_t kp[ZOO_KPS];
  spe_pp_out_t out = { kp };
  spe_movenet_pp_in_t in = { (void *)hm };
  (void)spe_movenet_pp_reset(&p);
  const int32_t err = (o->type == DataType_INT8) ? spe_movenet_pp_process_int8(&in, &out, &p)
                                                 : spe_movenet_pp_process(&in, &out, &p);
  if (err != AI_SPE_POSTPROCESS_ERROR_NO) return zoo_fail(c, ZOO_PP_ERR_DECODE, "decoder error");

  float32_t x0 = 1.f, y0 = 1.f, x1 = 0.f, y1 = 0.f, sum = 0.f;
  int32_t good = 0;
  for (uint32_t i = 0; i < ZOO_KPS; i++) {
    s->kp[i].x    = kp[i].x_center;
    s->kp[i].y    = kp[i].y_center;
    s->kp[i].conf = kp[i].proba;
    if (kp[i].proba < c->conf) continue;
    if (kp[i].x_center < x0) x0 = kp[i].x_center;
    if (kp[i].x_center > x1) x1 = kp[i].x_center;
    if (kp[i].y_center < y0) y0 = kp[i].y_center;
    if (kp[i].y_center > y1) y1 = kp[i].y_center;
    sum += kp[i].proba;
    good++;
  }
  if (good >= ZOO_POSE_MIN_KPS) {
    const float32_t px = 0.1f * (x1 - x0) + 0.02f, py = 0.1f * (y1 - y0) + 0.02f;
    x0 = (x0 - px < 0.f) ? 0.f : x0 - px;
    y0 = (y0 - py < 0.f) ? 0.f : y0 - py;
    x1 = (x1 + px > 1.f) ? 1.f : x1 + px;
    y1 = (y1 + py > 1.f) ? 1.f : y1 + py;
    s->det[0].x_center    = 0.5f * (x0 + x1);
    s->det[0].y_center    = 0.5f * (y0 + y1);
    s->det[0].width       = x1 - x0;
    s->det[0].height      = y1 - y0;
    s->det[0].conf        = sum / (float32_t)good;
    s->det[0].class_index = 0;
    c->n = 1;
  }
  return zoo_ok(c);
}

static void mn_draw(void *ctx, OVL_Mode_t mode)
{
  const zoo_pose_state_t *s = (const zoo_pose_state_t *)ctx;
  zoo_lcd_coco_pose(s->det, s->kp, s->c.n, s->c.conf, mode);
}

static void mn_emit(void *ctx)
{
  const zoo_pose_state_t *s = (const zoo_pose_state_t *)ctx;
  zoo_meta_coco_pose(s->det, s->kp, s->c.n, s->c.conf, s->c.slot);
}

static IPostProcessor g_mn_pp = {
  .init = zoo_det_init, .run = mn_run, .draw_lcd = mn_draw,
  .emit_metadata = mn_emit, .ctx = &g_mn,
};

IPostProcessor *PostProcess_MoveNet(float conf)
{
  return PostProcess_MoveNet_Ex(conf, VND_META_SLOT_MODEL_0);
}

IPostProcessor *PostProcess_MoveNet_Ex(float conf, vnd_meta_slot_t slot)
{
  static const char *const k_person[] = { "person" };
  g_mn.c.conf    = conf;
  g_mn.c.slot    = slot;
  g_mn.c.labels  = k_person;
  g_mn.c.nlabels = 1;
  g_mn.c.det     = g_mn.det;
  return &g_mn_pp;
}
