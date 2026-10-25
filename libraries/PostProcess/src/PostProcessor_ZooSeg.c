/**
 ******************************************************************************
 * @file    PostProcessor_ZooSeg.c
 * @brief   Semantic segmentation post-processor of PostProcessor_Zoo.h.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "PostProcessor_Zoo.h"
#include "zoo_pp.h"

#include <string.h>

/* Declared here because vision_models_pp.h redeclares qsort() with a clashing
 * type. Each finds the largest of len_arr values for up to 16 items: item k
 * reads arr[k * offset + i] in maxi_p, arr[k + i * offset] in maxi_tr_p. */
void vision_models_maxi_p_is8ou8(int8_t *arr, uint32_t len_arr, uint32_t offset,
                                 int8_t *maxim, uint8_t *index, uint32_t parallelize);
void vision_models_maxi_tr_p_is8ou8(int8_t *arr, uint32_t len_arr, uint32_t offset,
                                    int8_t *maxim, uint8_t *index, uint32_t parallelize);

/* Largest label map, in pixels. */
#define ZOO_SEG_MAX_PIXELS (512u * 512u)

/* A class under 1/256 of the picture is not reported. */
#define ZOO_SEG_MIN_SHARE_SHIFT 8u

typedef struct {
  zoo_common_t      c;
  od_pp_outBuffer_t det[ZOO_PP_MAX_DET];
  zoo_region_t      reg[ZOO_PP_MAX_DET];
  uint32_t          w, h;
} zoo_seg_state_t;

static zoo_seg_state_t g_seg = {
  .c = { .magic = ZOO_PP_MAGIC, .name = "DeepLabV3",
         .slot = VND_META_SLOT_MODEL_0, .status = ZOO_PP_OK },
};

/* The label map stays valid after run() for draw_lcd and emit_metadata. */
__attribute__((section(".ext_psram")))
static uint8_t s_map[ZOO_SEG_MAX_PIXELS];

typedef struct {
  uint32_t count;
  uint16_t x0, y0, x1, y1;
} zoo_class_stats_t;

static zoo_class_stats_t s_stats[256];

static void argmax_int8(const int8_t *src, uint32_t ncls, uint32_t npix, bool chw)
{
  int8_t best[16];
  for (uint32_t p = 0; p < npix; p += 16u) {
    const uint32_t left = npix - p;
    const uint32_t n = (left < 16u) ? left : 16u;
    if (chw) {
      vision_models_maxi_tr_p_is8ou8((int8_t *)&src[p], ncls, npix, best, &s_map[p], n);
    } else {
      vision_models_maxi_p_is8ou8((int8_t *)&src[p * ncls], ncls, ncls, best, &s_map[p], n);
    }
  }
}

static void argmax_any(const zoo_tensor_t *t, uint32_t ncls, uint32_t npix, bool chw)
{
  for (uint32_t p = 0; p < npix; p++) {
    uint32_t best = 0;
    float32_t vb = zoo_value(t, chw ? p : p * ncls);
    for (uint32_t k = 1; k < ncls; k++) {
      const float32_t v = zoo_value(t, chw ? (k * npix + p) : (p * ncls + k));
      if (v > vb) { vb = v; best = k; }
    }
    s_map[p] = (uint8_t)best;
  }
}

static int32_t seg_run(void *ctx, NN_Instance_TypeDef *model)
{
  zoo_seg_state_t *s = (zoo_seg_state_t *)ctx;
  zoo_common_t *c = &s->c;
  c->n = 0;
  s->w = s->h = 0u;

  zoo_tensor_t t[ZOO_MAX_OUTPUTS];
  const int32_t nt = zoo_outputs(model, t, ZOO_MAX_OUTPUTS);
  const zoo_tensor_t *o = NULL;
  for (int32_t i = 0; i < nt; i++) {
    if (t[i].ndims == 3u && (o == NULL || t[i].count > o->count)) o = &t[i];
  }
  if (o == NULL) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected one output of per-pixel class scores");

  /* The class axis is the short one. */
  const uint32_t a = o->dims[0], b = o->dims[1], d = o->dims[2];
  bool chw;
  uint32_t ncls, h, w;
  if (a < b && a < d)      { chw = true;  ncls = a; h = b; w = d; }
  else if (d < a && d < b) { chw = false; ncls = d; h = a; w = b; }
  else return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "cannot tell the class axis from the output shape");
  if (ncls < 2u || ncls > 255u) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected 2 to 255 classes");
  if (w * h > ZOO_SEG_MAX_PIXELS) return zoo_fail(c, ZOO_PP_ERR_SCRATCH, "label map larger than 512x512");
  const uint32_t npix = w * h;

  if (o->type == DataType_INT8) argmax_int8((const int8_t *)o->data, ncls, npix, chw);
  else                          argmax_any(o, ncls, npix, chw);

  for (uint32_t k = 0; k < ncls; k++) {
    s_stats[k].count = 0u;
    s_stats[k].x0 = (uint16_t)w;
    s_stats[k].y0 = (uint16_t)h;
    s_stats[k].x1 = 0u;
    s_stats[k].y1 = 0u;
  }
  for (uint32_t y = 0; y < h; y++) {
    const uint8_t *row = &s_map[y * w];
    for (uint32_t x = 0; x < w; x++) {
      zoo_class_stats_t *st = &s_stats[row[x]];
      st->count++;
      if (x < st->x0) st->x0 = (uint16_t)x;
      if (x >= st->x1) st->x1 = (uint16_t)(x + 1u);
      if (y < st->y0) st->y0 = (uint16_t)y;
      if (y >= st->y1) st->y1 = (uint16_t)(y + 1u);
    }
  }

  int32_t n = 0;
  const uint32_t min_count = (npix >> ZOO_SEG_MIN_SHARE_SHIFT) + 1u;
  for (uint32_t k = 1; k < ncls && n < ZOO_PP_MAX_DET; k++) {
    const zoo_class_stats_t *st = &s_stats[k];
    if (st->count < min_count) continue;
    zoo_region_t *r = &s->reg[n];
    r->x0 = st->x0; r->y0 = st->y0; r->x1 = st->x1; r->y1 = st->y1;
    r->box.x_center    = 0.5f * (float32_t)(st->x0 + st->x1) / (float32_t)w;
    r->box.y_center    = 0.5f * (float32_t)(st->y0 + st->y1) / (float32_t)h;
    r->box.width       = (float32_t)(st->x1 - st->x0) / (float32_t)w;
    r->box.height      = (float32_t)(st->y1 - st->y0) / (float32_t)h;
    r->box.conf        = (float32_t)st->count / (float32_t)npix;
    r->box.class_index = (int32_t)k;
    for (int32_t j = n; j > 0 && s->reg[j - 1].box.conf < s->reg[j].box.conf; j--) {
      const zoo_region_t tmp = s->reg[j - 1];
      s->reg[j - 1] = s->reg[j];
      s->reg[j] = tmp;
    }
    n++;
  }
  for (int32_t i = 0; i < n; i++) s->det[i] = s->reg[i].box;

  s->w = w;
  s->h = h;
  c->nb_classes = (int32_t)ncls;
  c->n = n;
  return zoo_ok(c);
}

static void seg_draw(void *ctx, OVL_Mode_t mode)
{
  const zoo_seg_state_t *s = (const zoo_seg_state_t *)ctx;
  zoo_lcd_regions(s->reg, s->c.n, (s->w != 0u) ? s_map : NULL, s->w, s->h,
                  s->c.labels, s->c.nlabels, s->c.nb_classes, mode);
}

static void seg_emit(void *ctx)
{
  const zoo_seg_state_t *s = (const zoo_seg_state_t *)ctx;
  zoo_meta_regions(s->reg, s->c.n, (s->w != 0u) ? s_map : NULL, s->w, s->h,
                   s->c.labels, s->c.nlabels, s->c.nb_classes, s->c.slot);
}

static IPostProcessor g_seg_pp = {
  .init = zoo_det_init, .run = seg_run, .draw_lcd = seg_draw,
  .emit_metadata = seg_emit, .ctx = &g_seg,
};

IPostProcessor *PostProcess_DeepLabV3(void)
{
  return PostProcess_DeepLabV3_Ex(NULL, 0, VND_META_SLOT_MODEL_0);
}

IPostProcessor *PostProcess_DeepLabV3_Ex(const char *const *labels,
                                         int nlabels, vnd_meta_slot_t slot)
{
  g_seg.c.labels  = labels;
  g_seg.c.nlabels = (labels != NULL) ? nlabels : 0;
  g_seg.c.slot    = slot;
  g_seg.c.det     = g_seg.det;
  return &g_seg_pp;
}
