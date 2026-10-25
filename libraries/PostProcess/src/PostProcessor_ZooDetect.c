/**
 ******************************************************************************
 * @file    PostProcessor_ZooDetect.c
 * @brief   Object detector post-processors of PostProcessor_Zoo.h.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "PostProcessor_Zoo.h"
#include "zoo_pp.h"

#include "od_yolov8_pp_if.h"
#include "od_st_yolox_pp_if.h"
#include "od_yolov2_pp_if.h"
#include "od_fd_blazeface_pp_if.h"
#include "yolox_pp.h"

#include <math.h>
#include <string.h>

typedef struct {
  zoo_common_t      c;
  od_pp_outBuffer_t det[ZOO_PP_MAX_DET];
} zoo_det_state_t;

#define ZOO_DET_STATE(nm, cf, io) \
  { .c = { .magic = ZOO_PP_MAGIC, .name = nm, .conf = cf, .iou = io, \
           .slot = VND_META_SLOT_MODEL_0, .status = ZOO_PP_OK } }

static void det_configure(zoo_det_state_t *s, float conf, float iou,
                          const char *const *labels, int nlabels,
                          vnd_meta_slot_t slot)
{
  s->c.conf    = conf;
  s->c.iou     = iou;
  s->c.labels  = labels;
  s->c.nlabels = (labels != NULL) ? nlabels : 0;
  s->c.slot    = slot;
  s->c.det     = s->det;
}

/* The largest output with ndims dimensions, or NULL. */
static const zoo_tensor_t *largest(const zoo_tensor_t *t, int32_t nt, uint32_t ndims)
{
  const zoo_tensor_t *o = NULL;
  for (int32_t i = 0; i < nt; i++) {
    if (t[i].ndims == ndims && (o == NULL || t[i].count > o->count)) o = &t[i];
  }
  return o;
}

/* A square map in channel-last order: the tensor itself, or a reordered copy
 * in scratch. NULL if scratch is short. */
static const void *as_hwc(const zoo_tensor_t *t, bool chw, uint32_t *g, uint32_t *ch)
{
  if (!chw) {
    *g = t->dims[0];
    *ch = t->dims[2];
    return t->data;
  }
  *ch = t->dims[0];
  *g = t->dims[1];
  void *dst = zoo_scratch_alloc(t->count * zoo_esize(t));
  if (dst != NULL) zoo_chw_to_hwc(t->data, dst, *ch, *g, *g, zoo_esize(t));
  return dst;
}

/* YOLOv8-style head: [4 + classes][boxes] */

typedef struct {
  zoo_det_state_t s;
  bool            pixels;   /* the model gives boxes in input pixels */
} zoo_y8_state_t;

static zoo_y8_state_t g_y8 = { .s = ZOO_DET_STATE("YOLOv8Detect", 0.25f, 0.5f) };

static int32_t y8_init(void *ctx, NN_Instance_TypeDef *model)
{
  ((zoo_y8_state_t *)ctx)->pixels = false;
  return zoo_det_init(ctx, model);
}

static int32_t y8_run(void *ctx, NN_Instance_TypeDef *model)
{
  zoo_y8_state_t *y = (zoo_y8_state_t *)ctx;
  zoo_common_t *c = &y->s.c;
  c->n = 0;
  zoo_scratch_reset();

  zoo_tensor_t t[ZOO_MAX_OUTPUTS];
  const int32_t nt = zoo_outputs(model, t, ZOO_MAX_OUTPUTS);
  const zoo_tensor_t *o = largest(t, nt, 2u);
  if (o == NULL) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected one output of [4 + classes][boxes]");
  if (o->type == DataType_UINT8) return zoo_fail(c, ZOO_PP_ERR_TYPE, "uint8 outputs are not supported");

  /* The box count is sum((size / stride)^2) over strides 8, 16 and 32. */
  uint32_t w = 0, h = 0;
  const bool have_in = zoo_input_size(model, &w, &h);
  const uint32_t nexp = have_in ? ((w / 8u) * (h / 8u) + (w / 16u) * (h / 16u) + (w / 32u) * (h / 32u)) : 0u;
  const uint32_t a = o->dims[0], b = o->dims[1];
  bool rows_are_boxes;
  if (b == nexp)      rows_are_boxes = false;
  else if (a == nexp) rows_are_boxes = true;
  else                rows_are_boxes = (a > b);
  const uint32_t nb   = rows_are_boxes ? a : b;
  const uint32_t vals = rows_are_boxes ? b : a;
  if (vals < 5u) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected 4 box values and at least one class");
  const int32_t nc = (int32_t)vals - 4;

  /* ST's decoder reads [4 + classes][boxes]. */
  const void *raw = o->data;
  if (rows_are_boxes) {
    void *tr = zoo_scratch_alloc(o->count * zoo_esize(o));
    if (tr == NULL) return zoo_fail(c, ZOO_PP_ERR_SCRATCH, "output too large to reorder");
    zoo_chw_to_hwc(o->data, tr, nb, 1u, vals, zoo_esize(o));
    raw = tr;
  }

  od_pp_outBuffer_t *cand = (od_pp_outBuffer_t *)zoo_scratch_alloc(nb * sizeof(od_pp_outBuffer_t));
  if (cand == NULL) return zoo_fail(c, ZOO_PP_ERR_SCRATCH, "too many boxes for the scratch memory");

  od_yolov8_pp_static_param_t p;
  memset(&p, 0, sizeof(p));
  p.nb_classes            = nc;
  p.nb_total_boxes        = (int32_t)nb;
  p.max_boxes_limit       = ZOO_PP_MAX_DET;
  p.conf_threshold        = c->conf;
  p.iou_threshold         = c->iou;
  p.raw_output_scale      = o->scale;
  p.raw_output_zero_point = (int8_t)o->zp;
  p.pScratchBuff          = NULL;   /* candidates as floats, in cand */
  od_yolov8_pp_in_centroid_t in = { (void *)raw };
  od_pp_out_t out = { cand, 0 };
  (void)od_yolov8_pp_reset(&p);
  const int32_t err = (o->type == DataType_INT8) ? od_yolov8_pp_process_int8(&in, &out, &p)
                                                 : od_yolov8_pp_process(&in, &out, &p);
  if (err != AI_OD_POSTPROCESS_ERROR_NO) return zoo_fail(c, ZOO_PP_ERR_DECODE, "decoder error");

  /* A value above 1.5 means the boxes are in input pixels, not normalised. */
  for (int32_t i = 0; i < out.nb_detect && !y->pixels; i++) {
    if (cand[i].x_center > 1.5f || cand[i].y_center > 1.5f ||
        cand[i].width > 1.5f || cand[i].height > 1.5f) {
      y->pixels = true;
    }
  }
  if (y->pixels && have_in) {
    const float32_t sx = 1.0f / (float32_t)w, sy = 1.0f / (float32_t)h;
    for (int32_t i = 0; i < out.nb_detect; i++) {
      cand[i].x_center *= sx;
      cand[i].width    *= sx;
      cand[i].y_center *= sy;
      cand[i].height   *= sy;
    }
  }

  c->nb_classes = nc;
  zoo_take(c, cand, out.nb_detect);
  return zoo_ok(c);
}

static IPostProcessor g_y8_pp = {
  .init = y8_init, .run = y8_run, .draw_lcd = zoo_det_draw,
  .emit_metadata = zoo_det_emit, .ctx = &g_y8,
};

IPostProcessor *PostProcess_YOLOv8Detect(float conf, float iou)
{
  return PostProcess_YOLOv8Detect_Ex(conf, iou, NULL, 0, VND_META_SLOT_MODEL_0);
}

IPostProcessor *PostProcess_YOLOv8Detect_Ex(float conf, float iou,
                                            const char *const *labels,
                                            int nlabels, vnd_meta_slot_t slot)
{
  det_configure(&g_y8.s, conf, iou, labels, nlabels, slot);
  return &g_y8_pp;
}

/* Square maps: channel-first or channel-last */

/* The order shared by a model's output maps. False if a map is not square. */
static bool maps_chw(const zoo_tensor_t *const *m, int32_t n, bool *chw)
{
  int32_t known = ZOO_LAYOUT_UNSURE;
  for (int32_t i = 0; i < n; i++) {
    const int32_t l = zoo_map_layout(m[i]);
    if (l == ZOO_LAYOUT_NONE) return false;
    if (l != ZOO_LAYOUT_UNSURE) {
      if (known != ZOO_LAYOUT_UNSURE && known != l) return false;
      known = l;
    }
  }
  if (known == ZOO_LAYOUT_UNSURE) {
    known = (m[0]->info != NULL && m[0]->info->chpos == CHPos_Last) ? ZOO_LAYOUT_HWC : ZOO_LAYOUT_CHW;
  }
  *chw = (known == ZOO_LAYOUT_CHW);
  return true;
}

/* ST YOLOX: three maps, strides 8, 16, 32, one anchor */

static zoo_det_state_t g_yx = ZOO_DET_STATE("STYoloX", 0.5f, 0.5f);

static int32_t yx_run(void *ctx, NN_Instance_TypeDef *model)
{
  zoo_common_t *c = &((zoo_det_state_t *)ctx)->c;
  c->n = 0;
  zoo_scratch_reset();

  zoo_tensor_t t[ZOO_MAX_OUTPUTS];
  const int32_t nt = zoo_outputs(model, t, ZOO_MAX_OUTPUTS);
  const zoo_tensor_t *m[3];
  int32_t nm = 0;
  for (int32_t i = 0; i < nt; i++) {
    if (t[i].ndims == 3u) {
      if (nm == 3) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected three output maps");
      m[nm++] = &t[i];
    }
  }
  if (nm != 3) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected three output maps");
  bool chw;
  if (!maps_chw(m, 3, &chw)) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "output maps are not square grids");

  /* Largest grid first: L, M, S. */
  for (int32_t i = 0; i < 2; i++) {
    for (int32_t j = i + 1; j < 3; j++) {
      const uint32_t gi = chw ? m[i]->dims[1] : m[i]->dims[0];
      const uint32_t gj = chw ? m[j]->dims[1] : m[j]->dims[0];
      if (gj > gi) { const zoo_tensor_t *x = m[i]; m[i] = m[j]; m[j] = x; }
    }
  }
  if (m[0]->type != m[1]->type || m[0]->type != m[2]->type ||
      (m[0]->type != DataType_INT8 && m[0]->type != DataType_FLOAT)) {
    return zoo_fail(c, ZOO_PP_ERR_TYPE, "expected three int8 or three float outputs");
  }

  const void *hwc[3];
  uint32_t g[3], ch[3];
  for (int32_t i = 0; i < 3; i++) {
    hwc[i] = as_hwc(m[i], chw, &g[i], &ch[i]);
    if (hwc[i] == NULL) return zoo_fail(c, ZOO_PP_ERR_SCRATCH, "outputs too large to reorder");
  }
  if (ch[0] != ch[1] || ch[0] != ch[2] || ch[0] < 6u) {
    return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected 5 + classes values per cell in every map");
  }
  const int32_t nc = (int32_t)ch[0] - 5;

  /* ST's default anchor, 0.5 of the picture, in grid cells of each map. */
  const float32_t anch[3][2] = {
    { 0.5f * (float32_t)g[0], 0.5f * (float32_t)g[0] },
    { 0.5f * (float32_t)g[1], 0.5f * (float32_t)g[1] },
    { 0.5f * (float32_t)g[2], 0.5f * (float32_t)g[2] },
  };
  const uint32_t total = g[0] * g[0] + g[1] * g[1] + g[2] * g[2];
  od_pp_outBuffer_t *cand = (od_pp_outBuffer_t *)zoo_scratch_alloc(total * sizeof(od_pp_outBuffer_t));
  if (cand == NULL) return zoo_fail(c, ZOO_PP_ERR_SCRATCH, "too many boxes for the scratch memory");

  od_st_yolox_pp_static_param_t p;
  memset(&p, 0, sizeof(p));
  p.nb_classes       = nc;
  p.nb_anchors       = 1;
  p.grid_width_L     = (int32_t)g[0];
  p.grid_height_L    = (int32_t)g[0];
  p.grid_width_M     = (int32_t)g[1];
  p.grid_height_M    = (int32_t)g[1];
  p.grid_width_S     = (int32_t)g[2];
  p.grid_height_S    = (int32_t)g[2];
  p.nb_input_boxes   = (int32_t)total;
  p.max_boxes_limit  = ZOO_PP_MAX_DET;
  p.conf_threshold   = c->conf;
  p.iou_threshold    = c->iou;
  p.pAnchors_L       = anch[0];
  p.pAnchors_M       = anch[1];
  p.pAnchors_S       = anch[2];
  p.raw_l_scale      = m[0]->scale;
  p.raw_m_scale      = m[1]->scale;
  p.raw_s_scale      = m[2]->scale;
  p.raw_l_zero_point = (int8_t)m[0]->zp;
  p.raw_m_zero_point = (int8_t)m[1]->zp;
  p.raw_s_zero_point = (int8_t)m[2]->zp;
  od_st_yolox_pp_in_t in = { (void *)hwc[0], (void *)hwc[1], (void *)hwc[2] };
  od_pp_out_t out = { cand, 0 };
  (void)od_st_yolox_pp_reset(&p);
  const int32_t err = (m[0]->type == DataType_INT8) ? od_st_yolox_pp_process_int8(&in, &out, &p)
                                                    : od_st_yolox_pp_process(&in, &out, &p);
  if (err != AI_OD_POSTPROCESS_ERROR_NO) return zoo_fail(c, ZOO_PP_ERR_DECODE, "decoder error");

  c->nb_classes = nc;
  zoo_take(c, cand, out.nb_detect);
  return zoo_ok(c);
}

static IPostProcessor g_yx_pp = {
  .init = zoo_det_init, .run = yx_run, .draw_lcd = zoo_det_draw,
  .emit_metadata = zoo_det_emit, .ctx = &g_yx,
};

IPostProcessor *PostProcess_STYoloX(float conf, float iou)
{
  return PostProcess_STYoloX_Ex(conf, iou, NULL, 0, VND_META_SLOT_MODEL_0);
}

IPostProcessor *PostProcess_STYoloX_Ex(float conf, float iou,
                                       const char *const *labels,
                                       int nlabels, vnd_meta_slot_t slot)
{
  det_configure(&g_yx, conf, iou, labels, nlabels, slot);
  return &g_yx_pp;
}

/* PixelKit's YOLOX: one output, a row per grid cell */

static zoo_det_state_t g_pk = ZOO_DET_STATE("PixelKitYOLOX", 0.5f, 0.45f);

static int32_t pk_run(void *ctx, NN_Instance_TypeDef *model)
{
  zoo_common_t *c = &((zoo_det_state_t *)ctx)->c;
  c->n = 0;
  zoo_scratch_reset();

  uint32_t w, h;
  if (!zoo_input_size(model, &w, &h) || w != h || w < 32u || (w % 32u) != 0u) {
    return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected a square input, a multiple of 32 pixels");
  }
  const uint32_t rows = (w / 8u) * (w / 8u) + (w / 16u) * (w / 16u) + (w / 32u) * (w / 32u);

  zoo_tensor_t t[ZOO_MAX_OUTPUTS];
  const int32_t nt = zoo_outputs(model, t, ZOO_MAX_OUTPUTS);
  if (nt != 1 || t[0].ndims != 2u || t[0].dims[0] != rows || t[0].dims[1] < 6u) {
    return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected one output, 5 + classes values per grid cell");
  }
  if (t[0].type != DataType_INT8 && t[0].type != DataType_FLOAT) {
    return zoo_fail(c, ZOO_PP_ERR_TYPE, "expected an int8 or float output");
  }
  const int32_t nc = (int32_t)t[0].dims[1] - 5;
  if (c->nlabels > 0 && c->nlabels != nc) {
    return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "the class names are from a different export than the model");
  }

  yolox_det_t *kept = (yolox_det_t *)zoo_scratch_alloc(ZOO_PP_MAX_DET * sizeof(yolox_det_t));
  od_pp_outBuffer_t *cand = (od_pp_outBuffer_t *)zoo_scratch_alloc(ZOO_PP_MAX_DET * sizeof(od_pp_outBuffer_t));
  if (kept == NULL || cand == NULL) return zoo_fail(c, ZOO_PP_ERR_SCRATCH, "no scratch memory for the boxes");

  const yolox_pp_params_t p = {
    .conf_threshold = c->conf, .iou_threshold = c->iou, .max_det = ZOO_PP_MAX_DET,
    .nb_classes = nc, .input_dim = (float32_t)w,
  };
  yolox_pp_out_t out = { kept, 0 };
  if (yolox_pp_process(LL_ATON_Output_Buffers_Info(model), &out, &p) != 0) {
    return zoo_fail(c, ZOO_PP_ERR_DECODE, "decoder error");
  }
  for (int32_t i = 0; i < out.nb_detect; i++) {
    cand[i].x_center    = kept[i].x_center;
    cand[i].y_center    = kept[i].y_center;
    cand[i].width       = kept[i].width;
    cand[i].height      = kept[i].height;
    cand[i].conf        = kept[i].conf;
    cand[i].class_index = kept[i].class_index;
  }

  c->nb_classes = nc;
  zoo_take(c, cand, out.nb_detect);
  return zoo_ok(c);
}

static IPostProcessor g_pk_pp = {
  .init = zoo_det_init, .run = pk_run, .draw_lcd = zoo_det_draw,
  .emit_metadata = zoo_det_emit, .ctx = &g_pk,
};

IPostProcessor *PostProcess_PixelKitYOLOX(float conf, float iou,
                                          const char *const *labels, int nlabels)
{
  return PostProcess_PixelKitYOLOX_Ex(conf, iou, labels, nlabels, VND_META_SLOT_MODEL_0);
}

IPostProcessor *PostProcess_PixelKitYOLOX_Ex(float conf, float iou,
                                             const char *const *labels,
                                             int nlabels, vnd_meta_slot_t slot)
{
  det_configure(&g_pk, conf, iou, labels, nlabels, slot);
  return &g_pk_pp;
}

/* Tiny YOLOv2: one map, five anchors */

/* ST's default anchors from parse_config.py, in grid cells. */
#define ZOO_Y2_ANCHORS 5
static const float32_t k_y2_anchors[2 * ZOO_Y2_ANCHORS] = {
  0.076023f, 0.258508f, 0.163031f, 0.413531f, 0.234769f,
  0.702585f, 0.427054f, 0.715892f, 0.748154f, 0.857092f,
};

static zoo_det_state_t g_y2 = ZOO_DET_STATE("TinyYOLOv2", 0.5f, 0.5f);

static int32_t y2_run(void *ctx, NN_Instance_TypeDef *model)
{
  zoo_common_t *c = &((zoo_det_state_t *)ctx)->c;
  c->n = 0;
  zoo_scratch_reset();

  zoo_tensor_t t[ZOO_MAX_OUTPUTS];
  const int32_t nt = zoo_outputs(model, t, ZOO_MAX_OUTPUTS);
  const zoo_tensor_t *o = largest(t, nt, 3u);
  if (o == NULL) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected one output map");
  if (o->type != DataType_INT8 && o->type != DataType_FLOAT) {
    return zoo_fail(c, ZOO_PP_ERR_TYPE, "expected an int8 or float output");
  }
  bool chw;
  if (!maps_chw(&o, 1, &chw)) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "output map is not a square grid");

  uint32_t g, ch;
  const void *hwc = as_hwc(o, chw, &g, &ch);
  if (hwc == NULL) return zoo_fail(c, ZOO_PP_ERR_SCRATCH, "output too large to reorder");
  if (ch % ZOO_Y2_ANCHORS != 0u || ch / ZOO_Y2_ANCHORS < 6u) {
    return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected 5 anchors x (5 + classes) values per cell");
  }
  const int32_t nc = (int32_t)(ch / ZOO_Y2_ANCHORS) - 5;

  /* ST's NMS clears one entry past the candidates, hence the + 1. */
  const uint32_t boxes = g * g * ZOO_Y2_ANCHORS;
  od_pp_outBuffer_t *work = (od_pp_outBuffer_t *)zoo_scratch_alloc((boxes + 1u) * sizeof(od_pp_outBuffer_t));
  od_pp_outBuffer_t *cand = (od_pp_outBuffer_t *)zoo_scratch_alloc(boxes * sizeof(od_pp_outBuffer_t));
  if (work == NULL || cand == NULL) return zoo_fail(c, ZOO_PP_ERR_SCRATCH, "too many boxes for the scratch memory");

  od_yolov2_pp_static_param_t p;
  memset(&p, 0, sizeof(p));
  p.nb_classes      = nc;
  p.nb_anchors      = ZOO_Y2_ANCHORS;
  p.grid_width      = (int32_t)g;
  p.grid_height     = (int32_t)g;
  p.nb_input_boxes  = (int32_t)boxes;
  p.max_boxes_limit = ZOO_PP_MAX_DET;
  p.conf_threshold  = c->conf;
  p.iou_threshold   = c->iou;
  p.pAnchors        = k_y2_anchors;
  p.raw_scale       = o->scale;
  p.raw_zero_point  = (int8_t)o->zp;
  p.pScratchBuffer  = work;
  od_yolov2_pp_in_t in = { (void *)hwc };
  od_pp_out_t out = { cand, 0 };
  (void)od_yolov2_pp_reset(&p);
  const int32_t err = (o->type == DataType_INT8) ? od_yolov2_pp_process_int8(&in, &out, &p)
                                                 : od_yolov2_pp_process(&in, &out, &p);
  if (err != AI_OD_POSTPROCESS_ERROR_NO) return zoo_fail(c, ZOO_PP_ERR_DECODE, "decoder error");

  c->nb_classes = nc;
  zoo_take(c, cand, out.nb_detect);
  return zoo_ok(c);
}

static IPostProcessor g_y2_pp = {
  .init = zoo_det_init, .run = y2_run, .draw_lcd = zoo_det_draw,
  .emit_metadata = zoo_det_emit, .ctx = &g_y2,
};

IPostProcessor *PostProcess_TinyYOLOv2(float conf, float iou)
{
  return PostProcess_TinyYOLOv2_Ex(conf, iou, NULL, 0, VND_META_SLOT_MODEL_0);
}

IPostProcessor *PostProcess_TinyYOLOv2_Ex(float conf, float iou,
                                          const char *const *labels,
                                          int nlabels, vnd_meta_slot_t slot)
{
  det_configure(&g_y2, conf, iou, labels, nlabels, slot);
  return &g_y2_pp;
}

/* SSD: scores [priors][1 + classes], boxes [priors][4] */

/* The priors of generate_ssd_priors() in ST's object_detection postprocess.py:
 * six feature maps, six priors a cell, 3000 in all. */
static const struct {
  uint8_t  fm;        /* feature map side */
  uint16_t shrink;    /* stride in pixels of the 300-pixel input */
  uint16_t smin;      /* box sizes, pixels */
  uint16_t smax;
} k_ssd_specs[6] = {
  { 19,  16,  60, 105 },
  { 10,  32, 105, 150 },
  {  5,  64, 150, 195 },
  {  3, 100, 195, 240 },
  {  2, 150, 240, 285 },
  {  1, 300, 285, 330 },
};
#define ZOO_SSD_PRIORS     3000u
#define ZOO_SSD_CENTER_VAR 0.1f
#define ZOO_SSD_SIZE_VAR   0.2f

static float32_t clamp01(float32_t v)
{
  return (v < 0.f) ? 0.f : ((v > 1.f) ? 1.f : v);
}

/* Prior idx as centre x, y, width, height, each clamped to 0..1. */
static void ssd_prior(uint32_t idx, float32_t img, float32_t pr[4])
{
  uint32_t l = 0;
  for (; l < 5u; l++) {
    const uint32_t n = (uint32_t)k_ssd_specs[l].fm * k_ssd_specs[l].fm * 6u;
    if (idx < n) break;
    idx -= n;
  }
  const uint32_t fm = k_ssd_specs[l].fm;
  const uint32_t cell = idx / 6u, k = idx % 6u;
  const float32_t scale = img / (float32_t)k_ssd_specs[l].shrink;
  const float32_t smin = (float32_t)k_ssd_specs[l].smin / img;
  float32_t w = smin, h = smin;
  if (k == 1u) {
    w = h = sqrtf((float32_t)k_ssd_specs[l].smax * (float32_t)k_ssd_specs[l].smin) / img;
  } else if (k >= 2u) {
    const float32_t r = sqrtf((k < 4u) ? 2.0f : 3.0f);
    if ((k & 1u) == 0u) { w = smin * r; h = smin / r; }
    else                { w = smin / r; h = smin * r; }
  }
  pr[0] = clamp01(((float32_t)(cell % fm) + 0.5f) / scale);
  pr[1] = clamp01(((float32_t)(cell / fm) + 0.5f) / scale);
  pr[2] = clamp01(w);
  pr[3] = clamp01(h);
}

static zoo_det_state_t g_ssd = ZOO_DET_STATE("SSD", 0.5f, 0.5f);

/* exp(-scale * d) for d = 0..255, for softmax over int8 scores. */
static float32_t s_ssd_lut[256];
static float32_t s_ssd_lut_scale = -1.0f;

static int32_t ssd_run(void *ctx, NN_Instance_TypeDef *model)
{
  zoo_common_t *c = &((zoo_det_state_t *)ctx)->c;
  c->n = 0;
  zoo_scratch_reset();

  zoo_tensor_t t[ZOO_MAX_OUTPUTS];
  const int32_t nt = zoo_outputs(model, t, ZOO_MAX_OUTPUTS);
  const zoo_tensor_t *bx = NULL, *sc = NULL;
  for (int32_t i = 0; i < nt; i++) {
    if (t[i].ndims != 2u) continue;
    if (t[i].dims[1] == 4u && bx == NULL) bx = &t[i];
    else if (sc == NULL) sc = &t[i];
  }
  if (bx == NULL || sc == NULL || sc->dims[0] != bx->dims[0] || sc->dims[1] < 2u) {
    return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected scores [priors][1 + classes] and boxes [priors][4]");
  }
  if (bx->dims[0] != ZOO_SSD_PRIORS) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected 3000 priors (300x300 input)");

  uint32_t w = 300u, h = 300u;
  (void)zoo_input_size(model, &w, &h);
  const float32_t img = (float32_t)w;
  const uint32_t n = ZOO_SSD_PRIORS, k = sc->dims[1];

  od_pp_outBuffer_t *cand = (od_pp_outBuffer_t *)zoo_scratch_alloc(n * sizeof(od_pp_outBuffer_t));
  if (cand == NULL) return zoo_fail(c, ZOO_PP_ERR_SCRATCH, "too many boxes for the scratch memory");

  /* Prefilter: the probability cannot exceed sigmoid(best - background). */
  float32_t conf = c->conf;
  if (conf < 1e-6f) conf = 1e-6f;
  if (conf > 1.0f - 1e-6f) conf = 1.0f - 1e-6f;
  const float32_t dmin = logf(conf / (1.0f - conf));

  const bool q8 = (sc->type == DataType_INT8);
  if (q8 && sc->scale != s_ssd_lut_scale) {
    for (uint32_t d = 0; d < 256u; d++) s_ssd_lut[d] = expf(-sc->scale * (float32_t)d);
    s_ssd_lut_scale = sc->scale;
  }

  int32_t nc = 0;
  for (uint32_t i = 0; i < n; i++) {
    uint32_t best = 1u;
    float32_t p;
    if (q8) {
      const int8_t *q = (const int8_t *)sc->data + i * k;
      int32_t qb = q[1];
      for (uint32_t j = 2; j < k; j++) {
        if (q[j] > qb) { qb = q[j]; best = j; }
      }
      if (sc->scale * (float32_t)(qb - q[0]) < dmin) continue;
      const int32_t qm = (qb > q[0]) ? qb : q[0];
      float32_t sum = 0.f;
      for (uint32_t j = 0; j < k; j++) sum += s_ssd_lut[qm - q[j]];
      p = s_ssd_lut[qm - qb] / sum;
    } else {
      const uint32_t base = i * k;
      float32_t vb = zoo_value(sc, base + 1u);
      for (uint32_t j = 2; j < k; j++) {
        const float32_t v = zoo_value(sc, base + j);
        if (v > vb) { vb = v; best = j; }
      }
      const float32_t v0 = zoo_value(sc, base);
      if (vb - v0 < dmin) continue;
      const float32_t vm = (vb > v0) ? vb : v0;
      float32_t sum = 0.f;
      for (uint32_t j = 0; j < k; j++) sum += expf(zoo_value(sc, base + j) - vm);
      p = expf(vb - vm) / sum;
    }
    if (p < c->conf) continue;

    /* convert_locations_to_boxes() with variances 0.1 and 0.2. */
    float32_t pr[4];
    ssd_prior(i, img, pr);
    od_pp_outBuffer_t *d = &cand[nc++];
    d->x_center    = zoo_value(bx, i * 4u + 0u) * ZOO_SSD_CENTER_VAR * pr[2] + pr[0];
    d->y_center    = zoo_value(bx, i * 4u + 1u) * ZOO_SSD_CENTER_VAR * pr[3] + pr[1];
    d->width       = expf(zoo_value(bx, i * 4u + 2u) * ZOO_SSD_SIZE_VAR) * pr[2];
    d->height      = expf(zoo_value(bx, i * 4u + 3u) * ZOO_SSD_SIZE_VAR) * pr[3];
    d->conf        = p;
    d->class_index = (int32_t)best - 1;   /* class 0 is the background */
  }

  c->nb_classes = (int32_t)k - 1;
  zoo_take(c, cand, nc);
  return zoo_ok(c);
}

static IPostProcessor g_ssd_pp = {
  .init = zoo_det_init, .run = ssd_run, .draw_lcd = zoo_det_draw,
  .emit_metadata = zoo_det_emit, .ctx = &g_ssd,
};

IPostProcessor *PostProcess_SSD(float conf, float iou)
{
  return PostProcess_SSD_Ex(conf, iou, NULL, 0, VND_META_SLOT_MODEL_0);
}

IPostProcessor *PostProcess_SSD_Ex(float conf, float iou,
                                   const char *const *labels, int nlabels,
                                   vnd_meta_slot_t slot)
{
  det_configure(&g_ssd, conf, iou, labels, nlabels, slot);
  return &g_ssd_pp;
}

/* BlazeFace, front camera, 128x128 */

/* Anchor centres of MediaPipe's SsdAnchorsCalculator for the front model: 2 a
 * cell at stride 8 and 6 at stride 16. */
#define ZOO_BF_MAX_ANCHORS (32u * 32u * 2u + 16u * 16u * 6u)   /* 256x256 input */

__attribute__((section(".ext_psram")))
static float32_t s_bf_anchors[2u * ZOO_BF_MAX_ANCHORS];
static uint32_t s_bf_size;   /* input size the anchors were built for */

static bool bf_anchors(uint32_t size)
{
  if (s_bf_size == size) return true;
  const uint32_t g0 = size / 8u, g1 = size / 16u;
  if (g0 * g0 * 2u + g1 * g1 * 6u > ZOO_BF_MAX_ANCHORS) return false;
  uint32_t n = 0;
  const uint32_t grid[2] = { g0, g1 }, per[2] = { 2u, 6u };
  for (uint32_t l = 0; l < 2u; l++) {
    for (uint32_t y = 0; y < grid[l]; y++) {
      for (uint32_t x = 0; x < grid[l]; x++) {
        for (uint32_t r = 0; r < per[l]; r++) {
          s_bf_anchors[2u * n + 0u] = ((float32_t)x + 0.5f) / (float32_t)grid[l];
          s_bf_anchors[2u * n + 1u] = ((float32_t)y + 0.5f) / (float32_t)grid[l];
          n++;
        }
      }
    }
  }
  s_bf_size = size;
  return true;
}

static zoo_det_state_t g_bf = ZOO_DET_STATE("BlazeFace", 0.5f, 0.3f);

static int32_t bf_run(void *ctx, NN_Instance_TypeDef *model)
{
  zoo_common_t *c = &((zoo_det_state_t *)ctx)->c;
  c->n = 0;
  zoo_scratch_reset();

  zoo_tensor_t t[ZOO_MAX_OUTPUTS];
  const int32_t nt = zoo_outputs(model, t, ZOO_MAX_OUTPUTS);
  const zoo_tensor_t *bx[2] = { NULL, NULL }, *sc[2] = { NULL, NULL };
  int32_t nb = 0, ns = 0;
  for (int32_t i = 0; i < nt; i++) {
    if (t[i].ndims == 2u && t[i].dims[1] >= 4u && nb < 2) bx[nb++] = &t[i];
    else if (t[i].ndims == 1u && ns < 2) sc[ns++] = &t[i];
  }
  if (nb != 2 || ns != 2) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected two box and two score outputs");

  uint32_t w = 0, h = 0;
  if (!zoo_input_size(model, &w, &h) || w != h) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "expected a square input");
  if (!bf_anchors(w)) return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "input too large");
  const uint32_t n0 = (w / 8u) * (w / 8u) * 2u, n1 = (w / 16u) * (w / 16u) * 6u;

  /* Stride-8 outputs first. */
  if (bx[0]->dims[0] != n0) { const zoo_tensor_t *x = bx[0]; bx[0] = bx[1]; bx[1] = x; }
  if (sc[0]->dims[0] != n0) { const zoo_tensor_t *x = sc[0]; sc[0] = sc[1]; sc[1] = x; }
  if (bx[0]->dims[0] != n0 || bx[1]->dims[0] != n1 || sc[0]->dims[0] != n0 || sc[1]->dims[0] != n1 ||
      bx[0]->dims[1] != bx[1]->dims[1] || (bx[0]->dims[1] & 1u) != 0u) {
    return zoo_fail(c, ZOO_PP_ERR_OUTPUTS, "outputs do not match the front model's anchors");
  }
  const int32_t type = bx[0]->type;
  if (bx[1]->type != type || sc[0]->type != type || sc[1]->type != type) {
    return zoo_fail(c, ZOO_PP_ERR_TYPE, "outputs of mixed data types");
  }

  od_pp_outBuffer_t *cand = (od_pp_outBuffer_t *)zoo_scratch_alloc((n0 + n1) * sizeof(od_pp_outBuffer_t));
  if (cand == NULL) return zoo_fail(c, ZOO_PP_ERR_SCRATCH, "too many boxes for the scratch memory");

  od_fd_blazeface_pp_static_param_t p;
  memset(&p, 0, sizeof(p));
  p.nb_classes         = 1;
  p.nb_keypoints       = (int32_t)(bx[0]->dims[1] - 4u) / 2;
  p.nb_detections_0    = (int32_t)n0;
  p.nb_detections_1    = (int32_t)n1;
  p.in_size            = w;
  p.max_boxes_limit    = ZOO_PP_MAX_DET;
  p.conf_threshold     = c->conf;
  p.iou_threshold      = c->iou;
  p.pAnchors_0         = &s_bf_anchors[0];
  p.pAnchors_1         = &s_bf_anchors[2u * n0];
  p.boxe_0_scale       = bx[0]->scale;
  p.proba_0_scale      = sc[0]->scale;
  p.boxe_1_scale       = bx[1]->scale;
  p.proba_1_scale      = sc[1]->scale;
  p.boxe_0_zero_point  = (uint8_t)bx[0]->zp;
  p.proba_0_zero_point = (uint8_t)sc[0]->zp;
  p.boxe_1_zero_point  = (uint8_t)bx[1]->zp;
  p.proba_1_zero_point = (uint8_t)sc[1]->zp;
  od_fd_blazeface_pp_in_t in = { bx[0]->data, bx[1]->data, sc[0]->data, sc[1]->data };
  od_pp_out_t out = { cand, 0 };
  (void)od_fd_blazeface_pp_reset(&p);
  int32_t err;
  if (type == DataType_FLOAT)     err = od_fd_blazeface_pp_process(&in, &out, &p);
  else if (type == DataType_INT8) err = od_fd_blazeface_pp_process_int8(&in, &out, &p);
  else                            err = od_fd_blazeface_pp_process_uint8(&in, &out, &p);
  if (err != AI_OD_POSTPROCESS_ERROR_NO) return zoo_fail(c, ZOO_PP_ERR_DECODE, "decoder error");

  c->nb_classes = 1;
  zoo_take(c, cand, out.nb_detect);
  return zoo_ok(c);
}

static IPostProcessor g_bf_pp = {
  .init = zoo_det_init, .run = bf_run, .draw_lcd = zoo_det_draw,
  .emit_metadata = zoo_det_emit, .ctx = &g_bf,
};

static const char *const k_face[] = { "face" };

IPostProcessor *PostProcess_BlazeFace(float conf, float iou)
{
  return PostProcess_BlazeFace_Ex(conf, iou, k_face, 1, VND_META_SLOT_MODEL_0);
}

IPostProcessor *PostProcess_BlazeFace_Ex(float conf, float iou,
                                         const char *const *labels,
                                         int nlabels, vnd_meta_slot_t slot)
{
  if (labels == NULL) { labels = k_face; nlabels = 1; }
  det_configure(&g_bf, conf, iou, labels, nlabels, slot);
  return &g_bf_pp;
}
