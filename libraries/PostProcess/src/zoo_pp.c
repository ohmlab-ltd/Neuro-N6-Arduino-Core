/**
 ******************************************************************************
 * @file    zoo_pp.c
 * @brief   Shared helpers for the model zoo post-processors.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "zoo_pp.h"
#include "PostProcessor_Zoo.h"
#include "NeuroN6.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Inference timestamps: ts[0] before the NPU pass, ts[1] after. */
extern uint32_t ts[];

/* COCO-80 names. The default table is weak, so a sketch can replace it. */
extern const char *classes_table[80];

/* Output tensors */

int32_t zoo_outputs(NN_Instance_TypeDef *model, zoo_tensor_t *t, int32_t max)
{
  if (model == NULL || t == NULL) return 0;
  const LL_Buffer_InfoTypeDef *b = LL_ATON_Output_Buffers_Info(model);
  if (b == NULL) return 0;

  int32_t n = 0;
  for (; n < max && b[n].name != NULL; n++) {
    const LL_Buffer_InfoTypeDef *bi = &b[n];
    zoo_tensor_t *z = &t[n];
    memset(z, 0, sizeof(*z));

    if (bi->type != DataType_FLOAT && bi->type != DataType_INT8 &&
        bi->type != DataType_UINT8) {
      return 0;
    }
    if (bi->type != DataType_FLOAT && bi->per_channel) return 0;

    z->info  = bi;
    z->data  = LL_Buffer_addr_start(bi);
    z->type  = (int32_t)bi->type;
    z->count = LL_Buffer_len(bi) / ((bi->type == DataType_FLOAT) ? 4u : 1u);
    z->scale = 1.0f;
    z->zp    = 0;
    if (bi->type != DataType_FLOAT) {
      if (bi->scale != NULL)  z->scale = bi->scale[0];
      if (bi->offset != NULL) z->zp    = (int32_t)bi->offset[0];
    }

    /* mem_shape is the order the elements sit in, whatever chpos says. */
    uint32_t prod = 1u;
    if (bi->mem_shape != NULL) {
      for (uint32_t d = 0; d < bi->mem_ndims; d++) {
        const uint32_t v = bi->mem_shape[d];
        prod *= v;
        if (v == 1u) continue;
        if (z->ndims == 6u) return 0;
        z->dims[z->ndims++] = v;
      }
    }
    if (bi->mem_shape == NULL || prod != z->count) {
      z->ndims   = 1u;
      z->dims[0] = z->count;
    } else if (z->ndims == 0u) {
      z->ndims   = 1u;          /* a single value */
      z->dims[0] = 1u;
    }
  }
  return n;
}

bool zoo_input_size(NN_Instance_TypeDef *model, uint32_t *w, uint32_t *h)
{
  if (model == NULL || w == NULL || h == NULL) return false;
  const LL_Buffer_InfoTypeDef *b = LL_ATON_Input_Buffers_Info(model);
  if (b == NULL || b[0].name == NULL || b[0].mem_shape == NULL ||
      b[0].mem_ndims < 3) {
    return false;
  }
  const uint32_t *s = b[0].mem_shape;
  const uint32_t nd = b[0].mem_ndims;
  /* {1, H, W, C} or {1, C, H, W}: the channel axis is the one of at most 4. */
  if (s[nd - 1] <= 4u) {
    *h = s[nd - 3];
    *w = s[nd - 2];
  } else if (s[nd - 3] <= 4u) {
    *h = s[nd - 2];
    *w = s[nd - 1];
  } else {
    return false;
  }
  return (*w > 0u && *h > 0u);
}

int32_t zoo_map_layout(const zoo_tensor_t *t)
{
  if (t->ndims != 3u) return ZOO_LAYOUT_NONE;
  const uint32_t a = t->dims[0], b = t->dims[1], c = t->dims[2];
  if (a == b && b == c) return ZOO_LAYOUT_UNSURE;
  if (b == c) return ZOO_LAYOUT_CHW;
  if (a == b) return ZOO_LAYOUT_HWC;
  return ZOO_LAYOUT_NONE;
}

void zoo_chw_to_hwc(const void *src, void *dst, uint32_t c, uint32_t h,
                    uint32_t w, uint32_t esize)
{
  const uint32_t hw = h * w;
  if (esize == 1u) {
    const uint8_t *s = (const uint8_t *)src;
    uint8_t *d = (uint8_t *)dst;
    for (uint32_t ch = 0; ch < c; ch++) {
      const uint8_t *sp = s + ch * hw;
      for (uint32_t p = 0; p < hw; p++) d[p * c + ch] = sp[p];
    }
  } else {
    const uint32_t *s = (const uint32_t *)src;
    uint32_t *d = (uint32_t *)dst;
    for (uint32_t ch = 0; ch < c; ch++) {
      const uint32_t *sp = s + ch * hw;
      for (uint32_t p = 0; p < hw; p++) d[p * c + ch] = sp[p];
    }
  }
}

/* Scratch arena */

__attribute__((section(".ext_psram"), aligned(32)))
static uint8_t s_scratch[ZOO_PP_SCRATCH_BYTES];
static uint32_t s_scratch_used;

void zoo_scratch_reset(void)
{
  s_scratch_used = 0u;
}

void *zoo_scratch_alloc(uint32_t bytes)
{
  const uint32_t off = (s_scratch_used + 7u) & ~7u;
  if (bytes > ZOO_PP_SCRATCH_BYTES || off > ZOO_PP_SCRATCH_BYTES - bytes) {
    return NULL;
  }
  s_scratch_used = off + bytes;
  return &s_scratch[off];
}

/* Detections */

float32_t zoo_iou(const od_pp_outBuffer_t *a, const od_pp_outBuffer_t *b)
{
  const float32_t ax0 = a->x_center - 0.5f * a->width;
  const float32_t ax1 = a->x_center + 0.5f * a->width;
  const float32_t ay0 = a->y_center - 0.5f * a->height;
  const float32_t ay1 = a->y_center + 0.5f * a->height;
  const float32_t bx0 = b->x_center - 0.5f * b->width;
  const float32_t bx1 = b->x_center + 0.5f * b->width;
  const float32_t by0 = b->y_center - 0.5f * b->height;
  const float32_t by1 = b->y_center + 0.5f * b->height;

  const float32_t iw = ((ax1 < bx1) ? ax1 : bx1) - ((ax0 > bx0) ? ax0 : bx0);
  const float32_t ih = ((ay1 < by1) ? ay1 : by1) - ((ay0 > by0) ? ay0 : by0);
  if (iw <= 0.f || ih <= 0.f) return 0.f;
  const float32_t inter = iw * ih;
  const float32_t uni = a->width * a->height + b->width * b->height - inter;
  return (uni > 0.f) ? (inter / uni) : 0.f;
}

static int zoo_conf_desc(const void *pa, const void *pb)
{
  const float32_t a = ((const od_pp_outBuffer_t *)pa)->conf;
  const float32_t b = ((const od_pp_outBuffer_t *)pb)->conf;
  return (a < b) - (a > b);
}

int32_t zoo_nms(od_pp_outBuffer_t *d, int32_t n, float32_t iou, int32_t max_keep)
{
  if (d == NULL || n <= 0) return 0;
  qsort(d, (size_t)n, sizeof(*d), zoo_conf_desc);

  int32_t kept = 0;
  for (int32_t i = 0; i < n && kept < max_keep; i++) {
    bool drop = false;
    for (int32_t k = 0; k < kept && !drop; k++) {
      if (d[k].class_index == d[i].class_index && zoo_iou(&d[k], &d[i]) > iou) {
        drop = true;
      }
    }
    if (!drop) d[kept++] = d[i];
  }
  return kept;
}

int32_t zoo_fail(zoo_common_t *c, int32_t code, const char *why)
{
  c->n = 0;
  if (c->status != code) {
    char line[160];
    snprintf(line, sizeof(line), "%s: %s", c->name, why);
    (void)vnd_meta_publish_debug_text(line);
  }
  c->status = code;
  return code;
}

void zoo_take(zoo_common_t *c, od_pp_outBuffer_t *cand, int32_t n)
{
  n = zoo_nms(cand, n, c->iou, ZOO_PP_MAX_DET);
  for (int32_t i = 0; i < n; i++) c->det[i] = cand[i];
  c->n = n;
}

int32_t zoo_det_init(void *ctx, NN_Instance_TypeDef *model)
{
  (void)model;
  zoo_common_t *c = (zoo_common_t *)ctx;
  c->n = 0;
  c->status = ZOO_PP_OK;
  return 0;
}

void zoo_det_draw(void *ctx, OVL_Mode_t mode)
{
  const zoo_common_t *c = (const zoo_common_t *)ctx;
  zoo_lcd_detections(c->det, c->n, c->labels, c->nlabels, c->nb_classes, mode);
}

void zoo_det_emit(void *ctx)
{
  const zoo_common_t *c = (const zoo_common_t *)ctx;
  zoo_meta_detections(c->det, c->n, c->labels, c->nlabels, c->nb_classes, c->slot);
}

const char *zoo_label(const char *const *labels, int32_t nlabels,
                      int32_t nb_classes, int32_t i)
{
  static char s_name[16];
  if (labels != NULL) {
    if (i >= 0 && i < nlabels && labels[i] != NULL) return labels[i];
  } else if (nb_classes == 80 && i >= 0 && i < 80 && classes_table[i] != NULL) {
    return classes_table[i];
  }
  snprintf(s_name, sizeof(s_name), "class %ld", (long)i);
  return s_name;
}

/* USB metadata */

/* NN-normalised box to camera-normalised centre and size, 4 x u16 LE. The
 * corners are clipped to the frame: a letterboxed input reaches past it. */
static void zoo_box_to_cam_u16(const od_pp_outBuffer_t *d, uint8_t *out8)
{
  const float32_t cl = vnd_meta_clamp01(nn_norm_to_cam_norm_x(d->x_center - 0.5f * d->width));
  const float32_t cr = vnd_meta_clamp01(nn_norm_to_cam_norm_x(d->x_center + 0.5f * d->width));
  const float32_t ct = vnd_meta_clamp01(nn_norm_to_cam_norm_y(d->y_center - 0.5f * d->height));
  const float32_t cb = vnd_meta_clamp01(nn_norm_to_cam_norm_y(d->y_center + 0.5f * d->height));
  float32_t v[4] = { 0.5f * (cl + cr), 0.5f * (ct + cb), cr - cl, cb - ct };
  for (int k = 0; k < 4; k++) {
    if (v[k] < 0.f) v[k] = 0.f;
    if (v[k] > 1.f) v[k] = 1.f;
    vnd_meta_put_u16_le(&out8[2 * k], vnd_meta_f01_to_u16(v[k]));
  }
}

static void zoo_meta_begin(vnd_meta_builder_t *meta, vnd_meta_slot_t slot)
{
  vnd_meta_builder_begin(meta, slot);
  if (slot == VND_META_SLOT_MODEL_0) {
    const uint32_t ms = ts[1] - ts[0];
    const uint16_t ms16 = (ms > 65535u) ? 65535u : (uint16_t)ms;
    (void)vnd_meta_builder_put_tlv(meta, META_INFERENCE_MS, &ms16, sizeof(ms16));
  }
}

/* META_ISEG_CLASSSTR: obj_id, then the name. */
static bool zoo_put_name(vnd_meta_builder_t *meta, uint8_t obj_id, const char *name)
{
  uint8_t tmp[1 + 48];
  uint32_t len = (uint32_t)strlen(name);
  if (len > 48u) len = 48u;
  tmp[0] = obj_id;
  memcpy(&tmp[1], name, len);
  return vnd_meta_builder_put_tlv(meta, META_ISEG_CLASSSTR, tmp, (uint16_t)(1u + len));
}

/* Box list: u8 count, then per box u16 xc, yc, w, h, u8 conf q8, u8 class. */
__attribute__((section(".ext_psram")))
static uint8_t s_boxes[1 + ZOO_PP_MAX_DET * 10];

void zoo_meta_detections(const od_pp_outBuffer_t *d, int32_t n,
                         const char *const *labels, int32_t nlabels,
                         int32_t nb_classes, vnd_meta_slot_t slot)
{
  vnd_meta_builder_t meta;
  zoo_meta_begin(&meta, slot);

  if (n > ZOO_PP_MAX_DET) n = ZOO_PP_MAX_DET;
  uint32_t bl = 1u;
  uint8_t kept = 0;
  for (int32_t i = 0; i < n && d != NULL; i++) {
    if (slot == VND_META_SLOT_MODEL_0 &&
        !zoo_put_name(&meta, kept, zoo_label(labels, nlabels, nb_classes, d[i].class_index))) {
      break;   /* slot full */
    }
    zoo_box_to_cam_u16(&d[i], &s_boxes[bl]);
    bl += 8u;
    s_boxes[bl++] = vnd_meta_conf_to_q8(d[i].conf);
    s_boxes[bl++] = (uint8_t)d[i].class_index;
    kept++;
  }
  s_boxes[0] = kept;
  if (slot == VND_META_SLOT_MODEL_0) {
    (void)vnd_meta_builder_put_tlv(&meta, META_BOXES, s_boxes, (uint16_t)bl);
  }
  vnd_meta_builder_finish(&meta);
}

/* COCO 17-keypoint skeleton */
enum { ZOO_HEAD = 0, ZOO_ARMS = 1, ZOO_TRUNK = 2, ZOO_LEGS = 3 };

const uint8_t zoo_coco_bindings[19][3] = {
  {15, 13, ZOO_LEGS},  {13, 11, ZOO_LEGS},  {16, 14, ZOO_LEGS},  {14, 12, ZOO_LEGS},
  {11, 12, ZOO_TRUNK}, { 5, 11, ZOO_TRUNK}, { 6, 12, ZOO_TRUNK},
  { 5,  6, ZOO_ARMS},  { 5,  7, ZOO_ARMS},  { 6,  8, ZOO_ARMS},  { 7,  9, ZOO_ARMS},
  { 8, 10, ZOO_ARMS},
  { 1,  2, ZOO_HEAD},  { 0,  1, ZOO_HEAD},  { 0,  2, ZOO_HEAD},  { 1,  3, ZOO_HEAD},
  { 2,  4, ZOO_HEAD},  { 3,  5, ZOO_HEAD},  { 4,  6, ZOO_HEAD},
};

const uint8_t zoo_coco_kp_color[17] = {
  ZOO_HEAD, ZOO_HEAD, ZOO_HEAD, ZOO_HEAD, ZOO_HEAD,
  ZOO_ARMS, ZOO_ARMS, ZOO_ARMS, ZOO_ARMS, ZOO_ARMS, ZOO_ARMS,
  ZOO_TRUNK, ZOO_TRUNK,
  ZOO_LEGS, ZOO_LEGS, ZOO_LEGS, ZOO_LEGS,
};

/* A keypoint is 7 bytes: id, colour, x, y, conf. 17 keypoints and 19 lines a
 * pose, u8 counts: at most 13 poses. */
#define ZOO_MAX_POSES 13

__attribute__((section(".ext_psram")))
static uint8_t s_kps[1 + ZOO_MAX_POSES * 17 * 7];
__attribute__((section(".ext_psram")))
static uint8_t s_binds[1 + ZOO_MAX_POSES * 19 * 3];

void zoo_meta_coco_pose(const od_pp_outBuffer_t *boxes, const zoo_kp_t *kps,
                        int32_t n, float32_t min_conf, vnd_meta_slot_t slot)
{
  vnd_meta_builder_t meta;
  zoo_meta_begin(&meta, slot);

  if (n > ZOO_MAX_POSES) n = ZOO_MAX_POSES;
  if (boxes == NULL || kps == NULL) n = 0;

  uint32_t bl = 1u, kl = 1u, il = 1u;
  for (int32_t p = 0; p < n; p++) {
    const uint8_t base = (uint8_t)(p * 17);
    /* The name labels box p, and only slot 0 sends boxes. */
    if (slot == VND_META_SLOT_MODEL_0) (void)zoo_put_name(&meta, (uint8_t)p, "person");
    zoo_box_to_cam_u16(&boxes[p], &s_boxes[bl]);
    bl += 8u;
    s_boxes[bl++] = vnd_meta_conf_to_q8(boxes[p].conf);
    s_boxes[bl++] = 0u;

    for (uint32_t k = 0; k < 17u; k++) {
      const zoo_kp_t *kp = &kps[p * 17 + (int32_t)k];
      float32_t x = nn_norm_to_cam_norm_x(kp->x);
      float32_t y = nn_norm_to_cam_norm_y(kp->y);
      if (x < 0.f) x = 0.f;
      if (x > 1.f) x = 1.f;
      if (y < 0.f) y = 0.f;
      if (y > 1.f) y = 1.f;
      s_kps[kl++] = (uint8_t)(base + k);
      s_kps[kl++] = zoo_coco_kp_color[k];
      vnd_meta_put_u16_le(&s_kps[kl], vnd_meta_f01_to_u16(x)); kl += 2u;
      vnd_meta_put_u16_le(&s_kps[kl], vnd_meta_f01_to_u16(y)); kl += 2u;
      s_kps[kl++] = vnd_meta_conf_to_q8((kp->conf >= min_conf) ? kp->conf : 0.f);
    }
    for (uint32_t b = 0; b < 19u; b++) {
      s_binds[il++] = (uint8_t)(base + zoo_coco_bindings[b][0]);
      s_binds[il++] = (uint8_t)(base + zoo_coco_bindings[b][1]);
      s_binds[il++] = zoo_coco_bindings[b][2];
    }
  }
  s_kps[0]   = (uint8_t)(n * 17);
  s_binds[0] = (uint8_t)(n * 19);
  (void)vnd_meta_builder_put_tlv(&meta, META_KEYPOINTS, s_kps, (uint16_t)kl);
  (void)vnd_meta_builder_put_tlv(&meta, META_BINDINGS, s_binds, (uint16_t)il);
  if (slot == VND_META_SLOT_MODEL_0) {
    s_boxes[0] = (uint8_t)n;
    (void)vnd_meta_builder_put_tlv(&meta, META_BOXES, s_boxes, (uint16_t)bl);
  }
  vnd_meta_builder_finish(&meta);
}

/* Largest mask side sent to the host, in mask pixels. */
#define ZOO_MASK_MAX 96u

/* META_ISEG_MASK: obj_id, W, H, u16 byte count, then W*H bits, row by row,
 * least significant bit first. */
static bool zoo_put_mask(vnd_meta_builder_t *meta, uint8_t obj_id,
                         const zoo_region_t *r, const uint8_t *map, uint32_t w)
{
  const uint32_t rw = (uint32_t)(r->x1 - r->x0);
  const uint32_t rh = (uint32_t)(r->y1 - r->y0);
  if (rw == 0u || rh == 0u) return false;
  const uint32_t mw = (rw < ZOO_MASK_MAX) ? rw : ZOO_MASK_MAX;
  const uint32_t mh = (rh < ZOO_MASK_MAX) ? rh : ZOO_MASK_MAX;
  const uint32_t nbytes = (mw * mh + 7u) / 8u;
  if (meta->len + 3u + 5u + nbytes > meta->capacity) return false;

  uint8_t *p = &meta->buf[meta->len];
  p[0] = META_ISEG_MASK;
  vnd_meta_put_u16_le(&p[1], (uint16_t)(5u + nbytes));
  p[3] = obj_id;
  p[4] = (uint8_t)mw;
  p[5] = (uint8_t)mh;
  vnd_meta_put_u16_le(&p[6], (uint16_t)nbytes);
  uint8_t *bits = &p[8];
  memset(bits, 0, nbytes);

  const uint8_t cls = (uint8_t)r->box.class_index;
  uint32_t bit = 0;
  for (uint32_t y = 0; y < mh; y++) {
    const uint32_t sy = r->y0 + (y * rh) / mh;
    const uint8_t *row = &map[sy * w];
    for (uint32_t x = 0; x < mw; x++, bit++) {
      const uint32_t sx = r->x0 + (x * rw) / mw;
      if (row[sx] == cls) bits[bit >> 3] |= (uint8_t)(1u << (bit & 7u));
    }
  }
  meta->len += 8u + nbytes;
  return true;
}

void zoo_meta_regions(const zoo_region_t *r, int32_t n, const uint8_t *map,
                      uint32_t w, uint32_t h, const char *const *labels,
                      int32_t nlabels, int32_t nb_classes,
                      vnd_meta_slot_t slot)
{
  (void)h;
  vnd_meta_builder_t meta;
  zoo_meta_begin(&meta, slot);

  if (n > ZOO_PP_MAX_DET) n = ZOO_PP_MAX_DET;
  uint32_t bl = 1u;
  uint8_t kept = 0;
  for (int32_t i = 0; i < n && r != NULL && map != NULL; i++) {
    if (slot == VND_META_SLOT_MODEL_0) {
      if (!zoo_put_mask(&meta, kept, &r[i], map, w)) break;
      if (!zoo_put_name(&meta, kept, zoo_label(labels, nlabels, nb_classes, r[i].box.class_index))) break;
    }
    zoo_box_to_cam_u16(&r[i].box, &s_boxes[bl]);
    bl += 8u;
    s_boxes[bl++] = vnd_meta_conf_to_q8(r[i].box.conf);
    s_boxes[bl++] = (uint8_t)r[i].box.class_index;
    kept++;
  }
  s_boxes[0] = kept;
  if (slot == VND_META_SLOT_MODEL_0) {
    (void)vnd_meta_builder_put_tlv(&meta, META_BOXES, s_boxes, (uint16_t)bl);
  }
  vnd_meta_builder_finish(&meta);
}

/* Sketch accessors */

static const zoo_common_t *zoo_common(const IPostProcessor *pp)
{
  if (pp == NULL || pp->ctx == NULL) return NULL;
  const zoo_common_t *c = (const zoo_common_t *)pp->ctx;
  return (c->magic == ZOO_PP_MAGIC) ? c : NULL;
}

int PostProcess_ZooResults(const IPostProcessor *pp, const od_pp_outBuffer_t **boxes)
{
  const zoo_common_t *c = zoo_common(pp);
  if (boxes != NULL) *boxes = (c != NULL) ? c->det : NULL;
  return (c != NULL && c->det != NULL) ? (int)c->n : 0;
}

int PostProcess_ZooStatus(const IPostProcessor *pp)
{
  const zoo_common_t *c = zoo_common(pp);
  return (c != NULL) ? (int)c->status : ZOO_PP_ERR_OUTPUTS;
}
