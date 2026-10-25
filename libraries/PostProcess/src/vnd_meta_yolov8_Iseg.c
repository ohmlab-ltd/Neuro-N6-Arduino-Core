/**
 * @file    vnd_meta_yolov8_Iseg.c
 * @brief   Instance segmentation metadata builder.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "vnd_meta_yolov8_Iseg.h"
#include "app_postprocess_iseg_yolo_v8_ui.h"
#include "NeuroN6.h"
#include "n6_host_contract.h"
#include <string.h>

/* The box count and each object id are one byte. */
N6_STATIC_ASSERT(AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT <= 255, "more boxes than a one-byte count holds");

/* Inference timestamps */
extern uint32_t ts[];

static inline const char *class_name_from_id(uint32_t cls)
{
  if (cls < NB_CLASSES)
    return classes_table[cls];
  return "unknown";
}

static uint16_t pack_mask_roi_1bpp_thresh(
    uint8_t *dst,
    const uint8_t *srcMxM,
    uint16_t M,
    uint16_t x0,
    uint16_t y0,
    uint16_t W,
    uint16_t H,
    uint8_t thr)
{
  const uint32_t nbits  = (uint32_t)W * (uint32_t)H;
  const uint32_t nbytes = (nbits + 7u) / 8u;
  memset(dst, 0, nbytes);

  uint32_t bit = 0;
  for (uint16_t y = 0; y < H; y++) {
    const uint32_t src_row = (uint32_t)(y0 + y) * (uint32_t)M;
    for (uint16_t x = 0; x < W; x++, bit++) {
      const uint32_t si = src_row + (uint32_t)(x0 + x);
      if (srcMxM[si] >= thr) {
        dst[bit >> 3] |= (uint8_t)(1u << (bit & 7u));
      }
    }
  }
  return (uint16_t)nbytes;
}

/* A box edge in mask cells, clamped to [0, M]. NaN gives 0. */
static int mask_edge(float32_t v, uint8_t M)
{
  if (!(v > 0.0f)) return 0;
  if (v >= (float32_t)M) return (int)M;
  return (int)v;
}

/* Camera-normalised to NN-normalised, the inverse of nn_norm_to_cam_norm_x. */
static float32_t cam_norm_to_nn_norm_x(float32_t xc)
{
  return (xc * (float32_t)g_camera_full_w - (float32_t)g_nn_crop_x) / (float32_t)g_nn_crop_w;
}

static float32_t cam_norm_to_nn_norm_y(float32_t yc)
{
  return (yc * (float32_t)g_camera_full_h - (float32_t)g_nn_crop_y) / (float32_t)g_nn_crop_h;
}

/* l, t, r and b are the box's edges, NN-normalised. */
static bool meta_put_iseg_mask_bbox_local(
    vnd_meta_builder_t *meta,
    uint8_t obj_id,
    const uint8_t *maskMxM,
    uint8_t M,
    float32_t l, float32_t t, float32_t r, float32_t b)
{
  if (!meta || !maskMxM || M == 0) return false;

  /* Each edge inside the mask, so the region the packer reads is too. */
  int x0 = mask_edge(l * (float32_t)M, M);
  int x1 = mask_edge(r * (float32_t)M, M);
  int y0 = mask_edge(t * (float32_t)M, M);
  int y1 = mask_edge(b * (float32_t)M, M);

  if (x1 <= x0) { x0 = (x0 > 0) ? (x0 - 1) : 0; x1 = x0 + 1; }
  if (y1 <= y0) { y0 = (y0 > 0) ? (y0 - 1) : 0; y1 = y0 + 1; }

  uint16_t outW = (uint16_t)(x1 - x0);
  uint16_t outH = (uint16_t)(y1 - y0);

  if (outW > 255) outW = 255;
  if (outH > 255) outH = 255;

  const uint16_t mask_bytes =
      (uint16_t)(((uint32_t)outW * (uint32_t)outH + 7u) / 8u);

  if (meta->len + 3u + 5u + mask_bytes > meta->capacity) return false;

  meta->buf[meta->len + 0] = META_ISEG_MASK;
  vnd_meta_put_u16_le(&meta->buf[meta->len + 1], (uint16_t)(5u + mask_bytes));
  meta->len += 3u;

  meta->buf[meta->len + 0] = obj_id;
  meta->buf[meta->len + 1] = (uint8_t)outW;
  meta->buf[meta->len + 2] = (uint8_t)outH;
  vnd_meta_put_u16_le(&meta->buf[meta->len + 3], mask_bytes);
  meta->len += 5u;

#ifndef ISEG_MASK_BIN_THR
#define ISEG_MASK_BIN_THR 1
#endif

  (void)pack_mask_roi_1bpp_thresh(
      &meta->buf[meta->len],
      maskMxM,
      (uint16_t)M,
      (uint16_t)x0,
      (uint16_t)y0,
      outW,
      outH,
      (uint8_t)ISEG_MASK_BIN_THR);

  meta->len += mask_bytes;
  return true;
}

void vnd_meta_build_iseg_on_slot(const iseg_pp_out_t *out, uint32_t inference_ms,
                                 const iseg_yolov8_pp_static_param_t *pp_params,
                                 vnd_meta_slot_t slot)
{
  vnd_meta_builder_t meta;
  vnd_meta_builder_begin(&meta, slot);

  /* Frame-global TLVs: only slot 0 sends META_INFERENCE_MS and META_BOXES. */
  const bool emit_frame_global = (slot == VND_META_SLOT_MODEL_0);

  if (emit_frame_global) {
    uint16_t inf_ms_u16 = (inference_ms > 65535u) ? 65535u : (uint16_t)inference_ms;
    (void)vnd_meta_builder_put_tlv(&meta, META_INFERENCE_MS, &inf_ms_u16, sizeof(inf_ms_u16));
  }

  if (!out || out->nb_detect == 0)
  {
    vnd_meta_builder_finish(&meta);
    return;
  }

  const uint8_t M = (uint8_t)AI_YOLOV8_SEG_PP_MASK_SIZE;

  uint32_t n = out->nb_detect;

  if (n > AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT) n = AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT;

  uint8_t obj_id = 0;
  uint8_t boxes[1 + (AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT * (2*4 + 2))];
  uint32_t bl = 0;

  boxes[bl++] = 0;
  uint8_t kept = 0;
  for (uint32_t i = 0; i < n; i++)
  {
    const iseg_pp_outBuffer_t *d = &out->pOutBuff[i];

    if (pp_params && d->conf < pp_params->conf_threshold) continue;
    if (!d->pMask) continue;

    uint8_t this_id = obj_id;

    /* NN-normalised to camera-normalised, through the two corners. The box is
     * cut to the network's view, then to the frame, which under a letterbox
     * is the smaller. */
    float32_t nn_l = vnd_meta_clamp01(d->x_center - 0.5f * d->width);
    float32_t nn_r = vnd_meta_clamp01(d->x_center + 0.5f * d->width);
    float32_t nn_t = vnd_meta_clamp01(d->y_center - 0.5f * d->height);
    float32_t nn_b = vnd_meta_clamp01(d->y_center + 0.5f * d->height);

    float32_t cam_l = vnd_meta_clamp01(nn_norm_to_cam_norm_x(nn_l));
    float32_t cam_r = vnd_meta_clamp01(nn_norm_to_cam_norm_x(nn_r));
    float32_t cam_t = vnd_meta_clamp01(nn_norm_to_cam_norm_y(nn_t));
    float32_t cam_b = vnd_meta_clamp01(nn_norm_to_cam_norm_y(nn_b));

    float32_t xc_f = 0.5f * (cam_l + cam_r);
    float32_t yc_f = 0.5f * (cam_t + cam_b);
    float32_t w_f  = cam_r - cam_l;
    float32_t h_f  = cam_b - cam_t;

    uint16_t xc = vnd_meta_f01_to_u16(xc_f);
    uint16_t yc = vnd_meta_f01_to_u16(yc_f);
    uint16_t ww = vnd_meta_f01_to_u16(w_f);
    uint16_t hh = vnd_meta_f01_to_u16(h_f);

    uint8_t confq = vnd_meta_conf_to_q8(d->conf);

    const char *name = class_name_from_id((uint32_t)d->class_index);
    if (!name) name = "unknown";
    uint16_t name_len = (uint16_t)strlen(name);
    if (name_len > 200) name_len = 200;

    /* The mask writer checks its own space. A mask goes out only with its
     * class name: without room for the name, the mask is taken back. Both
     * index slot 0's boxes, the only ones sent. The mask is cut to the same
     * box, as the host stretches it over the box. */
    const uint32_t len_before = meta.len;
    if (emit_frame_global && !meta_put_iseg_mask_bbox_local(
            &meta,
            this_id,
            (const uint8_t *)d->pMask,
            (uint8_t)M,
            cam_norm_to_nn_norm_x(cam_l), cam_norm_to_nn_norm_y(cam_t),
            cam_norm_to_nn_norm_x(cam_r), cam_norm_to_nn_norm_y(cam_b)))
    {
      continue;
    }

    if (emit_frame_global) {
      uint8_t tmp[1 + 200];
      tmp[0] = this_id;
      memcpy(&tmp[1], name, name_len);
      if (!vnd_meta_builder_put_tlv(&meta, META_ISEG_CLASSSTR, tmp, (uint16_t)(1u + name_len))) {
        meta.len = len_before;
        continue;
      }
    }

    vnd_meta_put_u16_le(&boxes[bl], xc); bl += 2;
    vnd_meta_put_u16_le(&boxes[bl], yc); bl += 2;
    vnd_meta_put_u16_le(&boxes[bl], ww); bl += 2;
    vnd_meta_put_u16_le(&boxes[bl], hh); bl += 2;
    boxes[bl++] = confq;
    boxes[bl++] = (uint8_t)d->class_index;
    kept++;

    obj_id++;
  }
  boxes[0] = kept;
  if (emit_frame_global) {
    (void)vnd_meta_builder_put_tlv(&meta, META_BOXES, boxes, (uint16_t)bl);
  }
  vnd_meta_builder_finish(&meta);
}

void Display_NS_Yolo_V8_Iseg_OnSlot(vnd_meta_slot_t slot)
{
  vnd_meta_build_iseg_on_slot(&pp_output, ts[1] - ts[0], &pp_params, slot);
}
