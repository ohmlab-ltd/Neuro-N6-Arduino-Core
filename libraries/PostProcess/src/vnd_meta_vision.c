/**
 * @file    vnd_meta_vision.c
 * @brief   Publisher for the vision records: QR, barcode, blob, AprilTag.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "vnd_meta_vision.h"
#include "NeuroN6.h"
#include <string.h>

void vnd_meta_vision_begin(vnd_meta_vision_t *v,
                           vnd_meta_slot_t vslot, uint8_t vtag,
                           uint8_t *payload, uint32_t cap)
{
  if (v == NULL) return;
  v->vslot   = vslot;
  v->vtag    = vtag;
  v->payload = payload;
  v->cap     = cap;
  v->pl      = 1u;      /* [0] is the count, written by finish() */
  v->bl      = 1u;      /* the same for the box list */
  v->count   = 0u;

  v->ev      = cap;
  v->ev_n    = 0u;
  v->want_ev = n6_host_evidence_wanted(N6_EVIDENCE_CROP);
}

/* Adds a VND_VISION_CROP_DIM square RGB565 crop of the box to the evidence
 * region. False when it does not fit or no host is subscribed. */
static bool put_crop(vnd_meta_vision_t *v, uint8_t result_index,
                     const uint8_t *src, int sw, int sh,
                     int bx, int by, int bw, int bh)
{
  if (bx < 0) { bw += bx; bx = 0; }
  if (by < 0) { bh += by; by = 0; }
  if (bx + bw > sw) bw = sw - bx;
  if (by + bh > sh) bh = sh - by;

  if (!v->want_ev || src == NULL || bw < 1 || bh < 1) return false;

  const uint32_t need = 3u + (uint32_t)VND_VISION_CROP_DIM *
                             (uint32_t)VND_VISION_CROP_DIM * 2u;
  if (v->ev < need || (v->ev - need) < v->pl) return false;   /* regions met */

  /* A TLV length is u16, and eight 64x64 crops come to 65560 bytes. */
  if ((v->cap - (v->ev - need)) + 1u > 65535u) return false;

  v->ev -= need;
  uint32_t w = v->ev;
  v->payload[w++] = result_index;
  v->payload[w++] = (uint8_t)VND_VISION_CROP_DIM;
  v->payload[w++] = (uint8_t)VND_VISION_CROP_DIM;

  for (int y = 0; y < VND_VISION_CROP_DIM; y++) {
    const int sy = by + (y * bh) / VND_VISION_CROP_DIM;
    const uint8_t *row = src + (size_t)sy * (size_t)sw * 3u;
    for (int x = 0; x < VND_VISION_CROP_DIM; x++) {
      const int sx = bx + (x * bw) / VND_VISION_CROP_DIM;
      const uint8_t *p = row + (size_t)sx * 3u;
      const uint16_t px = (uint16_t)(((p[0] >> 3) << 11) |
                                     ((p[1] >> 2) << 5)  |
                                      (p[2] >> 3));
      v->payload[w++] = (uint8_t)(px & 0xFFu);
      v->payload[w++] = (uint8_t)(px >> 8);
    }
  }
  v->ev_n++;
  return true;
}

bool vnd_meta_vision_add(vnd_meta_vision_t *v,
                         float cx, float cy, float w, float h,
                         const char *label,
                         const void *extra, uint16_t extra_len,
                         const uint8_t *rgb888, int src_w, int src_h)
{
  if (v == NULL || v->payload == NULL) return false;
  if (v->count >= (uint8_t)VND_VISION_MAX_RESULTS) return false;

  if (v->pl + 8u + (uint32_t)extra_len > v->ev) return false;
  if (v->bl + 10u > sizeof(v->boxes))            return false;

  /* NN-window normalised to full-frame normalised, through the corners, which
   * are clipped to the frame: a letterboxed input reaches past it. */
  const float l = vnd_meta_clamp01(nn_norm_to_cam_norm_x(cx - 0.5f * w));
  const float r = vnd_meta_clamp01(nn_norm_to_cam_norm_x(cx + 0.5f * w));
  const float t = vnd_meta_clamp01(nn_norm_to_cam_norm_y(cy - 0.5f * h));
  const float b = vnd_meta_clamp01(nn_norm_to_cam_norm_y(cy + 0.5f * h));
  const uint16_t ncx = vnd_meta_f01_to_u16(0.5f * (l + r));
  const uint16_t ncy = vnd_meta_f01_to_u16(0.5f * (t + b));
  const uint16_t nw  = vnd_meta_f01_to_u16(r - l);
  const uint16_t nh  = vnd_meta_f01_to_u16(b - t);

  /* Record: box, per-sensor fields, crop. */
  const uint32_t pl_start = v->pl;
  vnd_meta_put_u16_le(&v->payload[v->pl], ncx); v->pl += 2u;
  vnd_meta_put_u16_le(&v->payload[v->pl], ncy); v->pl += 2u;
  vnd_meta_put_u16_le(&v->payload[v->pl], nw);  v->pl += 2u;
  vnd_meta_put_u16_le(&v->payload[v->pl], nh);  v->pl += 2u;

  if (extra != NULL && extra_len > 0u) {
    memcpy(&v->payload[v->pl], extra, extra_len);
    v->pl += extra_len;
  }

  /* crop_w and crop_h are 0: the pixels travel in META_EVIDENCE_CROP. */
  if (v->pl + 2u > v->ev) { v->pl = pl_start; return false; }
  v->payload[v->pl++] = 0u;
  v->payload[v->pl++] = 0u;

  {
    /* A crop that does not fit is left out. */
    const int bw = (int)(w * (float)src_w);
    const int bh = (int)(h * (float)src_h);
    const int bx = (int)((cx - w * 0.5f) * (float)src_w);
    const int by = (int)((cy - h * 0.5f) * (float)src_h);
    (void)put_crop(v, v->count, rgb888, src_w, src_h, bx, by, bw, bh);
  }

  vnd_meta_put_u16_le(&v->boxes[v->bl], ncx); v->bl += 2u;
  vnd_meta_put_u16_le(&v->boxes[v->bl], ncy); v->bl += 2u;
  vnd_meta_put_u16_le(&v->boxes[v->bl], nw);  v->bl += 2u;
  vnd_meta_put_u16_le(&v->boxes[v->bl], nh);  v->bl += 2u;
  v->boxes[v->bl++] = 255u;   /* score */
  v->boxes[v->bl++] = 0u;     /* class */

  {
    char *dst = v->labels[v->count];
    uint32_t n = 0u;
    if (label != NULL) {
      while (label[n] != '\0' && n < (uint32_t)VND_VISION_LABEL_MAX) {
        dst[n] = label[n];
        n++;
      }
    }
    dst[n] = '\0';
  }

  v->count++;
  return true;
}

void vnd_meta_vision_finish(vnd_meta_vision_t *v)
{
  if (v == NULL) return;

  /* Published at count 0 too: it clears the host panel. */
  if (v->payload != NULL && v->cap >= 1u) {
    v->payload[0] = v->count;
    (void)vnd_meta_publish_tlv(v->vslot, v->vtag, v->payload, (uint16_t)v->pl);
  }

  /* The crops are in reverse result order. Each carries its result_index. */
  if (v->want_ev && v->payload != NULL && v->ev >= 1u && (v->ev - 1u) >= v->pl) {
    v->payload[--v->ev] = v->ev_n;
    (void)vnd_meta_publish_tlv(VND_META_SLOT_EVIDENCE,
                               (uint8_t)META_EVIDENCE_CROP,
                               &v->payload[v->ev],
                               (uint16_t)(v->cap - v->ev));
  }

  /* An empty box list is written too: it clears the overlay. */
  v->boxes[0] = v->count;

  vnd_meta_builder_t b;
  vnd_meta_builder_begin(&b, VND_META_SLOT_MODEL_0);
  if (!vnd_meta_builder_put_tlv(&b, (uint8_t)META_BOXES, v->boxes,
                                (uint16_t)v->bl)) {
    return;   /* nothing published */
  }
  for (uint8_t i = 0u; i < v->count; i++) {
    const uint32_t n = (uint32_t)strlen(v->labels[i]);
    if (n == 0u) continue;
    uint8_t tmp[1 + VND_VISION_LABEL_MAX];
    tmp[0] = i;                       /* obj_id, an index into META_BOXES */
    memcpy(&tmp[1], v->labels[i], n);
    if (!vnd_meta_builder_put_tlv(&b, (uint8_t)META_ISEG_CLASSSTR, tmp,
                                  (uint16_t)(1u + n))) {
      break;                          /* out of room; the boxes still go */
    }
  }
  vnd_meta_builder_finish(&b);
}

void vnd_meta_vision_announce(uint8_t kind)
{
  (void)vnd_meta_publish_tlv(VND_META_SLOT_VISION_ID, (uint8_t)META_VISION_ID,
                             &kind, 1u);
}
