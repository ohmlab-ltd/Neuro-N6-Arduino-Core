/**
 * @file    vnd_meta_vision.h
 * @brief   Publisher for the vision records: QR, barcode, blob, AprilTag.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef VND_META_VISION_H
#define VND_META_VISION_H

#include <stdint.h>
#include <stdbool.h>
#include "vnd_meta.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Results per frame, and the longest on-box label. */
#define VND_VISION_MAX_RESULTS  8
#define VND_VISION_LABEL_MAX    47

/* Side of the square RGB565 crop, in pixels. */
#define VND_VISION_CROP_DIM     64

typedef struct {
  vnd_meta_slot_t vslot;      /* such as VND_META_SLOT_QR */
  uint8_t         vtag;       /* such as META_QR_FRAME */

  /* Caller's buffer. The record grows up from 0, the crops down from cap. */
  uint8_t  *payload;
  uint32_t  cap;
  uint32_t  pl;         /* end of the record */
  uint32_t  ev;         /* start of the crops */
  uint8_t   ev_n;       /* crops this frame */
  bool      want_ev;    /* a host is subscribed, latched at begin() */

  /* Boxes and labels for the video overlay. */
  uint8_t   boxes[1 + VND_VISION_MAX_RESULTS * 10];
  uint32_t  bl;
  char      labels[VND_VISION_MAX_RESULTS][VND_VISION_LABEL_MAX + 1];
  uint8_t   count;
} vnd_meta_vision_t;

/* Starts a frame. payload is the caller's, sized for the record plus
 * VND_VISION_MAX_RESULTS crops of 8 KB. */
void vnd_meta_vision_begin(vnd_meta_vision_t *v,
                           vnd_meta_slot_t vslot, uint8_t vtag,
                           uint8_t *payload, uint32_t cap);

/* Adds one result. Returns false, having added nothing, on overflow.
 *   box:    cx, cy, w, h, 0..1 of the NN window
 *   label:  drawn on the box, or NULL
 *   extra:  per-sensor fields, little-endian, or NULL
 *   rgb888: frame the crop is cut from, or NULL */
bool vnd_meta_vision_add(vnd_meta_vision_t *v,
                         float cx, float cy, float w, float h,
                         const char *label,
                         const void *extra, uint16_t extra_len,
                         const uint8_t *rgb888, int src_w, int src_h);

/* Publishes the record to its slot, and the boxes and labels to
 * VND_META_SLOT_MODEL_0. Called on every frame, with or without results. */
void vnd_meta_vision_finish(vnd_meta_vision_t *v);

/* Publishes META_VISION_ID. kind is one of its bit values. Called about every
 * 2 s. */
void vnd_meta_vision_announce(uint8_t kind);

#ifdef __cplusplus
}
#endif

#endif /* VND_META_VISION_H */
