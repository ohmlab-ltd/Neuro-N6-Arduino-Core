/**
 ******************************************************************************
 * @file    zoo_pp.h
 * @brief   Shared helpers for the model zoo post-processors.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef ZOO_PP_H
#define ZOO_PP_H

#include <stdint.h>
#include <stdbool.h>

/* Outside extern "C" so arm_mve.h keeps its C++ overloads. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "ll_aton_rt_user_api.h"
#include "od_pp_output_if.h"
#include "vnd_meta.h"
#include "NeuroN6_IPostProcessor.h"

/* Detections kept after NMS, per post-processor. */
#ifndef ZOO_PP_MAX_DET
#define ZOO_PP_MAX_DET        (100)
#endif

/* Scratch arena size. Every run() starts from an empty arena. */
#ifndef ZOO_PP_SCRATCH_BYTES
#define ZOO_PP_SCRATCH_BYTES  (1024u * 1024u)
#endif

/* Error codes returned by run(). */
#define ZOO_PP_OK             (0)
#define ZOO_PP_ERR_OUTPUTS    (-10)  /* outputs are not what this model family produces */
#define ZOO_PP_ERR_TYPE       (-11)  /* output data type not supported */
#define ZOO_PP_ERR_SCRATCH    (-12)  /* ZOO_PP_SCRATCH_BYTES too small for this model */
#define ZOO_PP_ERR_DECODE     (-13)  /* ST's decoder reported an error */

/* State common to the zoo post-processors. Must be the first member of ctx. */

#define ZOO_PP_MAGIC          (0x5A4F4F31u)   /* "ZOO1" */

typedef struct {
  uint32_t           magic;
  const char        *name;        /* for the Terminal message on an error */
  float32_t          conf;
  float32_t          iou;
  const char *const *labels;
  int32_t            nlabels;
  vnd_meta_slot_t    slot;
  int32_t            nb_classes;  /* read from the outputs; 0 before a frame */
  int32_t            status;      /* last run() result */
  od_pp_outBuffer_t *det;         /* results, 0..1 of the network's input */
  int32_t            n;
} zoo_common_t;

/* Clears the results, prints "<name>: <why>" to Neuro Studio's Terminal when
 * the code changes, and returns code. */
int32_t zoo_fail(zoo_common_t *c, int32_t code, const char *why);

static inline int32_t zoo_ok(zoo_common_t *c)
{
  c->status = ZOO_PP_OK;
  return ZOO_PP_OK;
}

/* Runs NMS over cand[0..n) and keeps the best ZOO_PP_MAX_DET in c->det. */
void zoo_take(zoo_common_t *c, od_pp_outBuffer_t *cand, int32_t n);

/* IPostProcessor callbacks for the detectors. ctx is a zoo_common_t. */
int32_t zoo_det_init(void *ctx, NN_Instance_TypeDef *model);
void    zoo_det_draw(void *ctx, OVL_Mode_t mode);
void    zoo_det_emit(void *ctx);

/* Output tensors */

#define ZOO_MAX_OUTPUTS       (8)

typedef struct {
  const LL_Buffer_InfoTypeDef *info;
  uint8_t   *data;
  uint32_t   count;      /* elements */
  int32_t    type;       /* DataType_FLOAT, DataType_INT8 or DataType_UINT8 */
  uint32_t   ndims;      /* mem_shape with its size-1 dimensions dropped */
  uint32_t   dims[6];    /* in memory order, outermost first */
  float32_t  scale;      /* 1 for float tensors */
  int32_t    zp;         /* 0 for float tensors */
} zoo_tensor_t;

/* Fills t[] from the model's output buffers. Returns the count, or 0 if one
 * has an unsupported type or more than 6 dimensions. */
int32_t zoo_outputs(NN_Instance_TypeDef *model, zoo_tensor_t *t, int32_t max);

/* Width and height of the first input buffer. False if the model has no usable
 * input info. */
bool zoo_input_size(NN_Instance_TypeDef *model, uint32_t *w, uint32_t *h);

/* Element i as a float, dequantised. */
static inline float32_t zoo_value(const zoo_tensor_t *t, uint32_t i)
{
  switch (t->type) {
    case DataType_INT8:  return t->scale * (float32_t)((int32_t)((const int8_t *)t->data)[i] - t->zp);
    case DataType_UINT8: return t->scale * (float32_t)((int32_t)t->data[i] - t->zp);
    default:             return ((const float32_t *)t->data)[i];
  }
}

/* Bytes per element. */
static inline uint32_t zoo_esize(const zoo_tensor_t *t)
{
  return (t->type == DataType_FLOAT) ? 4u : 1u;
}

/* Order of a square map, [c][g][g] or [g][g][c]. */
#define ZOO_LAYOUT_NONE       (-1)   /* not a square map */
#define ZOO_LAYOUT_UNSURE     (0)    /* all three sizes equal */
#define ZOO_LAYOUT_CHW        (1)
#define ZOO_LAYOUT_HWC        (2)
int32_t zoo_map_layout(const zoo_tensor_t *t);

/* [c][h][w] to [h][w][c]. esize is 1 or 4 bytes per element. */
void zoo_chw_to_hwc(const void *src, void *dst, uint32_t c, uint32_t h,
                    uint32_t w, uint32_t esize);

/* Scratch arena */

void  zoo_scratch_reset(void);
/* 8-byte aligned. NULL when the arena is exhausted. */
void *zoo_scratch_alloc(uint32_t bytes);

/* Detections */

/* Greedy per-class NMS over d[0..n), in place, best first. Returns the number
 * kept, at most max_keep. */
int32_t zoo_nms(od_pp_outBuffer_t *d, int32_t n, float32_t iou, int32_t max_keep);

float32_t zoo_iou(const od_pp_outBuffer_t *a, const od_pp_outBuffer_t *b);

/* labels[i] when given, else the COCO-80 name when nb_classes is 80, else
 * "class <i>". The pointer stays valid until the next call. */
const char *zoo_label(const char *const *labels, int32_t nlabels,
                      int32_t nb_classes, int32_t i);

/* USB metadata */

/* Every publisher writes the whole slot. META_INFERENCE_MS and META_BOXES are
 * sent on VND_META_SLOT_MODEL_0 only. */

/* Boxes, with a META_ISEG_CLASSSTR name per box. */
void zoo_meta_detections(const od_pp_outBuffer_t *d, int32_t n,
                         const char *const *labels, int32_t nlabels,
                         int32_t nb_classes, vnd_meta_slot_t slot);

/* One keypoint: x, y NN-normalised, conf 0..1. */
typedef struct {
  float32_t x;
  float32_t y;
  float32_t conf;
} zoo_kp_t;

/* The COCO skeleton: 19 lines of {keypoint a, keypoint b, colour id}, and a
 * colour id per keypoint. Colour ids: 0 head, 1 arms, 2 trunk, 3 legs. */
extern const uint8_t zoo_coco_bindings[19][3];
extern const uint8_t zoo_coco_kp_color[17];

/* kps holds 17 entries per box, and a keypoint below min_conf is sent with
 * confidence 0. At most 13 poses. */
void zoo_meta_coco_pose(const od_pp_outBuffer_t *boxes, const zoo_kp_t *kps,
                        int32_t n, float32_t min_conf, vnd_meta_slot_t slot);

/* The box around every pixel of one class in a segmentation label map. */
typedef struct {
  od_pp_outBuffer_t box;    /* conf = share of the picture the class covers */
  uint16_t x0, y0, x1, y1;  /* label-map pixels, x1/y1 exclusive */
} zoo_region_t;

/* Regions of a [h][w] label map, each with a 1-bit META_ISEG_MASK of its
 * class, at most 96x96. */
void zoo_meta_regions(const zoo_region_t *r, int32_t n, const uint8_t *map,
                      uint32_t w, uint32_t h, const char *const *labels,
                      int32_t nlabels, int32_t nb_classes,
                      vnd_meta_slot_t slot);

/* LCD overlay */

void zoo_lcd_detections(const od_pp_outBuffer_t *d, int32_t n,
                        const char *const *labels, int32_t nlabels,
                        int32_t nb_classes, OVL_Mode_t mode);

void zoo_lcd_coco_pose(const od_pp_outBuffer_t *boxes, const zoo_kp_t *kps,
                       int32_t n, float32_t min_conf, OVL_Mode_t mode);

/* Tints every pixel of class >= 1 in a [h][w] label map, then labels each
 * region. */
void zoo_lcd_regions(const zoo_region_t *r, int32_t n, const uint8_t *map,
                     uint32_t w, uint32_t h, const char *const *labels,
                     int32_t nlabels, int32_t nb_classes, OVL_Mode_t mode);

/* Text lines at the top left, below the inference time. */
void zoo_lcd_text_lines(const char *const *text, int32_t lines, OVL_Mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* ZOO_PP_H */
