/**
 ******************************************************************************
 * @file    app_postprocess_pd_ui.c
 * @brief   Palm-detector post-processing wrapper.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "app_postprocess_pd_ui.h"
#include <stddef.h>
#include <string.h>

pd_ui_detection_t          pd_ui_detections[AI_PD_PP_MAX_DETECTIONS];
uint32_t                   pd_ui_nb_detections = 0u;
pd_model_pp_static_param_t pd_pp_params;

__attribute__((section(".ext_psram")))
static pd_pp_point_t s_anchors[AI_PD_PP_TOTAL_BOXES];

static pd_pp_box_t   s_boxes[AI_PD_PP_MAX_BOXES_LIMIT];
static pd_pp_point_t s_kps[AI_PD_PP_MAX_BOXES_LIMIT * AI_PD_PP_NB_KEYPOINTS];
static pd_pp_out_t   s_pp_out;

/* Anchor centres in 0..1, in output tensor order. MediaPipe
 * SsdAnchorsCalculator settings for palm_detection_full: strides {8,16,16,16},
 * aspect_ratios {1.0}, interpolated_scale_aspect_ratio 1.0, offset 0.5,
 * fixed_anchor_size. */
static void pd_build_anchors(pd_pp_point_t *a)
{
  static const struct { int grid; int per_cell; } layers[] = {
    { 24, 2 },   /* stride 8  over 192 px */
    { 12, 6 },   /* stride 16 over 192 px, 3 layers */
  };

  uint32_t n = 0u;
  for (uint32_t l = 0u; l < (sizeof(layers) / sizeof(layers[0])); l++) {
    const int g = layers[l].grid;
    for (int y = 0; y < g; y++) {
      for (int x = 0; x < g; x++) {
        const float32_t cx = ((float32_t)x + 0.5f) / (float32_t)g;
        const float32_t cy = ((float32_t)y + 0.5f) / (float32_t)g;
        for (int k = 0; k < layers[l].per_cell; k++) {
          if (n >= (uint32_t)AI_PD_PP_TOTAL_BOXES) return;
          a[n].x = cx;
          a[n].y = cy;
          n++;
        }
      }
    }
  }
}

static float32_t pd_ui_iou(const pd_pp_box_t *a, const pd_pp_box_t *b)
{
  const float32_t ax0 = a->x_center - 0.5f * a->width;
  const float32_t ax1 = a->x_center + 0.5f * a->width;
  const float32_t ay0 = a->y_center - 0.5f * a->height;
  const float32_t ay1 = a->y_center + 0.5f * a->height;
  const float32_t bx0 = b->x_center - 0.5f * b->width;
  const float32_t bx1 = b->x_center + 0.5f * b->width;
  const float32_t by0 = b->y_center - 0.5f * b->height;
  const float32_t by1 = b->y_center + 0.5f * b->height;

  const float32_t aa = (ax1 - ax0) * (ay1 - ay0);
  const float32_t ba = (bx1 - bx0) * (by1 - by0);
  if (aa <= 0.f || ba <= 0.f) return 0.f;

  const float32_t ix0 = (ax0 > bx0) ? ax0 : bx0;
  const float32_t iy0 = (ay0 > by0) ? ay0 : by0;
  const float32_t ix1 = (ax1 < bx1) ? ax1 : bx1;
  const float32_t iy1 = (ay1 < by1) ? ay1 : by1;
  const float32_t iw = (ix1 > ix0) ? (ix1 - ix0) : 0.f;
  const float32_t ih = (iy1 > iy0) ? (iy1 - iy0) : 0.f;
  const float32_t inter = iw * ih;

  const float32_t uni = aa + ba - inter;
  return (uni > 0.f) ? (inter / uni) : 0.f;
}

int32_t app_postprocess_pd_ui_init(float conf_threshold, float iou_threshold)
{
  pd_build_anchors(s_anchors);

  pd_pp_params.width           = (uint32_t)AI_PD_PP_INPUT_SIZE;
  pd_pp_params.height          = (uint32_t)AI_PD_PP_INPUT_SIZE;
  pd_pp_params.nb_keypoints    = (uint32_t)AI_PD_PP_NB_KEYPOINTS;
  pd_pp_params.conf_threshold  = conf_threshold;
  pd_pp_params.iou_threshold   = iou_threshold;
  pd_pp_params.nb_total_boxes  = (uint32_t)AI_PD_PP_TOTAL_BOXES;
  pd_pp_params.max_boxes_limit = (uint32_t)AI_PD_PP_MAX_BOXES_LIMIT;
  pd_pp_params.pAnchors        = s_anchors;
  /* Not used by the float path. */
  pd_pp_params.boxe_scale      = 1.0f;
  pd_pp_params.proba_scale     = 1.0f;
  pd_pp_params.boxe_zp         = 0;
  pd_pp_params.proba_zp        = 0;

  s_pp_out.pOutData = s_boxes;
  s_pp_out.box_nb   = 0u;

  pd_ui_nb_detections = 0u;
  memset(pd_ui_detections, 0, sizeof(pd_ui_detections));

  return pd_model_pp_reset(&pd_pp_params);
}

int32_t app_postprocess_pd_ui_run(NN_Instance_TypeDef *instance)
{
  const LL_Buffer_InfoTypeDef *nn_out = LL_ATON_Output_Buffers_Info(instance);

  pd_ui_nb_detections = 0u;
  if (nn_out == NULL) return AI_PD_POSTPROCESS_ERROR;

  /* A NULL name ends the array. */
  if (nn_out[0].name == NULL || nn_out[1].name == NULL) return AI_PD_POSTPROCESS_ERROR;

  /* Outputs: [1,2016,18] float32, a box and 7 keypoints per anchor, and
   * [1,2016,1] score logits. The larger one is the boxes. */
  const uint8_t *b0 = LL_Buffer_addr_start(&nn_out[0]);
  const uint8_t *b1 = LL_Buffer_addr_start(&nn_out[1]);
  if (b0 == NULL || b1 == NULL) return AI_PD_POSTPROCESS_ERROR;

  const uint32_t n0 = (uint32_t)(LL_Buffer_addr_end(&nn_out[0]) - b0);
  const uint32_t n1 = (uint32_t)(LL_Buffer_addr_end(&nn_out[1]) - b1);

  pd_model_pp_in_t pp_in;
  if (n0 >= n1) { pp_in.pBoxes = (void *)b0; pp_in.pProbs = (void *)b1; }
  else          { pp_in.pBoxes = (void *)b1; pp_in.pProbs = (void *)b0; }

  /* Rebound every run: the library's NMS copies pKps between slots. */
  for (uint32_t i = 0u; i < (uint32_t)AI_PD_PP_MAX_BOXES_LIMIT; i++) {
    s_boxes[i].pKps = &s_kps[i * (uint32_t)AI_PD_PP_NB_KEYPOINTS];
  }

  s_pp_out.pOutData = s_boxes;
  s_pp_out.box_nb   = 0u;

  const int32_t ret = pd_model_pp_process(&pp_in, &s_pp_out, &pd_pp_params);
  if (ret != AI_PD_POSTPROCESS_ERROR_NO) return ret;

  /* box_nb is the pre-NMS count, so the greedy IoU pass is repeated here. The
   * library sorts the boxes by descending probability. */
  uint32_t cand = s_pp_out.box_nb;
  if (cand > (uint32_t)AI_PD_PP_MAX_BOXES_LIMIT) cand = (uint32_t)AI_PD_PP_MAX_BOXES_LIMIT;

  for (uint32_t i = 0u; i < cand && pd_ui_nb_detections < (uint32_t)AI_PD_PP_MAX_DETECTIONS; i++) {
    const pd_pp_box_t *cb = &s_boxes[i];

    bool suppressed = false;
    for (uint32_t k = 0u; k < pd_ui_nb_detections; k++) {
      pd_pp_box_t kept;
      kept.x_center = pd_ui_detections[k].x_center;
      kept.y_center = pd_ui_detections[k].y_center;
      kept.width    = pd_ui_detections[k].width;
      kept.height   = pd_ui_detections[k].height;
      if (pd_ui_iou(cb, &kept) >= pd_pp_params.iou_threshold) { suppressed = true; break; }
    }
    if (suppressed) continue;

    pd_ui_detection_t *d = &pd_ui_detections[pd_ui_nb_detections];
    d->prob     = cb->prob;
    d->x_center = cb->x_center;
    d->y_center = cb->y_center;
    d->width    = cb->width;
    d->height   = cb->height;
    if (cb->pKps != NULL) {
      for (uint32_t j = 0u; j < (uint32_t)AI_PD_PP_NB_KEYPOINTS; j++) {
        d->kps[j].x = cb->pKps[j].x;
        d->kps[j].y = cb->pKps[j].y;
      }
    }
    pd_ui_nb_detections++;
  }

  return AI_PD_POSTPROCESS_ERROR_NO;
}
