/**
 * @file    yolox_pp.h
 * @brief   Post-processing for PixelKit-trained YOLOX detectors.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef YOLOX_PP_H
#define YOLOX_PP_H

#include <stdint.h>
#include <stdbool.h>

/* Outside extern "C" so arm_mve.h keeps its C++ overloads. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "ll_aton_rt_user_api.h"

/* Thresholded candidates fed to NMS. */
#ifndef YOLOX_PP_MAX_CAND
#define YOLOX_PP_MAX_CAND  (256)
#endif

typedef struct {
  float32_t x_center;   /* 0..1 of the model input square */
  float32_t y_center;
  float32_t width;
  float32_t height;
  float32_t conf;       /* objectness * class score */
  int32_t   class_index;
} yolox_det_t;

typedef struct yolox_pp_out {
  yolox_det_t *pOutBuff;   /* caller-provided array, >= max_det entries */
  int32_t      nb_detect;
} yolox_pp_out_t;

typedef struct {
  float32_t conf_threshold;
  float32_t iou_threshold;
  int32_t   max_det;
  int32_t   nb_classes;    /* class count */
  float32_t input_dim;     /* input side in pixels, square */
} yolox_pp_params_t;

/* Output tensor: N rows of dx, dy, dw, dh, objectness, then a score per class.
 * The box terms are in grid units and the scores are already sigmoided.
 * N = (dim/8)^2 + (dim/16)^2 + (dim/32)^2, ordered stride 8, 16, 32. */

/**
 * @brief  Decodes the network's single output into detections.
 * @param  outs  from LL_ATON_Output_Buffers_Info().
 * @return 0, -1 bad arguments, -2 no output tensor, -3 output shape does not
 *         match params.
 */
int32_t yolox_pp_process(const LL_Buffer_InfoTypeDef *outs,
                         yolox_pp_out_t *out,
                         const yolox_pp_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* YOLOX_PP_H */
