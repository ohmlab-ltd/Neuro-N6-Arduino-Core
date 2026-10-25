/**
 ******************************************************************************
 * @file    app_postprocess_pd_ui.h
 * @brief   MediaPipe palm-detector post-processing.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef APP_POSTPROCESS_PD_UI_H
#define APP_POSTPROCESS_PD_UI_H

#include <stdint.h>
#include <stdbool.h>

/* Outside extern "C" so arm_mve.h keeps its C++ overloads. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "pd_model_pp_if.h"
#include "ll_aton_rt_user_api.h"

/* A #define in a sketch does not reach the library. */

/* NN input side. Matches cfg.nn.width and cfg.nn.height. */
#ifndef AI_PD_PP_INPUT_SIZE
#define AI_PD_PP_INPUT_SIZE         (192)
#endif

/* Anchors: 24*24*2 + 12*12*6. */
#ifndef AI_PD_PP_TOTAL_BOXES
#define AI_PD_PP_TOTAL_BOXES        (2016)
#endif

/* Keypoints per palm: wrist, 4 knuckles, 2 thumb. */
#ifndef AI_PD_PP_NB_KEYPOINTS
#define AI_PD_PP_NB_KEYPOINTS       (7)
#endif

/* Pre-NMS candidate cap. */
#ifndef AI_PD_PP_MAX_BOXES_LIMIT
#define AI_PD_PP_MAX_BOXES_LIMIT    (48)
#endif

/* Post-NMS detection cap. */
#ifndef AI_PD_PP_MAX_DETECTIONS
#define AI_PD_PP_MAX_DETECTIONS     (8)
#endif

#ifndef AI_PD_PP_CONF_THRESHOLD
#define AI_PD_PP_CONF_THRESHOLD     (0.5f)
#endif

#ifndef AI_PD_PP_IOU_THRESHOLD
#define AI_PD_PP_IOU_THRESHOLD      (0.3f)
#endif

/* Geometry is 0..1 within the NN input. */
typedef struct {
  float32_t x;
  float32_t y;
} pd_ui_point_t;

typedef struct {
  float32_t     prob;   /* 0..1 */
  float32_t     x_center, y_center;
  float32_t     width, height;
  pd_ui_point_t kps[AI_PD_PP_NB_KEYPOINTS];
} pd_ui_detection_t;

/* Latest results, valid until the next app_postprocess_pd_ui_run. */
extern pd_ui_detection_t          pd_ui_detections[AI_PD_PP_MAX_DETECTIONS];
extern uint32_t                   pd_ui_nb_detections;
extern pd_model_pp_static_param_t pd_pp_params;

int32_t app_postprocess_pd_ui_init(float conf_threshold, float iou_threshold);
int32_t app_postprocess_pd_ui_run(NN_Instance_TypeDef *instance);

#ifdef __cplusplus
}
#endif

#endif /* APP_POSTPROCESS_PD_UI_H */
