/**
 ******************************************************************************
 * @file    app_postprocess_mpe_yolo_v8_ui.h
 * @brief   YOLOv8 multi-person pose post-processing wrapper.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef APP_POSTPROCESS_MPE_YOLO_V8_UI_H
#define APP_POSTPROCESS_MPE_YOLO_V8_UI_H

#include <stdint.h>
#include <stdbool.h>

/* Outside extern "C" so arm_mve.h keeps its C++ overloads. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "mpe_yolov8_pp_if.h"
#include "ll_aton_rt_user_api.h"

#ifndef AI_MPE_YOLOV8_PP_NB_CLASSES
#define AI_MPE_YOLOV8_PP_NB_CLASSES         (1)
#endif

#ifndef AI_MPE_YOLOV8_PP_TOTAL_BOXES
#define AI_MPE_YOLOV8_PP_TOTAL_BOXES        (1344)
#endif

#ifndef AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT
#define AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT    (10)
#endif

#ifndef AI_MPE_YOLOV8_PP_CONF_THRESHOLD
#define AI_MPE_YOLOV8_PP_CONF_THRESHOLD     (0.4f)
#endif

#ifndef AI_MPE_YOLOV8_PP_IOU_THRESHOLD
#define AI_MPE_YOLOV8_PP_IOU_THRESHOLD      (0.5f)
#endif

#ifndef AI_POSE_PP_POSE_KEYPOINTS_NB
#define AI_POSE_PP_POSE_KEYPOINTS_NB        (17)
#endif

extern mpe_pp_out_t                   mpe_pp_output;
extern mpe_yolov8_pp_static_param_t   mpe_pp_params;

/* Both return 0, or a negative value on error. */
int32_t app_postprocess_mpe_yolo_v8_ui_init(float conf_threshold, float iou_threshold);
int32_t app_postprocess_mpe_yolo_v8_ui_run(NN_Instance_TypeDef *instance);

#ifdef __cplusplus
}
#endif

#endif /* APP_POSTPROCESS_MPE_YOLO_V8_UI_H */
