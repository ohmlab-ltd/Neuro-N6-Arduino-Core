/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef APP_POSTPROCESS_ISEG_YOLO_V8_UI_H
#define APP_POSTPROCESS_ISEG_YOLO_V8_UI_H

#include <stdint.h>
#include <stdbool.h>

/* Outside extern "C" so arm_mve.h keeps its C++ overloads. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "iseg_yolov8_pp_if.h"
#include "ll_aton_rt_user_api.h"

#ifndef NB_CLASSES
#define NB_CLASSES (80)
#endif

/* Weak default. A sketch can define its own classes_table. */
extern const char* classes_table[NB_CLASSES];

#ifndef AI_YOLOV8_SEG_PP_TOTAL_BOXES
#define AI_YOLOV8_SEG_PP_TOTAL_BOXES       (1344)
#endif

#ifndef AI_YOLOV8_SEG_PP_NB_CLASSES
#define AI_YOLOV8_SEG_PP_NB_CLASSES        (NB_CLASSES)
#endif

#ifndef AI_YOLOV8_SEG_PP_MASK_NB
#define AI_YOLOV8_SEG_PP_MASK_NB           (32)
#endif

#ifndef AI_YOLOV8_SEG_PP_MASK_SIZE
#define AI_YOLOV8_SEG_PP_MASK_SIZE         (64)
#endif

#ifndef AI_YOLOV8_SEG_ZERO_POINT
#define AI_YOLOV8_SEG_ZERO_POINT           (25)
#endif

#ifndef AI_YOLOV8_SEG_SCALE
#define AI_YOLOV8_SEG_SCALE                (0.020020058378577232f)
#endif

#ifndef AI_YOLOV8_SEG_MASK_ZERO_POINT
#define AI_YOLOV8_SEG_MASK_ZERO_POINT      (-115)
#endif

#ifndef AI_YOLOV8_SEG_MASK_SCALE
#define AI_YOLOV8_SEG_MASK_SCALE           (0.0207486841827631f)
#endif

#ifndef AI_YOLOV8_SEG_PP_CONF_THRESHOLD
#define AI_YOLOV8_SEG_PP_CONF_THRESHOLD    (0.5f)
#endif

#ifndef AI_YOLOV8_SEG_PP_IOU_THRESHOLD
#define AI_YOLOV8_SEG_PP_IOU_THRESHOLD     (0.4f)
#endif

#ifndef AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT
#define AI_YOLOV8_SEG_PP_MAX_BOXES_LIMIT   (10)
#endif

extern iseg_pp_out_t pp_output;
extern iseg_yolov8_pp_static_param_t pp_params;

int32_t app_postprocess_iseg_yolo_v8_ui_init(float conf_threshold, float iou_threshold);
int32_t app_postprocess_iseg_yolo_v8_ui_run(NN_Instance_TypeDef *instance);

#ifdef __cplusplus
}
#endif

#endif /* APP_POSTPROCESS_ISEG_YOLO_V8_UI_H */
