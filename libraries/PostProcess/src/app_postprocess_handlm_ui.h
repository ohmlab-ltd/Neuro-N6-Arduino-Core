/**
 ******************************************************************************
 * @file    app_postprocess_handlm_ui.h
 * @brief   MediaPipe hand-landmark post-processing.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef APP_POSTPROCESS_HANDLM_UI_H
#define APP_POSTPROCESS_HANDLM_UI_H

#include <stdint.h>
#include <stdbool.h>

/* Outside extern "C" so arm_mve.h keeps its C++ overloads. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "ll_aton_rt_user_api.h"

#ifndef AI_HANDLM_NB_LANDMARKS
#define AI_HANDLM_NB_LANDMARKS   (21)
#endif

/* Model input side in pixels. */
#ifndef AI_HANDLM_INPUT_SIZE
#define AI_HANDLM_INPUT_SIZE     (224.0f)
#endif

/* Minimum presence score. The bundled model's score does not exceed 0.5. */
#ifndef AI_HANDLM_MIN_SCORE
#define AI_HANDLM_MIN_SCORE      (0.5f)
#endif

typedef struct {
  float32_t x;      /* 0..1 within the model input */
  float32_t y;
  float32_t z;      /* relative depth, wrist = 0, negative = toward camera */
} handlm_point_t;

typedef struct {
  bool           valid;                              /* score >= min_score  */
  float32_t      score;                              /* hand presence 0..1  */
  float32_t      handedness;                         /* 0 = left, 1 = right */
  handlm_point_t kps[AI_HANDLM_NB_LANDMARKS];
} handlm_result_t;

/* Latest result, valid until the next app_postprocess_handlm_ui_run. */
extern handlm_result_t handlm_output;
extern float32_t       handlm_min_score;

int32_t app_postprocess_handlm_ui_init(float min_score);
int32_t app_postprocess_handlm_ui_run(NN_Instance_TypeDef *instance);

#ifdef __cplusplus
}
#endif

#endif /* APP_POSTPROCESS_HANDLM_UI_H */
