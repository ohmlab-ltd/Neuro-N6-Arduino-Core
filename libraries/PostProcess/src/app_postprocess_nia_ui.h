/**
 * @file    app_postprocess_nia_ui.h
 * @brief   Init and run wrapper for the Nia detector.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef APP_POSTPROCESS_NIA_UI_H
#define APP_POSTPROCESS_NIA_UI_H

#include <stdint.h>
#include <stdbool.h>

/* Included before extern "C": nia_pp.h includes it again from inside. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "nia_pp.h"
#include "ll_aton_rt_user_api.h"

extern nia_pp_out_t    nia_pp_output;
extern nia_pp_params_t nia_pp_params;

/** Keeps up to NIA_PP_MAX_DET detections. iou_threshold is unused. */
int32_t app_postprocess_nia_ui_init(float conf_threshold, float iou_threshold);

/** Fills nia_pp_output. Returns 0 on success. */
int32_t app_postprocess_nia_ui_run(NN_Instance_TypeDef *instance);

#ifdef __cplusplus
}
#endif

#endif /* APP_POSTPROCESS_NIA_UI_H */
