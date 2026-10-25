/**
 * @file    app_postprocess_nia416_ui.h
 * @brief   Init and run wrapper for the Nia 416x416 detector.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef APP_POSTPROCESS_NIA416_UI_H
#define APP_POSTPROCESS_NIA416_UI_H

#include <stdint.h>
#include <stdbool.h>

#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "nia416_pp.h"
#include "ll_aton_rt_user_api.h"

extern nia_pp_out_t    nia416_pp_output;
extern nia_pp_params_t nia416_pp_params;

int32_t app_postprocess_nia416_ui_init(float conf_threshold, float iou_threshold);
int32_t app_postprocess_nia416_ui_run(NN_Instance_TypeDef *instance);

#ifdef __cplusplus
}
#endif

#endif /* APP_POSTPROCESS_NIA416_UI_H */
