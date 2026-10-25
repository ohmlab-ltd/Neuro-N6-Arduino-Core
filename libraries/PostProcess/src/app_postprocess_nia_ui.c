/**
 * @file    app_postprocess_nia_ui.c
 * @brief   Init and run wrapper for the Nia detector.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "app_postprocess_nia_ui.h"

nia_pp_out_t    nia_pp_output;
nia_pp_params_t nia_pp_params;

static nia_det_t s_out_detections[NIA_PP_MAX_DET] __attribute__((section(".ext_psram")));

int32_t app_postprocess_nia_ui_init(float conf_threshold, float iou_threshold)
{
  nia_pp_params.conf_threshold = conf_threshold;
  nia_pp_params.iou_threshold  = iou_threshold;
  nia_pp_params.max_det        = NIA_PP_MAX_DET;
  nia_pp_output.pOutBuff       = s_out_detections;
  nia_pp_output.nb_detect      = 0;
  return 0;
}

int32_t app_postprocess_nia_ui_run(NN_Instance_TypeDef *instance)
{
  const LL_Buffer_InfoTypeDef *nn_out_info = LL_ATON_Output_Buffers_Info(instance);
  nia_pp_output.pOutBuff = s_out_detections;
  return nia_pp_process(nn_out_info, &nia_pp_output, &nia_pp_params);
}
