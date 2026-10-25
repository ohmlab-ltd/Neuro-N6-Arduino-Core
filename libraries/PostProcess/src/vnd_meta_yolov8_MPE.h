/**
 * @file    vnd_meta_yolov8_MPE.h
 * @brief   Multi-person pose estimation metadata builder.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef VND_META_YOLOV8_MPE_H
#define VND_META_YOLOV8_MPE_H

#include "app_postprocess_mpe_yolo_v8_ui.h"
#include "vnd_meta.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Writes META_KEYPOINTS and META_BINDINGS, and on VND_META_SLOT_MODEL_0 also
 * META_INFERENCE_MS and META_BOXES. */
void vnd_meta_build_mpe_on_slot(const mpe_pp_out_t *out, uint32_t inference_ms,
                                const mpe_yolov8_pp_static_param_t *pp_params,
                                vnd_meta_slot_t slot);

/* Builds from mpe_pp_output, mpe_pp_params and ts[1] - ts[0]. */
void Display_NS_Yolo_V8_MPE_OnSlot(vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* VND_META_YOLOV8_MPE_H */
