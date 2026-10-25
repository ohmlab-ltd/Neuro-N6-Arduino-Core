/**
 * @file    vnd_meta_yolov8_Iseg.h
 * @brief   Instance segmentation metadata builder.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef VND_META_YOLOV8_ISEG_H
#define VND_META_YOLOV8_ISEG_H

#include "app_postprocess_iseg_yolo_v8_ui.h"
#include "vnd_meta.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Writes a META_ISEG_MASK and a META_ISEG_CLASSSTR per object, and on
 * VND_META_SLOT_MODEL_0 also META_INFERENCE_MS and META_BOXES. */
void vnd_meta_build_iseg_on_slot(const iseg_pp_out_t *out, uint32_t inference_ms,
                                 const iseg_yolov8_pp_static_param_t *pp_params,
                                 vnd_meta_slot_t slot);

/* Builds from pp_output, pp_params and ts[1] - ts[0]. */
void Display_NS_Yolo_V8_Iseg_OnSlot(vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* VND_META_YOLOV8_ISEG_H */
