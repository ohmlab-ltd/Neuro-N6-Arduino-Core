/**
 * @file    vnd_meta_pd.h
 * @brief   Palm-detector metadata builder.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef VND_META_PD_H
#define VND_META_PD_H

#include <stdint.h>
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "vnd_meta.h"
#include "app_postprocess_pd_ui.h"

/* Writes META_KEYPOINTS and META_BINDINGS, and on VND_META_SLOT_MODEL_0 also
 * META_INFERENCE_MS and META_BOXES. */
void vnd_meta_build_pd_on_slot(const pd_ui_detection_t *dets, uint32_t nb_dets,
                               uint32_t inference_ms, vnd_meta_slot_t slot);

/* Builds from pd_ui_detections and ts[1] - ts[0]. */
void Display_NS_PalmDetector_OnSlot(vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* VND_META_PD_H */
