/**
 * @file    vnd_meta_handlm.h
 * @brief   Hand-landmark metadata: META_KEYPOINTS and META_BINDINGS.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef VND_META_HANDLM_H
#define VND_META_HANDLM_H

#include <stdint.h>
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "vnd_meta.h"
#include "app_postprocess_handlm_ui.h"

void vnd_meta_build_handlm_on_slot(const handlm_result_t *res,
                                   uint32_t inference_ms, vnd_meta_slot_t slot);

void Display_NS_HandLandmarks_OnSlot(vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* VND_META_HANDLM_H */
