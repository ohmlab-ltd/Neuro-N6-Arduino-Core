/**
 * @file    vnd_meta_nia.h
 * @brief   Nia detection metadata builder.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef VND_META_NIA_H
#define VND_META_NIA_H

#include <stdint.h>

/* Included outside extern "C", before nia_pp.h includes it inside. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "vnd_meta.h"
#include "nia_pp.h"

/* Writes a META_ISEG_CLASSSTR per detection, and on VND_META_SLOT_MODEL_0 also
 * META_INFERENCE_MS and META_BOXES. inference_ms is the NPU time in ms. */
void vnd_meta_build_nia_on_slot(const nia_pp_out_t *out, uint32_t inference_ms,
                                const nia_pp_params_t *params,
                                vnd_meta_slot_t slot);

/* Builds from nia_pp_output, nia_pp_params and ts[1] - ts[0]. */
void Display_NS_Nia_OnSlot(vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* VND_META_NIA_H */
