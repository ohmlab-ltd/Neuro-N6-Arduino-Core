/**
 ******************************************************************************
 * @file    PostProcessor_HandLM.h
 * @brief   Singleton IPostProcessor factory for MediaPipe hand landmarks.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef POSTPROCESSOR_HANDLM_H
#define POSTPROCESSOR_HANDLM_H

#include "NeuroN6_IPostProcessor.h"
#include "NeuroN6_ROI.h"
#include "vnd_meta.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "app_postprocess_pd_ui.h"
#include "app_postprocess_handlm_ui.h"

/* min_score is the hand-presence threshold. The bundled model's score does not
 * exceed 0.5. */
IPostProcessor *PostProcess_HandLandmarks(float min_score);

/* The same, publishing to slot instead of VND_META_SLOT_PRIMARY. */
IPostProcessor *PostProcess_HandLandmarks_OnSlot(float min_score,
                                                 vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* POSTPROCESSOR_HANDLM_H */
