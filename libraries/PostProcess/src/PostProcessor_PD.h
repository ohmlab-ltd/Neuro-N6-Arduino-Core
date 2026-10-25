/**
 ******************************************************************************
 * @file    PostProcessor_PD.h
 * @brief   Singleton IPostProcessor factory for the MediaPipe palm detector.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef POSTPROCESSOR_PD_H
#define POSTPROCESSOR_PD_H

#include "NeuroN6_IPostProcessor.h"
#include "vnd_meta.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Model: palm_detection_full, 192x192 uint8 input, outputs [1,2016,1] scores
 * and [1,2016,18] boxes with landmarks. */
IPostProcessor *PostProcess_PalmDetector(float conf, float iou);

/* The same, publishing to slot instead of VND_META_SLOT_PRIMARY. */
IPostProcessor *PostProcess_PalmDetector_OnSlot(float conf, float iou,
                                                vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* POSTPROCESSOR_PD_H */
