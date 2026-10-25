/**
 ******************************************************************************
 * @file    PostProcessor_NIA416.h
 * @brief   Singleton IPostProcessor factory for the Nia 416x416 detector.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef POSTPROCESSOR_NIA416_H
#define POSTPROCESSOR_NIA416_H

#include "NeuroN6_IPostProcessor.h"
#include "vnd_meta.h"

#ifdef __cplusplus
extern "C" {
#endif

/* iou is the class-aware NMS threshold. Requires an export calibrated for
 * [0,1] input: scale 1/255, zp -128. */
IPostProcessor *PostProcess_NIA416(float conf, float iou);

/* The same, publishing to slot instead of VND_META_SLOT_PRIMARY. */
IPostProcessor *PostProcess_NIA416_OnSlot(float conf, float iou,
                                          vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* POSTPROCESSOR_NIA416_H */
