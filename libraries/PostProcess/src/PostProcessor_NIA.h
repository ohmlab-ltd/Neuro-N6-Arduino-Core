/**
 ******************************************************************************
 * @file    PostProcessor_NIA.h
 * @brief   Singleton IPostProcessor factory for the Nia detector.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef POSTPROCESSOR_NIA_H
#define POSTPROCESSOR_NIA_H

#include "NeuroN6_IPostProcessor.h"
#include "vnd_meta.h"

#ifdef __cplusplus
extern "C" {
#endif

/* conf is the confidence threshold. iou is ignored. */
IPostProcessor *PostProcess_NIA(float conf, float iou);

/* The same, publishing to slot instead of VND_META_SLOT_PRIMARY. */
IPostProcessor *PostProcess_NIA_OnSlot(float conf, float iou, vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* POSTPROCESSOR_NIA_H */
