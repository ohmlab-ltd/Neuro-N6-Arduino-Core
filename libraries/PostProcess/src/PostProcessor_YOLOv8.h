/**
 ******************************************************************************
 * @file    PostProcessor_YOLOv8.h
 * @brief   Singleton IPostProcessor factories for the bundled YOLOv8 models.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef POSTPROCESSOR_YOLOV8_H
#define POSTPROCESSOR_YOLOV8_H

#include "NeuroN6_IPostProcessor.h"
#include "vnd_meta.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Instance segmentation. */
IPostProcessor *PostProcess_YOLOv8_ISEG(float conf, float iou);

/* The same, publishing to slot instead of VND_META_SLOT_PRIMARY. */
IPostProcessor *PostProcess_YOLOv8_ISEG_OnSlot(float conf, float iou,
                                               vnd_meta_slot_t slot);

/* Multi-person pose estimation. */
IPostProcessor *PostProcess_YOLOv8_MPE(float conf, float iou);

/* The same, publishing to slot instead of VND_META_SLOT_PRIMARY. */
IPostProcessor *PostProcess_YOLOv8_MPE_OnSlot(float conf, float iou,
                                              vnd_meta_slot_t slot);

#ifdef __cplusplus
}
#endif

#endif /* POSTPROCESSOR_YOLOV8_H */
