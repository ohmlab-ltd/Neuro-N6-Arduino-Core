/**
 ******************************************************************************
 * @file    PostProcessor_Zoo.h
 * @brief   Singleton IPostProcessor factories for ST's model zoo, PixelKit
 *          detectors and image classifiers.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef POSTPROCESSOR_ZOO_H
#define POSTPROCESSOR_ZOO_H

#include <stdint.h>

/* Outside extern "C" so arm_mve.h keeps its C++ overloads. */
#include "arm_math_types.h"

#include "NeuroN6_IPostProcessor.h"
#include "vnd_meta.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "od_pp_output_if.h"

/* _Ex variants: NULL labels give the COCO-80 names for an 80-class model and
 * "class N" otherwise, and "face" for BlazeFace. The other variants publish to
 * VND_META_SLOT_MODEL_0. */

/* Object detectors */

/* conf is the lowest score kept, 0..1. iou is the overlap above which the
 * weaker of two boxes of the same class is dropped. */

/* Ultralytics detection head, one output [4 + classes][boxes]: YOLOv8, YOLO11,
 * YOLOv5u, and YOLO26 as ST's zoo exports it. */
IPostProcessor *PostProcess_YOLOv8Detect(float conf, float iou);
IPostProcessor *PostProcess_YOLOv8Detect_Ex(float conf, float iou,
                                            const char *const *labels,
                                            int nlabels, vnd_meta_slot_t slot);

/* ST's YOLOX, st_yoloxn: three outputs at strides 8, 16 and 32, one anchor
 * each, ST's default anchor size. */
IPostProcessor *PostProcess_STYoloX(float conf, float iou);
IPostProcessor *PostProcess_STYoloX_Ex(float conf, float iou,
                                       const char *const *labels,
                                       int nlabels, vnd_meta_slot_t slot);

/* YOLOX-Nano or YOLOX-Tiny trained in PixelKit: one output, a row per cell of
 * the stride 8, 16 and 32 grids. Names must match the model's class count. */
IPostProcessor *PostProcess_PixelKitYOLOX(float conf, float iou,
                                          const char *const *labels,
                                          int nlabels);
IPostProcessor *PostProcess_PixelKitYOLOX_Ex(float conf, float iou,
                                             const char *const *labels,
                                             int nlabels, vnd_meta_slot_t slot);

/* Tiny YOLOv2, yolov2t and st_yololcv1: one output, five anchors with ST's
 * default sizes. */
IPostProcessor *PostProcess_TinyYOLOv2(float conf, float iou);
IPostProcessor *PostProcess_TinyYOLOv2_Ex(float conf, float iou,
                                          const char *const *labels,
                                          int nlabels, vnd_meta_slot_t slot);

/* SSD MobileNet from ST's PyTorch zoo, 300x300 input, 3000 priors: class
 * scores with the background first, and box offsets. */
IPostProcessor *PostProcess_SSD(float conf, float iou);
IPostProcessor *PostProcess_SSD_Ex(float conf, float iou,
                                   const char *const *labels, int nlabels,
                                   vnd_meta_slot_t slot);

/* BlazeFace front-camera face detector, blazeface_front_128. Boxes only. */
IPostProcessor *PostProcess_BlazeFace(float conf, float iou);
IPostProcessor *PostProcess_BlazeFace_Ex(float conf, float iou,
                                         const char *const *labels,
                                         int nlabels, vnd_meta_slot_t slot);

/* Pose */

/* MoveNet single-person heatmaps, 17 COCO keypoints. conf is the lowest
 * keypoint score drawn, and a person needs three keypoints that reach it. */
IPostProcessor *PostProcess_MoveNet(float conf);
IPostProcessor *PostProcess_MoveNet_Ex(float conf, vnd_meta_slot_t slot);

/* Segmentation */

/* One output [classes][H][W] or [H][W][classes], class 0 the background. Each
 * class present is reported as a box, scored by its share of the picture. */
IPostProcessor *PostProcess_DeepLabV3(void);
IPostProcessor *PostProcess_DeepLabV3_Ex(const char *const *labels,
                                         int nlabels, vnd_meta_slot_t slot);

/* Image classification */

/* One output of per-class scores, probabilities or logits. Reports up to 5
 * classes scoring at least conf. */
IPostProcessor *PostProcess_Classifier(float conf, const char *const *labels,
                                       int nlabels);
/* top_k is the number of classes reported, 1 to 8. */
IPostProcessor *PostProcess_Classifier_Ex(float conf, const char *const *labels,
                                          int nlabels, int top_k,
                                          vnd_meta_slot_t slot);

/* Results */

/* Points *boxes at the latest boxes, 0..1 of the picture the network saw,
 * valid until the next frame. Returns the count, 0 for the classifier. */
int PostProcess_ZooResults(const IPostProcessor *pp, const od_pp_outBuffer_t **boxes);

/* The classifier's best class and its score, or -1 when nothing reached conf. */
int PostProcess_ClassifierTop(const IPostProcessor *pp, float *score);

/* 0 while the model's outputs decode, else a negative ZOO_PP_ERR_* code. */
int PostProcess_ZooStatus(const IPostProcessor *pp);

#ifdef __cplusplus
}
#endif

#endif /* POSTPROCESSOR_ZOO_H */
