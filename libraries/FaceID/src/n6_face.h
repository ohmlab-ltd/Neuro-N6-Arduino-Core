/**
 ******************************************************************************
 * @file    n6_face.h
 * @brief   Face detection and face embedding as two post-processors.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6_FACE_H
#define N6_FACE_H

#include <stdint.h>
#include <stdbool.h>

#include "NeuroN6_IPostProcessor.h"
#include "NeuroN6_ROI.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Landmarks, in the detector's order. */
enum {
  N6_FACE_LM_EYE_L = 0,
  N6_FACE_LM_EYE_R = 1,
  N6_FACE_LM_NOSE  = 2,
  N6_FACE_LM_MOUTH_L = 3,
  N6_FACE_LM_MOUTH_R = 4,
  N6_FACE_NUM_LM = 5
};

/* Embedding width. A model with another output size is rejected. */
#define N6_FACE_EMB_DIM   128

/* Most faces reported from one frame. */
#ifndef N6_FACE_MAX
#define N6_FACE_MAX       8
#endif

/* One detected face. Coordinates are camera-normalised, 0..1. */
typedef struct {
  float cx, cy;                       /* box centre */
  float w,  h;                        /* box size */
  float score;                        /* detector confidence, 0..1 */
  float lm_x[N6_FACE_NUM_LM];         /* landmarks */
  float lm_y[N6_FACE_NUM_LM];
} n6_face_t;

/* Detector */

/* Post-processor for the face detector, for Vision.runWith(). min_score
 * gates the heatmap, 0.5 by default. */
IPostProcessor *n6_face_detector(void);
void            n6_face_detector_set_threshold(float min_score);

/* Results of the most recent detector pass, valid until the next. */
int              n6_face_count(void);
const n6_face_t *n6_face_get(int i);

/* Highest heatmap value in the last pass, above the threshold or not. */
float            n6_face_last_peak(void);

/* Alignment */

/* Crop that fits the eye landmarks to the ArcFace 112x112 template by
 * translation and scale, without rotation. Falls back to an expanded box
 * when the eye landmarks are unusable. */
n6_roi_t n6_face_align_roi(const n6_face_t *f);

/* Embedder */

/* Post-processor for the recognition model, for Vision.runRoi(). */
IPostProcessor *n6_face_embedder(void);

/* The embedding from the most recent embedder pass, L2-normalised. NULL, and
 * false, when that pass failed. */
bool         n6_face_embedding_valid(void);
const float *n6_face_embedding(void);

/* Camera-normalised 0..1 to LCD pixels, independent of the NN's crop. */
int n6_face_cam_to_lcd_x(float xn);
int n6_face_cam_to_lcd_y(float yn);
int n6_face_cam_to_lcd_w(float wn);
int n6_face_cam_to_lcd_h(float hn);

#ifdef __cplusplus
}
#endif

#endif /* N6_FACE_H */
