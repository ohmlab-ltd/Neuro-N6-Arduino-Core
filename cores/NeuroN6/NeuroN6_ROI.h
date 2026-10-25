/**
 ******************************************************************************
 * @file    NeuroN6_ROI.h
 * @brief   Region of interest for cascaded models.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef NEURON6_ROI_H
#define NEURON6_ROI_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Camera-normalised, [0..1] within the full frame. angle is clockwise in
 * radians, 0 upright. Vision.runRoi() does not rotate the pixels. */
typedef struct {
  float cx;
  float cy;
  float w;
  float h;
  float angle;
} n6_roi_t;

/* The _nn_ builders take NN-normalised coordinates, [0..1] within the crop the
 * current model is fed, and convert them with the live crop. Call them straight
 * after the run that produced the coordinates. */
n6_roi_t n6_roi_from_nn_box(float cx, float cy, float w, float h);

/* Rotates the ROI so the p0->p1 vector points at target_angle_deg, measured
 * counter-clockwise from +X. 90 is straight up. */
n6_roi_t n6_roi_from_nn_box_kps(float cx, float cy, float w, float h,
                                float x0, float y0, float x1, float y1,
                                float target_angle_deg);

/* Takes camera-normalised coordinates. */
n6_roi_t n6_roi_from_cam_box(float cx, float cy, float w, float h, float angle);

/* Shifts the box along its own axes, squares it to the longer side when
 * square_long is set, then scales it about its centre. Shifts are in units of
 * the box's size. */
n6_roi_t n6_roi_expand(n6_roi_t roi, float scale_x, float scale_y,
                       float shift_x, float shift_y, bool square_long);

/* Size of the upright rectangle that contains the rotated ROI. */
void n6_roi_aabb(const n6_roi_t *roi, float *out_w, float *out_h);

/* Maps a point in the ROI, [0..1], to camera-normalised coordinates, rotating
 * by roi->angle. For a Vision.runRoi() result, whose crop is upright, pass
 * angle 0. */
void n6_roi_unmap_point(const n6_roi_t *roi, float rx, float ry,
                        float *out_cam_x, float *out_cam_y);

typedef struct {
  /* Keypoint indices that give the direction, or both -1 for an upright ROI. */
  int8_t rot_kp_start;
  int8_t rot_kp_end;
  /* Extra endpoints averaged into rot_kp_end, -1 when unused. */
  int8_t rot_kp_end2;
  int8_t rot_kp_end3;
  /* Angle the start->end vector is rotated to, counter-clockwise from +X. */
  float  target_angle_deg;
  /* As for n6_roi_expand(). */
  float  scale_x, scale_y;
  float  shift_x, shift_y;
  bool   square_long;
} n6_roi_spec_t;

/* Builds an ROI from a detection box and optional keypoints, both
 * NN-normalised. kps holds x then y at each entry, kp_stride floats apart, and
 * may be NULL. A NULL spec gives the plain box. */
n6_roi_t n6_roi_build(float cx, float cy, float w, float h,
                      const float *kps, int kp_stride, int kp_count,
                      const n6_roi_spec_t *spec);

/* Builds an ROI from the keypoints' bounding box, then applies the spec. */
n6_roi_t n6_roi_from_nn_keypoints(const float *kps, int kp_stride, int kp_count,
                                  const n6_roi_spec_t *spec);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_ROI_H */
