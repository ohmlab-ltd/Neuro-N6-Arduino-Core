/**
 ******************************************************************************
 * @file    n6_apriltag_pose.h
 * @brief   Tag pose from its corners and a stored camera calibration.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6_APRILTAG_POSE_H
#define N6_APRILTAG_POSE_H

#include <stdint.h>
#include <stdbool.h>
#include "n6_apriltag.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Pinhole intrinsics and 5-term distortion, as stored in the asset. */
#define N6_CAMERA_CAL_MAGIC   0x4C41434Eu   /* 'NCAL' little-endian */
#define N6_CAMERA_CAL_VERSION 1u

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t width, height;   /* resolution calibrated at */
  float fx, fy, cx, cy;     /* pixels, at that resolution */
  float k1, k2, p1, p2, k3; /* radial and tangential distortion */
} n6_camera_cal_t;

/* Tag pose in the camera frame. */
typedef struct {
  float tx, ty, tz;         /* metres, tz along the optical axis */
  float rx, ry, rz;         /* Euler XYZ, degrees */
} n6_apriltag_pose_t;

/* Loads the stored calibration. False when the board has none. */
bool n6_camera_cal_load(n6_camera_cal_t *out);

/* Stages a calibration as the asset and resets so the FSBL programs it. Does
 * not return on success. */
bool n6_camera_cal_store(const n6_camera_cal_t *cal);

void n6_camera_cal_scale(const n6_camera_cal_t *in, int w, int h,
                         n6_camera_cal_t *out);

/* Pose of a tag found in a win_w x win_h image. cal must be scaled to that
 * size, and tag_size_m is the black square's side in metres. Returns false
 * on degenerate corners. */
bool n6_apriltag_pose(const n6_apriltag_t *tag, int win_w, int win_h,
                      const n6_camera_cal_t *cal, float tag_size_m,
                      n6_apriltag_pose_t *out);

#ifdef __cplusplus
}
#endif

#endif /* N6_APRILTAG_POSE_H */
