/**
 ******************************************************************************
 * @file    n6_apriltag_pose.c
 * @brief   Planar pose from four corners.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "n6_apriltag_pose.h"
#include "n6_asset.h"
#include <math.h>
#include <string.h>

/* Calibration asset */

bool n6_camera_cal_load(n6_camera_cal_t *out)
{
  if (out == NULL) return false;
  uint32_t len = 0u, ver = 0u;
  /* The store holds one asset, which may belong to another subsystem. */
  if (!n6_asset_read(out, sizeof(*out), &len, &ver)) return false;
  if (len != sizeof(*out)) return false;
  if (out->magic != N6_CAMERA_CAL_MAGIC) return false;
  if (out->version != N6_CAMERA_CAL_VERSION) return false;
  if (!(out->fx > 1.0f) || !(out->fy > 1.0f)) return false;
  return true;
}

bool n6_camera_cal_store(const n6_camera_cal_t *cal)
{
  if (cal == NULL) return false;
  if (cal->magic != N6_CAMERA_CAL_MAGIC) return false;

  uint32_t ver = 0u;
  (void)n6_asset_present(&ver, NULL);
  if (!n6_asset_stage(cal, (uint32_t)sizeof(*cal), ver + 1u)) return false;
  n6_asset_apply();          /* does not return */
  return true;
}

void n6_camera_cal_scale(const n6_camera_cal_t *in, int w, int h,
                         n6_camera_cal_t *out)
{
  if (in == NULL || out == NULL || w <= 0 || h <= 0) return;
  *out = *in;
  if (in->width == 0u || in->height == 0u) return;

  /* The distortion coefficients are dimensionless and do not scale. */
  const float sx = (float)w / (float)in->width;
  const float sy = (float)h / (float)in->height;
  out->fx = in->fx * sx;  out->cx = in->cx * sx;
  out->fy = in->fy * sy;  out->cy = in->cy * sy;
  out->width  = (uint16_t)w;
  out->height = (uint16_t)h;
}

/* Geometry */

/* Undistorts one pixel to a normalised camera ray by fixed-point iteration. */
static void undistort(const n6_camera_cal_t *c, float u, float v,
                      float *xn, float *yn)
{
  const float x0 = (u - c->cx) / c->fx;
  const float y0 = (v - c->cy) / c->fy;
  float x = x0, y = y0;

  for (int i = 0; i < 5; i++) {
    const float r2 = x * x + y * y;
    const float radial = 1.0f + c->k1 * r2 + c->k2 * r2 * r2 + c->k3 * r2 * r2 * r2;
    const float dx = 2.0f * c->p1 * x * y + c->p2 * (r2 + 2.0f * x * x);
    const float dy = c->p1 * (r2 + 2.0f * y * y) + 2.0f * c->p2 * x * y;
    if (radial < 1e-6f) break;
    x = (x0 - dx) / radial;
    y = (y0 - dy) / radial;
  }
  *xn = x; *yn = y;
}

/* Solves A x = b for n = 8 by Gaussian elimination. False when singular. */
static bool solve8(float A[8][9])
{
  for (int col = 0; col < 8; col++) {
    int piv = col;
    for (int r = col + 1; r < 8; r++)
      if (fabsf(A[r][col]) > fabsf(A[piv][col])) piv = r;
    if (fabsf(A[piv][col]) < 1e-9f) return false;
    if (piv != col) for (int k = col; k < 9; k++) {
      const float t = A[col][k]; A[col][k] = A[piv][k]; A[piv][k] = t;
    }
    const float d = A[col][col];
    for (int k = col; k < 9; k++) A[col][k] /= d;
    for (int r = 0; r < 8; r++) {
      if (r == col) continue;
      const float f = A[r][col];
      if (f == 0.0f) continue;
      for (int k = col; k < 9; k++) A[r][k] -= f * A[col][k];
    }
  }
  return true;
}

bool n6_apriltag_pose(const n6_apriltag_t *tag, int win_w, int win_h,
                      const n6_camera_cal_t *cal, float tag_size_m,
                      n6_apriltag_pose_t *out)
{
  if (tag == NULL || cal == NULL || out == NULL) return false;
  if (win_w <= 0 || win_h <= 0 || !(tag_size_m > 0.0f)) return false;
  if (!(cal->fx > 1.0f) || !(cal->fy > 1.0f)) return false;

  /* Tag-frame corners in the detector's order, origin at the tag centre. */
  const float s = tag_size_m * 0.5f;
  const float obj[4][2] = { { -s,  s }, {  s,  s }, {  s, -s }, { -s, -s } };

  float img[4][2];
  for (int i = 0; i < 4; i++) {
    const float u = tag->corners[i][0] * (float)win_w;
    const float v = tag->corners[i][1] * (float)win_h;
    undistort(cal, u, v, &img[i][0], &img[i][1]);
  }

  /* Homography from the tag plane to normalised image coordinates, h8 = 1. */
  float A[8][9];
  memset(A, 0, sizeof(A));
  for (int i = 0; i < 4; i++) {
    const float X = obj[i][0], Y = obj[i][1];
    const float x = img[i][0], y = img[i][1];
    float *r0 = A[2 * i], *r1 = A[2 * i + 1];
    r0[0] = X; r0[1] = Y; r0[2] = 1.0f; r0[6] = -x * X; r0[7] = -x * Y; r0[8] = x;
    r1[3] = X; r1[4] = Y; r1[5] = 1.0f; r1[6] = -y * X; r1[7] = -y * Y; r1[8] = y;
  }
  if (!solve8(A)) return false;

  const float h[9] = { A[0][8], A[1][8], A[2][8],
                       A[3][8], A[4][8], A[5][8],
                       A[6][8], A[7][8], 1.0f };

  /* The homography's columns are r0, r1 and t, up to scale. */
  const float n0 = sqrtf(h[0]*h[0] + h[3]*h[3] + h[6]*h[6]);
  const float n1 = sqrtf(h[1]*h[1] + h[4]*h[4] + h[7]*h[7]);
  if (n0 < 1e-9f || n1 < 1e-9f) return false;
  /* h8 is 1 and the scale positive, so tz is positive: the tag is in front. */
  const float scale = 2.0f / (n0 + n1);

  float r0[3] = { h[0]*scale, h[3]*scale, h[6]*scale };
  float r1[3] = { h[1]*scale, h[4]*scale, h[7]*scale };
  const float t[3] = { h[2]*scale, h[5]*scale, h[8]*scale };

  /* Orthonormalise r0 and r1, splitting their disagreement between them. */
  float dot = r0[0]*r1[0] + r0[1]*r1[1] + r0[2]*r1[2];
  for (int i = 0; i < 3; i++) {
    const float a = r0[i] - 0.5f * dot * r1[i];
    const float b = r1[i] - 0.5f * dot * r0[i];
    r0[i] = a; r1[i] = b;
  }
  for (int k = 0; k < 2; k++) {
    float *r = k ? r1 : r0;
    const float n = sqrtf(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
    if (n < 1e-9f) return false;
    r[0] /= n; r[1] /= n; r[2] /= n;
  }
  const float r2[3] = { r0[1]*r1[2] - r0[2]*r1[1],
                        r0[2]*r1[0] - r0[0]*r1[2],
                        r0[0]*r1[1] - r0[1]*r1[0] };

  /* R = [r0 r1 r2] as columns, converted to Euler XYZ. */
  const float R00 = r0[0], R01 = r1[0], R02 = r2[0];
  const float R10 = r0[1], R11 = r1[1], R12 = r2[1];
  const float R20 = r0[2], R21 = r1[2], R22 = r2[2];

  const float sy_ = sqrtf(R00*R00 + R10*R10);
  const float RAD = 57.2957795f;
  if (sy_ > 1e-6f) {
    out->rx = atan2f(R21, R22) * RAD;
    out->ry = atan2f(-R20, sy_) * RAD;
    out->rz = atan2f(R10, R00) * RAD;
  } else {                       /* gimbal lock */
    out->rx = atan2f(-R12, R11) * RAD;
    out->ry = atan2f(-R20, sy_) * RAD;
    out->rz = 0.0f;
  }
  out->tx = t[0]; out->ty = t[1]; out->tz = t[2];
  return true;
}
