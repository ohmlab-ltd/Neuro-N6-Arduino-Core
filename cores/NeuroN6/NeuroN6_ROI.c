/**
 ******************************************************************************
 * @file    NeuroN6_ROI.c
 * @brief   Region of interest geometry for cascaded models.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "NeuroN6_ROI.h"
#include "NeuroN6.h"

#include <math.h>

#ifndef N6_PI
#define N6_PI  3.14159265358979323846f
#endif

/* Camera size in pixels, or 1 before it is set. */
static inline void n6_cam_dims(float *w, float *h)
{
  const float cw = (float)g_camera_full_w;
  const float ch = (float)g_camera_full_h;
  *w = (cw > 0.0f) ? cw : 1.0f;
  *h = (ch > 0.0f) ? ch : 1.0f;
}

n6_roi_t n6_roi_from_cam_box(float cx, float cy, float w, float h, float angle)
{
  n6_roi_t r;
  r.cx = cx; r.cy = cy; r.w = w; r.h = h; r.angle = angle;
  return r;
}

n6_roi_t n6_roi_from_nn_box(float cx, float cy, float w, float h)
{
  const float l = nn_norm_to_cam_norm_x(cx - 0.5f * w);
  const float r = nn_norm_to_cam_norm_x(cx + 0.5f * w);
  const float t = nn_norm_to_cam_norm_y(cy - 0.5f * h);
  const float b = nn_norm_to_cam_norm_y(cy + 0.5f * h);

  n6_roi_t out;
  out.cx    = 0.5f * (l + r);
  out.cy    = 0.5f * (t + b);
  out.w     = (r > l) ? (r - l) : 0.0f;
  out.h     = (b > t) ? (b - t) : 0.0f;
  out.angle = 0.0f;
  return out;
}

n6_roi_t n6_roi_from_nn_box_kps(float cx, float cy, float w, float h,
                                float x0, float y0, float x1, float y1,
                                float target_angle_deg)
{
  n6_roi_t out = n6_roi_from_nn_box(cx, cy, w, h);

  float cam_w, cam_h;
  n6_cam_dims(&cam_w, &cam_h);

  /* Measured in pixels: normalised deltas scale x and y differently. */
  const float px0 = nn_norm_to_cam_norm_x(x0) * cam_w;
  const float py0 = nn_norm_to_cam_norm_y(y0) * cam_h;
  const float px1 = nn_norm_to_cam_norm_x(x1) * cam_w;
  const float py1 = nn_norm_to_cam_norm_y(y1) * cam_h;

  const float dx = px1 - px0;
  const float dy = py1 - py0;

  if (dx != 0.0f || dy != 0.0f) {
    const float target = target_angle_deg * (N6_PI / 180.0f);
    /* y grows downward, so dy is negated. The result is clockwise-positive. */
    out.angle = target - atan2f(-dy, dx);

    while (out.angle >   N6_PI) out.angle -= 2.0f * N6_PI;
    while (out.angle <= -N6_PI) out.angle += 2.0f * N6_PI;
  }
  return out;
}

n6_roi_t n6_roi_expand(n6_roi_t roi, float scale_x, float scale_y,
                       float shift_x, float shift_y, bool square_long)
{
  float cam_w, cam_h;
  n6_cam_dims(&cam_w, &cam_h);

  float cxp = roi.cx * cam_w;
  float cyp = roi.cy * cam_h;
  float wp  = roi.w  * cam_w;
  float hp  = roi.h  * cam_h;

  /* Order matters: shift by the original w and h, then square, then scale. */
  if (shift_x != 0.0f || shift_y != 0.0f) {
    const float ca = cosf(roi.angle);
    const float sa = sinf(roi.angle);
    const float dx = shift_x * wp;
    const float dy = shift_y * hp;
    cxp += dx * ca - dy * sa;
    cyp += dx * sa + dy * ca;
  }

  /* Squared in pixels, so the region is square on the sensor. */
  if (square_long) {
    const float side = (wp > hp) ? wp : hp;
    wp = side;
    hp = side;
  }

  wp *= scale_x;
  hp *= scale_y;

  roi.cx = cxp / cam_w;
  roi.cy = cyp / cam_h;
  roi.w  = wp  / cam_w;
  roi.h  = hp  / cam_h;
  return roi;
}

void n6_roi_aabb(const n6_roi_t *roi, float *out_w, float *out_h)
{
  if (!roi) return;

  float cam_w, cam_h;
  n6_cam_dims(&cam_w, &cam_h);

  const float wp = roi->w * cam_w;
  const float hp = roi->h * cam_h;
  const float ca = fabsf(cosf(roi->angle));
  const float sa = fabsf(sinf(roi->angle));

  if (out_w) *out_w = (wp * ca + hp * sa) / cam_w;
  if (out_h) *out_h = (wp * sa + hp * ca) / cam_h;
}

/* Applies a spec to a box in NN-normalised coordinates. */
static n6_roi_t n6_roi_apply_spec(float cx, float cy, float w, float h,
                                  const float *kps, int kp_stride, int kp_count,
                                  const n6_roi_spec_t *spec)
{
  if (spec == NULL) {
    return n6_roi_from_nn_box(cx, cy, w, h);
  }

  const bool want_rot = (spec->rot_kp_start >= 0) && (spec->rot_kp_end >= 0);
  const bool have_kps = (kps != NULL) && (kp_stride >= 2) &&
                        (spec->rot_kp_start < kp_count) &&
                        (spec->rot_kp_end   < kp_count);

  n6_roi_t roi;
  if (want_rot && have_kps) {
    const float *a = &kps[(int)spec->rot_kp_start * kp_stride];

    float ex = 0.0f, ey = 0.0f;
    int   n  = 0;
    const int8_t ends[3] = { spec->rot_kp_end, spec->rot_kp_end2, spec->rot_kp_end3 };
    for (int i = 0; i < 3; i++) {
      if (ends[i] < 0 || ends[i] >= kp_count) continue;
      const float *p = &kps[(int)ends[i] * kp_stride];
      ex += p[0];
      ey += p[1];
      n++;
    }

    if (n > 0) {
      ex /= (float)n;
      ey /= (float)n;
      roi = n6_roi_from_nn_box_kps(cx, cy, w, h, a[0], a[1], ex, ey,
                                   spec->target_angle_deg);
    } else {
      roi = n6_roi_from_nn_box(cx, cy, w, h);
    }
  } else {
    roi = n6_roi_from_nn_box(cx, cy, w, h);
  }

  return n6_roi_expand(roi, spec->scale_x, spec->scale_y,
                       spec->shift_x, spec->shift_y, spec->square_long);
}

n6_roi_t n6_roi_build(float cx, float cy, float w, float h,
                      const float *kps, int kp_stride, int kp_count,
                      const n6_roi_spec_t *spec)
{
  return n6_roi_apply_spec(cx, cy, w, h, kps, kp_stride, kp_count, spec);
}

n6_roi_t n6_roi_from_nn_keypoints(const float *kps, int kp_stride, int kp_count,
                                  const n6_roi_spec_t *spec)
{
  const n6_roi_t empty = { 0.5f, 0.5f, 0.0f, 0.0f, 0.0f };
  if (kps == NULL || kp_stride < 2 || kp_count < 1) return empty;

  float minx = kps[0], maxx = kps[0];
  float miny = kps[1], maxy = kps[1];
  for (int k = 1; k < kp_count; k++) {
    const float x = kps[k * kp_stride + 0];
    const float y = kps[k * kp_stride + 1];
    if (x < minx) minx = x;
    if (x > maxx) maxx = x;
    if (y < miny) miny = y;
    if (y > maxy) maxy = y;
  }

  const float cx = 0.5f * (minx + maxx);
  const float cy = 0.5f * (miny + maxy);
  const float w  = maxx - minx;
  const float h  = maxy - miny;
  if (!(w > 0.0f) || !(h > 0.0f)) return empty;

  return n6_roi_apply_spec(cx, cy, w, h, kps, kp_stride, kp_count, spec);
}

void n6_roi_unmap_point(const n6_roi_t *roi, float rx, float ry,
                        float *out_cam_x, float *out_cam_y)
{
  if (!roi) return;

  float cam_w, cam_h;
  n6_cam_dims(&cam_w, &cam_h);

  const float lx = (rx - 0.5f) * (roi->w * cam_w);
  const float ly = (ry - 0.5f) * (roi->h * cam_h);

  const float ca = cosf(roi->angle);
  const float sa = sinf(roi->angle);

  const float px = lx * ca - ly * sa;
  const float py = lx * sa + ly * ca;

  if (out_cam_x) *out_cam_x = roi->cx + px / cam_w;
  if (out_cam_y) *out_cam_y = roi->cy + py / cam_h;
}
