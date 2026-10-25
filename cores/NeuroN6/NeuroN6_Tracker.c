/**
 ******************************************************************************
 * @file    NeuroN6_Tracker.c
 * @brief   Detect-then-track state machine.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "NeuroN6_Tracker.h"

#include <math.h>
#include <string.h>

#ifndef N6_PI
#define N6_PI  3.14159265358979323846f
#endif

/* Interpolates between two angles the short way round. */
static float n6_ang_lerp(float from, float to, float t)
{
  float d = to - from;
  while (d >   N6_PI) d -= 2.0f * N6_PI;
  while (d <= -N6_PI) d += 2.0f * N6_PI;
  return from + d * t;
}

void n6_tracker_init(n6_tracker_t *t, float smoothing, uint8_t max_misses)
{
  if (!t) return;
  memset(t, 0, sizeof(*t));
  if (smoothing < 0.0f) smoothing = 0.0f;
  if (smoothing > 0.99f) smoothing = 0.99f;   /* 1.0 would freeze the region */
  t->smoothing  = smoothing;
  t->max_misses = (max_misses > 0u) ? max_misses : 1u;
  t->state      = N6_TRACK_SEARCHING;
}

void n6_tracker_reset(n6_tracker_t *t)
{
  if (!t) return;
  t->state  = N6_TRACK_SEARCHING;
  t->misses = 0u;
  t->hits   = 0u;
}

void n6_tracker_update(n6_tracker_t *t, const n6_roi_t *roi)
{
  if (!t || !roi) return;

  /* An empty region is ignored: it crops to nothing. */
  if (!(roi->w > 0.0f) || !(roi->h > 0.0f)) return;

  if (t->state != N6_TRACK_TRACKING || t->hits == 0u) {
    t->roi   = *roi;
    t->state = N6_TRACK_TRACKING;
    t->hits  = 1u;
    t->misses = 0u;
    return;
  }

  const float s = t->smoothing;
  const float k = 1.0f - s;

  t->roi.cx    = t->roi.cx * s + roi->cx * k;
  t->roi.cy    = t->roi.cy * s + roi->cy * k;
  t->roi.w     = t->roi.w  * s + roi->w  * k;
  t->roi.h     = t->roi.h  * s + roi->h  * k;
  t->roi.angle = n6_ang_lerp(t->roi.angle, roi->angle, k);

  if (t->hits < 0xFFFFFFFFu) t->hits++;
  t->misses = 0u;
}

bool n6_tracker_miss(n6_tracker_t *t)
{
  if (!t) return false;
  if (t->state != N6_TRACK_TRACKING) return false;

  if (t->misses < 0xFFu) t->misses++;

  if (t->misses >= t->max_misses) {
    /* The stored ROI is kept. */
    t->state  = N6_TRACK_SEARCHING;
    t->hits   = 0u;
    return false;
  }

  return true;
}
