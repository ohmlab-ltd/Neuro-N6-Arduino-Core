/**
 ******************************************************************************
 * @file    NeuroN6_Tracker.h
 * @brief   Detect-then-track state machine for cascaded models.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef NEURON6_TRACKER_H
#define NEURON6_TRACKER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "NeuroN6_ROI.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  N6_TRACK_SEARCHING = 0,   /* no lock: run the detector */
  N6_TRACK_TRACKING  = 1,   /* locked: run the second stage on the ROI */
} n6_track_state_t;

typedef struct {
  float    smoothing;    /* 0 follows each region, toward 1 damps more */
  uint8_t  max_misses;   /* consecutive misses before the lock drops */

  /* State, read through the accessors. */
  n6_track_state_t state;
  n6_roi_t         roi;        /* smoothed */
  uint8_t          misses;     /* consecutive */
  uint32_t         hits;       /* updates since acquisition */
} n6_tracker_t;

/* smoothing is clamped to 0..0.99, max_misses to at least 1. Starts in
 * SEARCHING. */
void n6_tracker_init(n6_tracker_t *t, float smoothing, uint8_t max_misses);

/* Drops the lock. */
void n6_tracker_reset(n6_tracker_t *t);

static inline bool n6_tracker_is_tracking(const n6_tracker_t *t)
{
  return (t != NULL) && (t->state == N6_TRACK_TRACKING);
}

/* The region to feed Vision.runRoi(). Valid while tracking. */
static inline const n6_roi_t *n6_tracker_roi(const n6_tracker_t *t)
{
  return (t != NULL) ? &t->roi : NULL;
}

/* Smooths roi into the stored region and clears the miss count. The update
 * that acquires the lock is taken unsmoothed. */
void n6_tracker_update(n6_tracker_t *t, const n6_roi_t *roi);

/* n6_tracker_update() from an upright box. Coordinates as for n6_roi_t. */
static inline void n6_tracker_update_box(n6_tracker_t *t,
                                         float cx, float cy, float w, float h)
{
  n6_roi_t r;
  r.cx = cx; r.cy = cy; r.w = w; r.h = h;
  r.angle = 0.0f;
  n6_tracker_update(t, &r);
}

/* Reports a frame with nothing found. Returns true while still locked, false
 * when the lock drops. */
bool n6_tracker_miss(n6_tracker_t *t);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_TRACKER_H */
