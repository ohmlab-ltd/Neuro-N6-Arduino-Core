/**
 ******************************************************************************
 * @file    n6_motion_gate.h
 * @brief   Frame-change gate: a grey thumbnail difference with hysteresis.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6_MOTION_GATE_H
#define N6_MOTION_GATE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint8_t *prev;
  uint8_t *cur;
  int      w, h;
  uint8_t  delta;           /* per-pixel grey change that counts, default 24 */
  uint16_t on_permille;     /* on at or above this score, default 30 */
  uint16_t off_permille;    /* off at or below this score, default 10 */
  bool     have_prev;
  bool     active;
  uint16_t last_score;      /* per-mille of pixels changed at the last update */
} n6_motion_gate_t;

/* mem holds 2*w*h bytes, or NULL takes them from the PSRAM pool after
 * Vision.begin(). Sets the thresholds to their defaults. */
bool n6_motion_gate_begin(n6_motion_gate_t *g, int w, int h, uint8_t *mem);

/* Forgets the previous frame, so the next update returns 0. */
void n6_motion_gate_reset(n6_motion_gate_t *g);

/* Returns the per-mille, 0..1000, of gate pixels whose grey changed by more
 * than delta, and 0 on the first frame. */
uint16_t n6_motion_gate_score(n6_motion_gate_t *g, const uint8_t *rgb888,
                              int src_w, int src_h);

/* score() with hysteresis. Returns the gate state. */
bool n6_motion_gate_update(n6_motion_gate_t *g, const uint8_t *rgb888,
                           int src_w, int src_h);

#ifdef __cplusplus
}
#endif

#endif /* N6_MOTION_GATE_H */
