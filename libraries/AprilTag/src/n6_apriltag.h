/**
 ******************************************************************************
 * @file    n6_apriltag.h
 * @brief   AprilTag tag36h11 detection on the Cortex-M55. Not reentrant.
 ******************************************************************************
 *
 * The tag36h11 codebook comes from upstream AprilTag. See LICENSE.apriltag.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef N6_APRILTAG_H
#define N6_APRILTAG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Largest image side in pixels. Sets the size of the static buffers. */
#ifndef N6_APRILTAG_MAX_DIM
#define N6_APRILTAG_MAX_DIM 320
#endif

typedef struct {
  int32_t id;          /* 0..586 */
  int32_t hamming;     /* bit errors corrected */
  float   margin;      /* light/dark separation, 0..1 */

  /* Axis-aligned box, normalised 0..1 to the image. */
  float   cx, cy, w, h;

  /* Corners, normalised, clockwise from the tag's own top-left. */
  float   corners[4][2];

  /* Rotation in the image, degrees CCW from upright, -180..180. */
  float   angle;
} n6_apriltag_t;

typedef struct {
  /* Bit errors accepted, 0..2. */
  int32_t max_hamming;

  /* Shortest quad side accepted, in pixels. A tag needs about 6 px per cell
   * and is 10 cells across. */
  int32_t min_side_px;

  /* Local contrast a tile needs to be thresholded, 0..255. */
  int32_t min_contrast;
} n6_apriltag_cfg_t;

#define N6_APRILTAG_CFG_DEFAULT { 2, 40, 20 }

/* Finds tags in an 8-bit grayscale image of stride w, with w and h at most
 * N6_APRILTAG_MAX_DIM. Writes out best margin first and returns the count,
 * negative for invalid arguments. */
int n6_apriltag_find_gray(const uint8_t *gray, int w, int h,
                          const n6_apriltag_cfg_t *cfg,
                          n6_apriltag_t *out, int max_out);

/* The same on RGB888. gray_scratch holds at least w*h bytes. */
int n6_apriltag_find_rgb888(const uint8_t *rgb, int w, int h,
                            uint8_t *gray_scratch,
                            const n6_apriltag_cfg_t *cfg,
                            n6_apriltag_t *out, int max_out);

/* Counts from one detect call. */
typedef struct {
  int32_t blobs;          /* dark blobs found */
  int32_t quads;          /* blobs that fit a quad */
  int32_t decoded;        /* tags decoded */

  int32_t rej_contrast;   /* quad rejected: black and white too close */
  int32_t rej_border;     /* quad rejected: border ring not dark */
  int32_t rej_code;       /* quad rejected: no code within max_hamming */
  int32_t drop_stack;     /* blob dropped: fill frontier hit MAX_STACK */
  int32_t drop_boundary;  /* blob dropped: boundary list hit MAX_BOUNDARY */
  int32_t drop_npx;       /* pixels reached by the largest dropped blob */

  int32_t best_hamming;   /* closest any quad got, 99 if none */
} n6_apriltag_stats_t;

/* Copies the last detect call's thresholded bitmap, one byte per pixel,
 * 255 dark and 0 light. */
void n6_apriltag_debug_dark(uint8_t *out, int w, int h);

/* Copies the last 8x8 cell grid sampled, one byte per cell, 1 = light. */
void n6_apriltag_debug_cells(uint8_t *out64);

/* Stats from the most recent detect call. */
void n6_apriltag_last_stats(n6_apriltag_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* N6_APRILTAG_H */
