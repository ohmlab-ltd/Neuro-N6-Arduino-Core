/**
 ******************************************************************************
 * @file    n6_imgproc.h
 * @brief   CPU image-processing primitives.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6_IMGPROC_H
#define N6_IMGPROC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Colour blob finder */

typedef struct {
  int32_t target_hue;      /* degrees 0..359: 0 red, 60 yellow, 120 green, 240 blue */
  int32_t hue_tol;         /* degrees either side of target_hue */
  int32_t sat_min;         /* chroma floor, 0..255 */
  int32_t val_min;         /* brightness floor, 0..255 */
  bool    gate_hue;        /* false ignores hue: sat_min and val_min only */

  int32_t min_cell_hits;   /* gated pixels a grid cell needs to seed or join a cluster */
  int32_t min_blob_px;     /* gated pixels a cluster needs to count */
} n6_blob_cfg_t;

/* A found blob. The box is normalised 0..1 to the source image. */
typedef struct {
  float    cx, cy, w, h;
  int32_t  hue;            /* at the centroid, degrees */
  uint32_t pixels;         /* gated pixels in the cluster */
  float    area;           /* pixels over the source's w*h, 0..1 */
} n6_blob_t;

/* Finds the most prominent colour blob, with its box snapped to the cells of a
 * coarse grid. Not reentrant.
 * @param  rgb  packed RGB888, w*h*3 bytes
 * @param  out  written only on true */
bool n6_blob_find_rgb888(const uint8_t *rgb, int w, int h,
                         const n6_blob_cfg_t *cfg, n6_blob_t *out);

/* Hue in degrees, 0..359, at one RGB888 pixel, or 0 when achromatic. */
int32_t n6_hue_at_rgb888(const uint8_t *rgb, int w, int x, int y);

/* Grayscale primitives */

/* Region of interest in pixels, clipped to the image. NULL means the whole
 * image. */
typedef struct {
  int32_t x, y, w, h;
} n6img_rect_t;

/* BT.601 luma. dst needs w*h bytes and must not overlap src. */
void n6img_rgb888_to_gray(const uint8_t *src, uint8_t *dst, int w, int h);

typedef struct {
  uint8_t  min, max;
  uint8_t  mean;
  uint8_t  median;
  uint32_t stdev;     /* x256 */
  uint32_t pixels;
} n6img_stats_t;

void n6img_statistics(const uint8_t *gray, int w, int h,
                      const n6img_rect_t *roi, n6img_stats_t *out);

/* bins is zeroed first. */
void n6img_histogram(const uint8_t *gray, int w, int h,
                     const n6img_rect_t *roi, uint32_t bins[256]);

uint8_t n6img_otsu(const uint32_t bins[256]);

/* dst[i] is 255 where lo <= gray[i] <= hi, else 0. dst may equal gray. */
void n6img_threshold(const uint8_t *gray, uint8_t *dst, int w, int h,
                     uint8_t lo, uint8_t hi);

/* Morphology on a 0/255 image. scratch needs w*h bytes. */
void n6img_erode(uint8_t *bin, uint8_t *scratch, int w, int h, int iterations);
void n6img_dilate(uint8_t *bin, uint8_t *scratch, int w, int h, int iterations);

/* dst[i] = |a[i] - b[i]|. */
void n6img_absdiff(const uint8_t *a, const uint8_t *b, uint8_t *dst,
                   int w, int h);

typedef struct {
  float    cx, cy, w, h;   /* normalised 0..1 */
  uint32_t pixels;
} n6img_component_t;

/* Finds the 8-connected components of a 0/255 image with at least min_px
 * pixels, largest first. labels needs w*h uint16_t of scratch. Returns the
 * count, -1 on an invalid argument, or -2 when more than 2047 components are
 * open at once. */
int n6img_find_components(const uint8_t *bin, uint16_t *labels, int w, int h,
                          uint32_t min_px,
                          n6img_component_t *out, int max_out);

#ifdef __cplusplus
}
#endif

#endif /* N6_IMGPROC_H */
