/* Host test of the segmentation metadata builder,
 * libraries/PostProcess/src/vnd_meta_yolov8_Iseg.c: a box past the frame, or
 * NaN, reads no byte outside its 64 x 64 mask, which ends at a page the
 * process cannot read. The host stretches each mask over its box, so a box
 * part way out of the frame or the network's view is sent as the part the
 * mask covers, to within a mask cell. Under a stretch, a centre crop and a
 * letterbox. POSIX only. From this folder:
 *
 *   gcc -std=gnu11 -include ../../../extras/postprocess-tests/shim/host_prefix.h \
 *       -I../../../extras/postprocess-tests/shim -I../../../libraries/PostProcess/src \
 *       -I../../../cores/NeuroN6/lib_vision_models_pp/lib_vision_models_pp/Inc \
 *       -I../../../cores/NeuroN6 test_iseg_mask.c \
 *       ../../../libraries/PostProcess/src/vnd_meta_yolov8_Iseg.c \
 *       ../../../libraries/PostProcess/src/vnd_meta.c -lm -o /tmp/test_iseg_mask
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#define _DEFAULT_SOURCE
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>
#include "vnd_meta_yolov8_Iseg.h"
#include "app_postprocess_iseg_yolo_v8_ui.h"

int16_t  g_lcd_camera_x, g_lcd_camera_y, g_nn_crop_x, g_nn_crop_y;
uint16_t g_lcd_camera_w = 320, g_lcd_camera_h = 240, g_nn_crop_w = 640, g_nn_crop_h = 480;
uint16_t g_camera_full_w = 640, g_camera_full_h = 480;
uint32_t g_n6_boot_seed;
uint32_t ts[2];
const char *classes_table[NB_CLASSES] = { "person" };
iseg_pp_out_t pp_output;
iseg_yolov8_pp_static_param_t pp_params;
uint16_t *OVL_GetBackBuffer(void) { return NULL; }
void OVL_Commit(void) {}
uint32_t HAL_GetTick(void) { return 0; }

#define M AI_YOLOV8_SEG_PP_MASK_SIZE

/* The camera pixels the network sees, of a 640 x 480 frame. */
static const struct { const char *name; int16_t x, y; uint16_t w, h; } views[] = {
  { "stretch",      0,   0, 640, 480 },
  { "centre crop", 80,   0, 480, 480 },
  { "letterbox",    0, -80, 640, 640 },
};

/* NN-normalised centre x, y, width, height. */
static const float boxes[][4] = {
  { 0.50f, 0.50f, 0.20f, 0.20f },
  { 0.95f, 0.50f, 0.30f, 0.20f },
  { 0.50f, 0.05f, 0.20f, 0.30f },
  { 0.02f, 0.97f, 0.20f, 0.20f },
};

/* One detection through the builder: the box sent, frame-normalised, and the
 * mask's width and height in cells. */
static bool sent(const float in[4], float box[4], int cells[2])
{
  static uint8_t mask[M * M];
  iseg_pp_outBuffer_t one;
  memset(&one, 0, sizeof(one));
  one.x_center = in[0];
  one.y_center = in[1];
  one.width = in[2];
  one.height = in[3];
  one.conf = 0.9f;
  one.pMask = mask;
  memset(mask, 1, sizeof(mask));
  iseg_pp_out_t lone = { &one, 1 };
  vnd_meta_build_iseg_on_slot(&lone, 7u, NULL, VND_META_SLOT_MODEL_0);

  const uint8_t *p;
  uint32_t len;
  bool got_box = false, got_mask = false;
  if (!vnd_meta_get_slot(VND_META_SLOT_MODEL_0, &p, &len)) return false;
  for (uint32_t i = 0; i + 3u <= len;) {
    const uint32_t n = (uint32_t)p[i + 1] | ((uint32_t)p[i + 2] << 8);
    const uint8_t *v = &p[i + 3];
    if (i + 3u + n > len) return false;
    if (p[i] == META_ISEG_MASK && n >= 3u) {
      cells[0] = v[1];
      cells[1] = v[2];
      got_mask = true;
    }
    if (p[i] == META_BOXES && n >= 11u && v[0] == 1u) {
      for (int k = 0; k < 4; k++) {
        box[k] = (float)((uint32_t)v[1 + 2 * k] | ((uint32_t)v[2 + 2 * k] << 8)) / 65535.f;
      }
      got_box = true;
    }
    i += 3u + n;
  }
  return got_box && got_mask;
}

int main(void)
{
  static const float centres[] = { 0.5f, 0.95f, 1.10f, 1.50f, 2.04f, -0.60f, NAN, INFINITY, -INFINITY };
  const size_t n = sizeof(centres) / sizeof(centres[0]);
  const size_t page = (size_t)sysconf(_SC_PAGESIZE);
  const size_t mask_bytes = M * M;
  uint8_t *pg = mmap(NULL, 2 * page, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  int checks = 0, fails = 0;

  if (pg == MAP_FAILED || mask_bytes > page || mprotect(pg + page, page, PROT_NONE) != 0) {
    printf("FAIL could not set up a guard page\n");
    return 1;
  }
  for (size_t g = 0; g < sizeof(views) / sizeof(views[0]); g++) {
    g_nn_crop_x = views[g].x;
    g_nn_crop_y = views[g].y;
    g_nn_crop_w = views[g].w;
    g_nn_crop_h = views[g].h;

    for (size_t i = 0; i < n; i++) {
      iseg_pp_outBuffer_t one;
      memset(&one, 0, sizeof(one));
      one.x_center = centres[i];
      one.y_center = centres[i];
      one.width = 0.1f;
      one.height = 0.1f;
      one.conf = 0.9f;
      one.pMask = pg + page - mask_bytes;
      memset(one.pMask, 1, mask_bytes);
      iseg_pp_out_t lone = { &one, 1 };
      const pid_t pid = fork();
      if (pid == 0) {
        vnd_meta_build_iseg_on_slot(&lone, 7u, NULL, VND_META_SLOT_MODEL_0);
        _exit(0);
      }
      int st = 0;
      waitpid(pid, &st, 0);
      checks++;
      if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) {
        printf("FAIL %s: a box centred at %.2f read past its mask\n", views[g].name,
               (double)centres[i]);
        fails++;
      }
    }

    /* One mask cell, frame-normalised. */
    const float cell_w = (float)g_nn_crop_w / ((float)M * (float)g_camera_full_w);
    const float cell_h = (float)g_nn_crop_h / ((float)M * (float)g_camera_full_h);
    for (size_t b = 0; b < sizeof(boxes) / sizeof(boxes[0]); b++) {
      float box[4];
      int cells[2];
      checks++;
      if (!sent(boxes[b], box, cells)) {
        printf("FAIL %s: box %d sent no box or no mask\n", views[g].name, (int)b);
        fails++;
        continue;
      }
      const float mask_w = (float)cells[0] * cell_w;
      const float mask_h = (float)cells[1] * cell_h;
      if (fabsf(box[2] - mask_w) > cell_w + 1e-3f || fabsf(box[3] - mask_h) > cell_h + 1e-3f) {
        printf("FAIL %s: box %d is %.3f x %.3f, its mask %.3f x %.3f\n", views[g].name, (int)b,
               (double)box[2], (double)box[3], (double)mask_w, (double)mask_h);
        fails++;
      }
    }
  }
  printf("%d checks, %d failed\n", checks, fails);
  return fails != 0;
}
