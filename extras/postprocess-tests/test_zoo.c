/* Host test driver for the model-zoo post-processors.
 *
 *   test_zoo <manifest>
 *
 * The manifest names the output tensors and the post-processor:
 *
 *   input  <w> <h>
 *   camera <full_w> <full_h> <crop_x> <crop_y> <crop_w> <crop_h>   (optional)
 *   tensor <name> <float|int8|uint8> <scale> <zp> <chpos:0|1|2> <ndims> <d...> <file>
 *   pp     <yolov8|yolox|pixelkit|yolov2|ssd|blazeface|movenet|deeplab|classifier> <conf> <iou> [top_k]
 *   slot   <n>   (optional; yolov8 and deeplab publish to VND_META_SLOT_MODEL_n)
 *
 * For pixelkit, top_k is the number of COCO names to pass. A second argument
 * is a file to write the overlay to, 800x480 ARGB4444.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "PostProcessor_Zoo.h"
#include "zoo_pp.h"
#include "vnd_meta.h"

/* the network's buffers */

#define MAXT 8
static LL_Buffer_InfoTypeDef g_out[MAXT + 1];
static LL_Buffer_InfoTypeDef g_in[2];
static uint32_t g_out_shape[MAXT][6];
static uint32_t g_in_shape[4];
static float    g_scale[MAXT];
static int16_t  g_zp[MAXT];
static char     g_names[MAXT][64];
static int      g_nout;

const LL_Buffer_InfoTypeDef *LL_ATON_Output_Buffers_Info(const NN_Instance_TypeDef *n)
{
  (void)n;
  return g_out;
}

const LL_Buffer_InfoTypeDef *LL_ATON_Input_Buffers_Info(const NN_Instance_TypeDef *n)
{
  (void)n;
  return (g_in[0].name != NULL) ? g_in : NULL;
}

/* the overlay, with a fence on each side */

#define OVL_W 800
#define OVL_H 480
#define FENCE 4096
static uint16_t g_ovl[FENCE + OVL_W * OVL_H + FENCE];
static int g_commits;

uint16_t *OVL_GetBackBuffer(void) { return &g_ovl[FENCE]; }
void OVL_Commit(void) { g_commits++; }

int16_t  g_lcd_camera_x = 80;
int16_t  g_lcd_camera_y = 0;
uint16_t g_lcd_camera_w = 640;
uint16_t g_lcd_camera_h = 480;
int16_t  g_nn_crop_x = 0;
int16_t  g_nn_crop_y = 0;
uint16_t g_nn_crop_w = 640;
uint16_t g_nn_crop_h = 480;
uint16_t g_camera_full_w = 640;
uint16_t g_camera_full_h = 480;

/* Inference timestamps the publishers read. */
uint32_t ts[2] = { 100u, 112u };

/* vnd_meta.c stamps its Terminal log with this. */
uint32_t HAL_GetTick(void) { return 1000u; }

/* The core's boot seed, the last byte of the Terminal log record. */
uint32_t g_n6_boot_seed = 1u;

/* COCO-80 names, as classes_table_default.c provides on the board. */
const char *classes_table[80] = {
  "person", "bicycle", "car", "motorcycle",
  "airplane", "bus", "train", "truck",
  "boat", "traffic light", "fire hydrant", "stop sign",
  "parking meter", "bench", "bird", "cat",
  "dog", "horse", "sheep", "cow",
  "elephant", "bear", "zebra", "giraffe",
  "backpack", "umbrella", "handbag", "tie",
  "suitcase", "frisbee", "skis", "snowboard",
  "sports ball", "kite", "baseball bat", "baseball glove",
  "skateboard", "surfboard", "tennis racket", "bottle",
  "wine glass", "cup", "fork", "knife",
  "spoon", "bowl", "banana", "apple",
  "sandwich", "orange", "broccoli", "carrot",
  "hot dog", "pizza", "donut", "cake",
  "chair", "couch", "potted plant", "bed",
  "dining table", "toilet", "tv", "laptop",
  "mouse", "remote", "keyboard", "cell phone",
  "microwave", "oven", "toaster", "sink",
  "refrigerator", "book", "clock", "vase",
  "scissors", "teddy bear", "hair drier", "toothbrush"
};

/* manifest */

static unsigned char *load(const char *path, size_t *len)
{
  FILE *f = fopen(path, "rb");
  if (f == NULL) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  unsigned char *p = (unsigned char *)malloc((size_t)n + 16u);
  if (fread(p, 1u, (size_t)n, f) != (size_t)n) { fprintf(stderr, "short read %s\n", path); exit(2); }
  fclose(f);
  *len = (size_t)n;
  return p;
}

int main(int argc, char **argv)
{
  if (argc < 2) { fprintf(stderr, "usage: test_zoo <manifest>\n"); return 2; }
  FILE *mf = fopen(argv[1], "r");
  if (mf == NULL) { fprintf(stderr, "cannot open %s\n", argv[1]); return 2; }

  char pp_name[32] = "";
  float conf = 0.5f, iou = 0.5f;
  int top_k = 5;
  int slot = 0;
  char line[1024];
  while (fgets(line, sizeof(line), mf) != NULL) {
    char kind[32];
    if (sscanf(line, "%31s", kind) != 1) continue;
    if (strcmp(kind, "input") == 0) {
      unsigned w, h;
      sscanf(line, "%*s %u %u", &w, &h);
      g_in_shape[0] = 1u; g_in_shape[1] = h; g_in_shape[2] = w; g_in_shape[3] = 3u;
      g_in[0].name = "input";
      g_in[0].mem_shape = g_in_shape;
      g_in[0].mem_ndims = 4u;
      g_in[0].type = DataType_UINT8;
      g_in[1].name = NULL;
    } else if (strcmp(kind, "camera") == 0) {
      unsigned fw, fh, cw, ch;
      int cx, cy;
      sscanf(line, "%*s %u %u %d %d %u %u", &fw, &fh, &cx, &cy, &cw, &ch);
      g_camera_full_w = (uint16_t)fw; g_camera_full_h = (uint16_t)fh;
      g_nn_crop_x = (int16_t)cx; g_nn_crop_y = (int16_t)cy;
      g_nn_crop_w = (uint16_t)cw; g_nn_crop_h = (uint16_t)ch;
    } else if (strcmp(kind, "tensor") == 0) {
      char name[64], type[16], file[512];
      float sc; int zp, chpos, nd;
      int pos = 0;
      if (sscanf(line, "%*s %63s %15s %f %d %d %d%n", name, type, &sc, &zp, &chpos, &nd, &pos) != 6) {
        fprintf(stderr, "bad tensor line: %s", line); return 2;
      }
      LL_Buffer_InfoTypeDef *b = &g_out[g_nout];
      memset(b, 0, sizeof(*b));
      uint32_t count = 1u;
      for (int d = 0; d < nd; d++) {
        unsigned v; int adv = 0;
        sscanf(line + pos, " %u%n", &v, &adv);
        pos += adv;
        g_out_shape[g_nout][d] = v;
        count *= v;
      }
      sscanf(line + pos, " %511s", file);
      size_t bytes = 0;
      unsigned char *data = load(file, &bytes);
      snprintf(g_names[g_nout], sizeof(g_names[g_nout]), "%s", name);
      g_scale[g_nout] = sc;
      g_zp[g_nout] = (int16_t)zp;
      b->name = g_names[g_nout];
      b->addr_base = data;
      b->offset_start = 0u;
      b->offset_end = (uint32_t)bytes;
      b->mem_shape = g_out_shape[g_nout];
      b->mem_ndims = (uint16_t)nd;
      b->chpos = (Buffer_CHPos_TypeDef)chpos;
      b->type = (strcmp(type, "float") == 0) ? DataType_FLOAT
              : ((strcmp(type, "int8") == 0) ? DataType_INT8 : DataType_UINT8);
      if (b->type != DataType_FLOAT) {
        b->scale = &g_scale[g_nout];
        b->offset = &g_zp[g_nout];
      }
      const uint32_t esz = (b->type == DataType_FLOAT) ? 4u : 1u;
      if (bytes != (size_t)count * esz) {
        fprintf(stderr, "%s: %u bytes on disk, %u from the shape\n", file,
                (unsigned)bytes, (unsigned)(count * esz));
        return 2;
      }
      g_nout++;
      g_out[g_nout].name = NULL;
    } else if (strcmp(kind, "pp") == 0) {
      sscanf(line, "%*s %31s %f %f %d", pp_name, &conf, &iou, &top_k);
    } else if (strcmp(kind, "slot") == 0) {
      sscanf(line, "%*s %d", &slot);
    }
  }
  fclose(mf);

  IPostProcessor *pp = NULL;
  const char *const *labels = NULL;
  int nlabels = 0;
  if (strcmp(pp_name, "yolov8") == 0)
    pp = PostProcess_YOLOv8Detect_Ex(conf, iou, NULL, 0, (vnd_meta_slot_t)slot);
  else if (strcmp(pp_name, "yolox") == 0)      pp = PostProcess_STYoloX(conf, iou);
  else if (strcmp(pp_name, "pixelkit") == 0) {
    labels = (top_k > 0) ? (const char *const *)classes_table : NULL;
    nlabels = (top_k > 0) ? top_k : 0;
    pp = PostProcess_PixelKitYOLOX(conf, iou, labels, nlabels);
  }
  else if (strcmp(pp_name, "yolov2") == 0)     pp = PostProcess_TinyYOLOv2(conf, iou);
  else if (strcmp(pp_name, "ssd") == 0)        pp = PostProcess_SSD(conf, iou);
  else if (strcmp(pp_name, "blazeface") == 0)  pp = PostProcess_BlazeFace(conf, iou);
  else if (strcmp(pp_name, "movenet") == 0)    pp = PostProcess_MoveNet(conf);
  else if (strcmp(pp_name, "deeplab") == 0)
    pp = PostProcess_DeepLabV3_Ex(NULL, 0, (vnd_meta_slot_t)slot);
  else if (strcmp(pp_name, "classifier") == 0) {
    labels = (const char *const *)classes_table; nlabels = 80;   /* names for the first 80 classes */
    pp = PostProcess_Classifier_Ex(conf, labels, nlabels, top_k, VND_META_SLOT_MODEL_0);
  } else { fprintf(stderr, "unknown post-processor '%s'\n", pp_name); return 2; }

  static uint8_t meta_buf[1024u * 1024u];
  vnd_meta_init(meta_buf, sizeof(meta_buf));

  for (size_t i = 0; i < sizeof(g_ovl) / sizeof(g_ovl[0]); i++) g_ovl[i] = 0xDEADu;

  NN_Instance_TypeDef *model = (NN_Instance_TypeDef *)(void *)&g_out;   /* only a handle */
  if (pp->init != NULL) (void)pp->init(pp->ctx, model);
  const int32_t st = pp->run(pp->ctx, model);
  printf("status %d\n", (int)st);

  if (pp->draw_lcd != NULL) pp->draw_lcd(pp->ctx, OVL_RELOAD_CLEAR);
  int fence_ok = 1;
  uint32_t drawn = 0;
  for (int i = 0; i < FENCE; i++) {
    if (g_ovl[i] != 0xDEADu || g_ovl[FENCE + OVL_W * OVL_H + i] != 0xDEADu) fence_ok = 0;
  }
  for (int i = 0; i < OVL_W * OVL_H; i++) if (g_ovl[FENCE + i] != 0u) drawn++;
  printf("lcd %s drawn %u commits %d\n", fence_ok ? "ok" : "OVERFLOW", drawn, g_commits);

  const od_pp_outBuffer_t *boxes = NULL;
  const int n = PostProcess_ZooResults(pp, &boxes);
  printf("nb %d\n", n);
  for (int i = 0; i < n; i++) {
    printf("det %.6f %.6f %.6f %.6f %.6f %d\n", boxes[i].x_center, boxes[i].y_center,
           boxes[i].width, boxes[i].height, boxes[i].conf, (int)boxes[i].class_index);
  }
  if (strcmp(pp_name, "classifier") == 0) {
    float s = 0.f;
    const int top = PostProcess_ClassifierTop(pp, &s);
    printf("top %d %.6f\n", top, s);
  }

  if (argc > 2) {
    FILE *f = fopen(argv[2], "wb");
    if (f != NULL) {
      fwrite(&g_ovl[FENCE], sizeof(uint16_t), (size_t)(OVL_W * OVL_H), f);
      fclose(f);
    }
  }

  if (pp->emit_metadata != NULL) pp->emit_metadata(pp->ctx);
  const uint8_t *meta = NULL;
  uint32_t mlen = 0;
  if (vnd_meta_get(&meta, &mlen)) {
    printf("meta %u ", mlen);
    for (uint32_t i = 0; i < mlen; i++) printf("%02x", meta[i]);
    printf("\n");
  } else {
    printf("meta 0\n");
  }
  return 0;
}
