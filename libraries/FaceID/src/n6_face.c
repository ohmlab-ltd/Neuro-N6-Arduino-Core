/**
 ******************************************************************************
 * @file    n6_face.c
 * @brief   CenterFace decode, ArcFace-template alignment, embedding readout.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "n6_face.h"

#include <math.h>
#include <string.h>
#include <stddef.h>

#include "NeuroN6.h"
#include "ll_aton_runtime.h"

/* Detector state */

static float      s_min_score = 0.50f;
static n6_face_t  s_faces[N6_FACE_MAX];
static int        s_nfaces = 0;
static float      s_last_peak = 0.0f;

void n6_face_detector_set_threshold(float min_score) { s_min_score = min_score; }
int  n6_face_count(void) { return s_nfaces; }
float n6_face_last_peak(void) { return s_last_peak; }

const n6_face_t *n6_face_get(int i)
{
  if (i < 0 || i >= s_nfaces) return NULL;
  return &s_faces[i];
}

/* Detector outputs. Scale and offset both have 2 channels and are told apart
 * by map mean: scale averages positive, offset near zero. */
typedef struct {
  const float *heat;      /* grid*grid, heatmap */
  const float *scale;     /* grid*grid*2, log size */
  const float *offset;    /* grid*grid*2, sub-cell offset */
  const float *lms;       /* grid*grid*10, five (y,x) pairs as box fractions */
  int          grid;
} face_heads_t;

static float map_mean(const float *p, int n)
{
  float acc = 0.0f;
  for (int i = 0; i < n; i++) acc += p[i];
  return acc / (float)n;
}

static bool sort_heads(NN_Instance_TypeDef *instance, face_heads_t *h)
{
  const LL_Buffer_InfoTypeDef *out = LL_ATON_Output_Buffers_Info(instance);
  if (out == NULL) return false;

  const float *two[2] = { NULL, NULL };
  uint32_t     two_n[2] = { 0u, 0u };
  int          ntwo = 0;

  memset(h, 0, sizeof(*h));

  const float *bufs[8]; uint32_t lens[8]; int n = 0;
  for (int i = 0; out[i].name != NULL && n < 8; i++) {
    const uint8_t *start = LL_Buffer_addr_start(&out[i]);
    const uint8_t *end   = LL_Buffer_addr_end(&out[i]);
    if (start == NULL || end == NULL || end <= start) continue;
    bufs[n] = (const float *)(const void *)start;
    lens[n] = (uint32_t)(end - start) / 4u;
    n++;
  }
  if (n != 4) return false;

  /* The smallest buffer is the 1-channel heatmap, which gives the cell count. */
  uint32_t smallest = lens[0];
  for (int i = 1; i < n; i++) if (lens[i] < smallest) smallest = lens[i];
  const uint32_t cells = smallest;
  if (cells == 0u) return false;

  /* The grid must be square. */
  int grid = 0;
  while ((uint32_t)(grid + 1) * (uint32_t)(grid + 1) <= cells) grid++;
  if ((uint32_t)grid * (uint32_t)grid != cells) return false;
  h->grid = grid;

  ntwo = 0;
  for (int i = 0; i < n; i++) {
    const uint32_t ch = lens[i] / cells;
    if (lens[i] != ch * cells) return false;
    if      (ch == 1u)  h->heat = bufs[i];
    else if (ch == 10u) h->lms  = bufs[i];
    else if (ch == 2u && ntwo < 2) { two[ntwo] = bufs[i]; two_n[ntwo] = lens[i]; ntwo++; }
    else return false;
  }
  if (h->heat == NULL || h->lms == NULL || ntwo != 2) return false;

  const float m0 = map_mean(two[0], (int)two_n[0]);
  const float m1 = map_mean(two[1], (int)two_n[1]);
  if (m0 > m1) { h->scale = two[0]; h->offset = two[1]; }
  else         { h->scale = two[1]; h->offset = two[0]; }

  return true;
}

/* Detector post-processor */

static int32_t face_det_init(void *ctx, NN_Instance_TypeDef *model)
{
  (void)ctx; (void)model;
  s_nfaces = 0;
  return 0;
}

/* True when the cell is the strongest in its 3x3 neighbourhood. */
static bool is_peak(const float *heat, int grid, int r, int c, float v)
{
  for (int dr = -1; dr <= 1; dr++) {
    const int rr = r + dr;
    if (rr < 0 || rr >= grid) continue;
    for (int dc = -1; dc <= 1; dc++) {
      const int cc = c + dc;
      if (cc < 0 || cc >= grid) continue;
      if (dr == 0 && dc == 0) continue;
      if (heat[rr * grid + cc] > v) return false;
    }
  }
  return true;
}

static int32_t face_det_run(void *ctx, NN_Instance_TypeDef *model)
{
  (void)ctx;
  s_nfaces = 0;

  face_heads_t h;
  if (!sort_heads(model, &h)) { s_last_peak = 0.0f; return -1; }

  const int   grid = h.grid;
  const float gf   = (float)grid;

  s_last_peak = 0.0f;
  for (int k = 0; k < grid * grid; k++)
    if (h.heat[k] > s_last_peak) s_last_peak = h.heat[k];

  for (int r = 0; r < grid && s_nfaces < N6_FACE_MAX; r++) {
    for (int c = 0; c < grid && s_nfaces < N6_FACE_MAX; c++) {
      const int   idx = r * grid + c;
      const float v   = h.heat[idx];
      if (v < s_min_score) continue;
      if (!is_peak(h.heat, grid, r, c, v)) continue;

      const float sy = expf(h.scale[idx * 2 + 0]);      /* height, cells */
      const float sx = expf(h.scale[idx * 2 + 1]);      /* width, cells */
      const float oy = h.offset[idx * 2 + 0];
      const float ox = h.offset[idx * 2 + 1];

      const float cx_nn = ((float)c + ox + 0.5f) / gf;
      const float cy_nn = ((float)r + oy + 0.5f) / gf;
      const float w_nn  = sx / gf;
      const float h_nn  = sy / gf;

      if (w_nn <= 0.0f || h_nn <= 0.0f || w_nn > 2.0f || h_nn > 2.0f) continue;

      n6_face_t *f = &s_faces[s_nfaces];
      f->score = v;

      /* To camera-normalised while this model's crop is live. The first
       * Vision.runRoi() replaces it. */
      f->cx = nn_norm_to_cam_norm_x(cx_nn);
      f->cy = nn_norm_to_cam_norm_y(cy_nn);
      f->w  = nn_norm_to_cam_norm_x(cx_nn + w_nn * 0.5f) -
              nn_norm_to_cam_norm_x(cx_nn - w_nn * 0.5f);
      f->h  = nn_norm_to_cam_norm_y(cy_nn + h_nn * 0.5f) -
              nn_norm_to_cam_norm_y(cy_nn - h_nn * 0.5f);

      /* Landmarks are (y,x) fractions of the box, not of the frame. */
      const float x0_nn = cx_nn - w_nn * 0.5f;
      const float y0_nn = cy_nn - h_nn * 0.5f;
      for (int j = 0; j < N6_FACE_NUM_LM; j++) {
        const float ly = h.lms[idx * 10 + j * 2 + 0];
        const float lx = h.lms[idx * 10 + j * 2 + 1];
        f->lm_x[j] = nn_norm_to_cam_norm_x(x0_nn + lx * w_nn);
        f->lm_y[j] = nn_norm_to_cam_norm_y(y0_nn + ly * h_nn);
      }

      s_nfaces++;
    }
  }

  return 0;
}

static IPostProcessor s_det_pp = {
  .init = face_det_init,
  .run  = face_det_run,
  .draw_lcd = NULL,          /* the sketch owns the overlay */
  .emit_metadata = NULL,
  .ctx = NULL,
};

IPostProcessor *n6_face_detector(void) { return &s_det_pp; }

/* Alignment: ArcFace 112x112 template, translation and uniform scale */

#define TPL_SIZE      112.0f
#define TPL_IOD        35.2372f     /* 73.5318 - 38.2946 */
#define TPL_EYE_X      55.9132f     /* (38.2946 + 73.5318) / 2 */
#define TPL_EYE_Y      51.5989f     /* (51.6963 + 51.5014) / 2 */

/* Box expansion for the crop when the eye landmarks are unusable. */
#define BOX_FALLBACK   1.2f

n6_roi_t n6_face_align_roi(const n6_face_t *f)
{
  n6_roi_t roi;
  roi.angle = 0.0f;                 /* the cropper cannot rotate */

  if (f == NULL) {
    roi.cx = roi.cy = 0.5f; roi.w = roi.h = 1.0f;
    return roi;
  }

  /* Distances are taken in pixels: normalised x and y have different scales. */
  const float fw = (g_camera_full_w > 0u) ? (float)g_camera_full_w : 1.0f;
  const float fh = (g_camera_full_h > 0u) ? (float)g_camera_full_h : 1.0f;

  const float ex0 = f->lm_x[N6_FACE_LM_EYE_L] * fw;
  const float ey0 = f->lm_y[N6_FACE_LM_EYE_L] * fh;
  const float ex1 = f->lm_x[N6_FACE_LM_EYE_R] * fw;
  const float ey1 = f->lm_y[N6_FACE_LM_EYE_R] * fh;

  const float dx  = ex1 - ex0;
  const float dy  = ey1 - ey0;
  const float iod = sqrtf(dx * dx + dy * dy);

  const float box_px = (f->w * fw > f->h * fh) ? f->w * fw : f->h * fh;
  if (iod < 2.0f || iod > box_px * 2.0f) {
    roi.cx = f->cx;
    roi.cy = f->cy;
    roi.w  = f->w * BOX_FALLBACK;
    roi.h  = f->h * BOX_FALLBACK;
    return roi;
  }

  const float side = iod * (TPL_SIZE / TPL_IOD);        /* crop side, px */
  const float emx  = (ex0 + ex1) * 0.5f;
  const float emy  = (ey0 + ey1) * 0.5f;

  /* The eye midpoint sits at the template's position inside the crop. */
  const float cx_px = emx + (0.5f - TPL_EYE_X / TPL_SIZE) * side;
  const float cy_px = emy + (0.5f - TPL_EYE_Y / TPL_SIZE) * side;

  roi.cx = cx_px / fw;
  roi.cy = cy_px / fh;
  roi.w  = side  / fw;      /* square in pixels */
  roi.h  = side  / fh;
  return roi;
}

/* Embedder post-processor */

static float s_emb[N6_FACE_EMB_DIM];
static bool  s_emb_valid = false;

bool         n6_face_embedding_valid(void) { return s_emb_valid; }
const float *n6_face_embedding(void) { return s_emb_valid ? s_emb : NULL; }

static int32_t face_emb_init(void *ctx, NN_Instance_TypeDef *model)
{
  (void)ctx; (void)model;
  s_emb_valid = false;
  memset(s_emb, 0, sizeof(s_emb));
  return 0;
}

static int32_t face_emb_run(void *ctx, NN_Instance_TypeDef *model)
{
  (void)ctx;
  s_emb_valid = false;

  const LL_Buffer_InfoTypeDef *out = LL_ATON_Output_Buffers_Info(model);
  if (out == NULL || out[0].name == NULL) return -1;

  const uint8_t *start = LL_Buffer_addr_start(&out[0]);
  const uint8_t *end   = LL_Buffer_addr_end(&out[0]);
  if (start == NULL || end == NULL || end <= start) return -1;

  if ((uint32_t)(end - start) / 4u != (uint32_t)N6_FACE_EMB_DIM) return -1;

  const float *v = (const float *)(const void *)start;

  /* L2-normalised, so a later comparison is a dot product. */
  float n2 = 0.0f;
  for (int i = 0; i < N6_FACE_EMB_DIM; i++) n2 += v[i] * v[i];
  if (!(n2 > 1e-12f)) return -1;

  const float inv = 1.0f / sqrtf(n2);
  for (int i = 0; i < N6_FACE_EMB_DIM; i++) s_emb[i] = v[i] * inv;

  s_emb_valid = true;
  return 0;
}

static IPostProcessor s_emb_pp = {
  .init = face_emb_init,
  .run  = face_emb_run,
  .draw_lcd = NULL,
  .emit_metadata = NULL,
  .ctx = NULL,
};

IPostProcessor *n6_face_embedder(void) { return &s_emb_pp; }

/* Drawing helpers */

int n6_face_cam_to_lcd_x(float xn)
{
  return (int)((float)g_lcd_camera_x + xn * (float)g_lcd_camera_w);
}
int n6_face_cam_to_lcd_y(float yn)
{
  return (int)((float)g_lcd_camera_y + yn * (float)g_lcd_camera_h);
}
int n6_face_cam_to_lcd_w(float wn) { return (int)(wn * (float)g_lcd_camera_w); }
int n6_face_cam_to_lcd_h(float hn) { return (int)(hn * (float)g_lcd_camera_h); }
