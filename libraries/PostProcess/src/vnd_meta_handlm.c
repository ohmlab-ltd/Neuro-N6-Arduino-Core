/**
 * @file    vnd_meta_handlm.c
 * @brief   Hand-landmark metadata: META_KEYPOINTS and META_BINDINGS.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "vnd_meta_handlm.h"
#include "NeuroN6.h"

/* Inference timestamps */
extern uint32_t ts[];

/* Colour ids of the host palette. */
#define COLOR_GREEN   0
#define COLOR_BLUE    1
#define COLOR_MAGENTA 2
#define COLOR_ORANGE  3

/* MediaPipe hand landmarks: 0 wrist, 1-4 thumb, 5-8 index, 9-12 middle,
 * 13-16 ring, 17-20 pinky, each from base to tip. */
static const uint8_t s_bindings[][3] = {
  {  0,  1, COLOR_MAGENTA }, {  1,  2, COLOR_MAGENTA },
  {  2,  3, COLOR_MAGENTA }, {  3,  4, COLOR_MAGENTA },   /* thumb  */
  {  0,  5, COLOR_BLUE    }, {  5,  6, COLOR_BLUE    },
  {  6,  7, COLOR_BLUE    }, {  7,  8, COLOR_BLUE    },   /* index  */
  {  9, 10, COLOR_GREEN   }, { 10, 11, COLOR_GREEN   },
  { 11, 12, COLOR_GREEN   },                              /* middle */
  { 13, 14, COLOR_ORANGE  }, { 14, 15, COLOR_ORANGE  },
  { 15, 16, COLOR_ORANGE  },                              /* ring   */
  {  0, 17, COLOR_GREEN   }, { 17, 18, COLOR_GREEN   },
  { 18, 19, COLOR_GREEN   }, { 19, 20, COLOR_GREEN   },   /* pinky  */
  {  5,  9, COLOR_ORANGE  }, {  9, 13, COLOR_ORANGE  },
  { 13, 17, COLOR_ORANGE  },                              /* palm arch */
};
#define NUM_BINDINGS (sizeof(s_bindings) / sizeof(s_bindings[0]))

static const uint8_t s_kp_color[AI_HANDLM_NB_LANDMARKS] = {
  COLOR_ORANGE,                                            /* 0  wrist  */
  COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
  COLOR_BLUE,    COLOR_BLUE,    COLOR_BLUE,    COLOR_BLUE,
  COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,
  COLOR_ORANGE,  COLOR_ORANGE,  COLOR_ORANGE,  COLOR_ORANGE,
  COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,
};

static inline float32_t clamp01(float32_t v)
{
  if (v < 0.f) return 0.f;
  if (v > 1.f) return 1.f;
  return v;
}

void vnd_meta_build_handlm_on_slot(const handlm_result_t *res,
                                   uint32_t inference_ms, vnd_meta_slot_t slot)
{
  vnd_meta_builder_t meta;
  vnd_meta_builder_begin(&meta, slot);

  /* Frame-global TLV: only slot 0 sends META_INFERENCE_MS. */
  if (slot == VND_META_SLOT_MODEL_0) {
    uint16_t inf_ms_u16 = (inference_ms > 65535u) ? 65535u : (uint16_t)inference_ms;
    (void)vnd_meta_builder_put_tlv(&meta, META_INFERENCE_MS, &inf_ms_u16, sizeof(inf_ms_u16));
  }

  if (res == NULL || !res->valid) {
    vnd_meta_builder_finish(&meta);   /* no hand */
    return;
  }

  const uint32_t NKP = (uint32_t)AI_HANDLM_NB_LANDMARKS;
  const uint32_t NBI = (uint32_t)NUM_BINDINGS;

  uint8_t  kp_buf[1 + AI_HANDLM_NB_LANDMARKS * 7];
  uint32_t kl = 0u;
  kp_buf[kl++] = (uint8_t)NKP;

  uint8_t  bi_buf[1 + NUM_BINDINGS * 3];
  uint32_t bil = 0u;
  bi_buf[bil++] = (uint8_t)NBI;

  /* Landmarks are relative to the ROI crop Vision.runRoi() published. */
  for (uint32_t k = 0u; k < NKP; k++) {
    const float32_t kx = clamp01(nn_norm_to_cam_norm_x(res->kps[k].x));
    const float32_t ky = clamp01(nn_norm_to_cam_norm_y(res->kps[k].y));

    kp_buf[kl++] = (uint8_t)k;                                  /* id      */
    kp_buf[kl++] = s_kp_color[k];                               /* colorId */
    vnd_meta_put_u16_le(&kp_buf[kl], vnd_meta_f01_to_u16(kx)); kl += 2;
    vnd_meta_put_u16_le(&kp_buf[kl], vnd_meta_f01_to_u16(ky)); kl += 2;
    kp_buf[kl++] = vnd_meta_conf_to_q8(res->score);             /* conf    */
  }

  for (uint32_t b = 0u; b < NBI; b++) {
    bi_buf[bil++] = s_bindings[b][0];
    bi_buf[bil++] = s_bindings[b][1];
    bi_buf[bil++] = s_bindings[b][2];
  }

  (void)vnd_meta_builder_put_tlv(&meta, META_KEYPOINTS, kp_buf, (uint16_t)kl);
  (void)vnd_meta_builder_put_tlv(&meta, META_BINDINGS,  bi_buf, (uint16_t)bil);

  vnd_meta_builder_finish(&meta);
}

void Display_NS_HandLandmarks_OnSlot(vnd_meta_slot_t slot)
{
  vnd_meta_build_handlm_on_slot(&handlm_output, ts[1] - ts[0], slot);
}
