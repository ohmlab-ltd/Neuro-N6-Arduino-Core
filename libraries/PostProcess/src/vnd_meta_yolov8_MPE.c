/**
 * @file    vnd_meta_yolov8_MPE.c
 * @brief   Multi-person pose estimation metadata builder.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "vnd_meta_yolov8_MPE.h"
#include "app_postprocess_mpe_yolo_v8_ui.h"
#include "NeuroN6.h"
#include "n6_host_contract.h"

/* Inference timestamps */
extern uint32_t ts[];

/* Colour ids of the host palette: 0 green, 1 blue, 2 magenta, 3 orange. */

#define COLOR_HEAD  0
#define COLOR_ARMS  1
#define COLOR_TRUNK 2
#define COLOR_LEGS  3

/* COCO 17-keypoint skeleton: 19 bindings of {kp_a, kp_b, colorId}. */
static const uint8_t s_bindings[][3] = {
  {15, 13, COLOR_LEGS},
  {13, 11, COLOR_LEGS},
  {16, 14, COLOR_LEGS},
  {14, 12, COLOR_LEGS},
  {11, 12, COLOR_TRUNK},
  { 5, 11, COLOR_TRUNK},
  { 6, 12, COLOR_TRUNK},
  { 5,  6, COLOR_ARMS},
  { 5,  7, COLOR_ARMS},
  { 6,  8, COLOR_ARMS},
  { 7,  9, COLOR_ARMS},
  { 8, 10, COLOR_ARMS},
  { 1,  2, COLOR_HEAD},
  { 0,  1, COLOR_HEAD},
  { 0,  2, COLOR_HEAD},
  { 1,  3, COLOR_HEAD},
  { 2,  4, COLOR_HEAD},
  { 3,  5, COLOR_HEAD},
  { 4,  6, COLOR_HEAD},
};

#define NUM_BINDINGS (sizeof(s_bindings) / sizeof(s_bindings[0]))

/* Box, keypoint and binding counts, and keypoint ids, are one byte each. */
N6_STATIC_ASSERT((AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT * AI_POSE_PP_POSE_KEYPOINTS_NB) <= 255,
                 "more keypoints a frame than a one-byte id numbers");
N6_STATIC_ASSERT((AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT * NUM_BINDINGS) <= 255,
                 "more bindings a frame than a one-byte count holds");

static const uint8_t s_kp_color[17] = {
  COLOR_HEAD,  /* 0  nose          */
  COLOR_HEAD,  /* 1  left_eye      */
  COLOR_HEAD,  /* 2  right_eye     */
  COLOR_HEAD,  /* 3  left_ear      */
  COLOR_HEAD,  /* 4  right_ear     */
  COLOR_ARMS,  /* 5  left_shoulder */
  COLOR_ARMS,  /* 6  right_shoulder*/
  COLOR_ARMS,  /* 7  left_elbow    */
  COLOR_ARMS,  /* 8  right_elbow   */
  COLOR_ARMS,  /* 9  left_wrist    */
  COLOR_ARMS,  /* 10 right_wrist   */
  COLOR_TRUNK, /* 11 left_hip      */
  COLOR_TRUNK, /* 12 right_hip     */
  COLOR_LEGS,  /* 13 left_knee     */
  COLOR_LEGS,  /* 14 right_knee    */
  COLOR_LEGS,  /* 15 left_ankle    */
  COLOR_LEGS,  /* 16 right_ankle   */
};

void vnd_meta_build_mpe_on_slot(const mpe_pp_out_t *out, uint32_t inference_ms,
                                const mpe_yolov8_pp_static_param_t *pp_params,
                                vnd_meta_slot_t slot)
{
  vnd_meta_builder_t meta;
  vnd_meta_builder_begin(&meta, slot);

  /* Frame-global TLVs: only slot 0 sends META_INFERENCE_MS and META_BOXES. */
  const bool emit_frame_global = (slot == VND_META_SLOT_MODEL_0);

  if (emit_frame_global) {
    uint16_t inf_ms_u16 = (inference_ms > 65535u) ? 65535u : (uint16_t)inference_ms;
    (void)vnd_meta_builder_put_tlv(&meta, META_INFERENCE_MS, &inf_ms_u16, sizeof(inf_ms_u16));
  }

  if (!out || out->nb_detect == 0)
  {
    vnd_meta_builder_finish(&meta);
    return;
  }

  uint32_t n = (uint32_t)out->nb_detect;
  if (n > (uint32_t)AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT)
    n = (uint32_t)AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT;

  const uint32_t NKP = AI_POSE_PP_POSE_KEYPOINTS_NB;
  const uint32_t NBI = NUM_BINDINGS;

  uint8_t boxes[1 + AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT * 10];
  uint32_t bl = 0;
  boxes[bl++] = 0; /* count placeholder */

  uint8_t kp_buf[1 + AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT * AI_POSE_PP_POSE_KEYPOINTS_NB * 7];
  uint32_t kl = 0;
  kp_buf[kl++] = 0; /* count placeholder */

  uint8_t bi_buf[1 + AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT * NUM_BINDINGS * 3];
  uint32_t bil = 0;
  bi_buf[bil++] = 0; /* count placeholder */

  uint8_t kept_persons = 0;
  uint16_t total_kps = 0;
  uint16_t total_binds = 0;

  for (uint32_t d = 0; d < n; d++)
  {
    const mpe_pp_outBuffer_t *det = &out->pOutBuff[d];
    if (!det || !det->pKeyPoints) continue;
    if (pp_params && det->conf < pp_params->conf_threshold) continue;

    const uint8_t id_base = (uint8_t)(kept_persons * NKP);

    /* NN-normalised to camera-normalised, through the two corners. */
    float32_t nn_l = det->x_center - 0.5f * det->width;
    float32_t nn_r = det->x_center + 0.5f * det->width;
    float32_t nn_t = det->y_center - 0.5f * det->height;
    float32_t nn_b = det->y_center + 0.5f * det->height;

    float32_t cam_l = vnd_meta_clamp01(nn_norm_to_cam_norm_x(nn_l));
    float32_t cam_r = vnd_meta_clamp01(nn_norm_to_cam_norm_x(nn_r));
    float32_t cam_t = vnd_meta_clamp01(nn_norm_to_cam_norm_y(nn_t));
    float32_t cam_b = vnd_meta_clamp01(nn_norm_to_cam_norm_y(nn_b));

    float32_t xc_f = 0.5f * (cam_l + cam_r);
    float32_t yc_f = 0.5f * (cam_t + cam_b);
    float32_t w_f  = cam_r - cam_l;
    float32_t h_f  = cam_b - cam_t;

    vnd_meta_put_u16_le(&boxes[bl], vnd_meta_f01_to_u16(xc_f)); bl += 2;
    vnd_meta_put_u16_le(&boxes[bl], vnd_meta_f01_to_u16(yc_f)); bl += 2;
    vnd_meta_put_u16_le(&boxes[bl], vnd_meta_f01_to_u16(w_f));  bl += 2;
    vnd_meta_put_u16_le(&boxes[bl], vnd_meta_f01_to_u16(h_f));  bl += 2;
    boxes[bl++] = vnd_meta_conf_to_q8(det->conf);
    boxes[bl++] = (uint8_t)det->class_index;

    for (uint32_t k = 0; k < NKP; k++)
    {
      const mpe_pp_keyPoints_t *kp = &det->pKeyPoints[k];

      /* vnd_meta_f01_to_u16 clamps each to the frame. */
      float32_t kx_cam = nn_norm_to_cam_norm_x(kp->x);
      float32_t ky_cam = nn_norm_to_cam_norm_y(kp->y);

      kp_buf[kl++] = id_base + (uint8_t)k;            /* id       */
      kp_buf[kl++] = s_kp_color[k];                    /* colorId  */
      vnd_meta_put_u16_le(&kp_buf[kl],
                          vnd_meta_f01_to_u16(kx_cam));
      kl += 2;                                          /* x u16le  */
      vnd_meta_put_u16_le(&kp_buf[kl],
                          vnd_meta_f01_to_u16(ky_cam));
      kl += 2;                                          /* y u16le  */
      kp_buf[kl++] = vnd_meta_conf_to_q8(kp->conf);   /* conf u8  */
    }
    total_kps += (uint16_t)NKP;

    for (uint32_t b = 0; b < NBI; b++)
    {
      bi_buf[bil++] = id_base + s_bindings[b][0];  /* kp_a     */
      bi_buf[bil++] = id_base + s_bindings[b][1];  /* kp_b     */
      bi_buf[bil++] = s_bindings[b][2];             /* colorId  */
    }
    total_binds += (uint16_t)NBI;

    kept_persons++;
  }

  kp_buf[0] = (uint8_t)total_kps;
  (void)vnd_meta_builder_put_tlv(&meta, META_KEYPOINTS, kp_buf, (uint16_t)kl);

  bi_buf[0] = (uint8_t)total_binds;
  (void)vnd_meta_builder_put_tlv(&meta, META_BINDINGS, bi_buf, (uint16_t)bil);

  if (emit_frame_global) {
    boxes[0] = kept_persons;
    (void)vnd_meta_builder_put_tlv(&meta, META_BOXES, boxes, (uint16_t)bl);
  }

  vnd_meta_builder_finish(&meta);
}

void Display_NS_Yolo_V8_MPE_OnSlot(vnd_meta_slot_t slot)
{
  vnd_meta_build_mpe_on_slot(&mpe_pp_output, ts[1] - ts[0], &mpe_pp_params, slot);
}
