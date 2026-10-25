/**
 * @file    vnd_meta_nia.c
 * @brief   Nia detection metadata builder.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "vnd_meta_nia.h"
#include "app_postprocess_nia_ui.h"
#include "NeuroN6.h"
#include "n6_host_contract.h"
#include <string.h>

/* The box count is one byte. */
N6_STATIC_ASSERT(NIA_PP_MAX_DET <= 255, "more boxes than a one-byte count holds");

/* Inference timestamps: ts[0] before, ts[1] after. */
extern uint32_t ts[];

/* COCO-80 names. The default table is weak, so a sketch can replace it. */
#ifndef NB_CLASSES
#define NB_CLASSES (80)
#endif
extern const char *classes_table[NB_CLASSES];

static inline const char *nia_class_name(uint32_t cls)
{
  if (cls < (uint32_t)NB_CLASSES) return classes_table[cls];
  return "unknown";
}

/* A count byte, then 10 bytes per box. Used from the Vision loop only. */
__attribute__((section(".ext_psram")))
static uint8_t s_boxes[1 + (NIA_PP_MAX_DET * (2 * 4 + 2))];

void vnd_meta_build_nia_on_slot(const nia_pp_out_t *out, uint32_t inference_ms,
                                const nia_pp_params_t *params,
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

  if (!out || out->nb_detect <= 0) {
    vnd_meta_builder_finish(&meta);
    return;
  }

  uint32_t n = (uint32_t)out->nb_detect;
  if (n > (uint32_t)NIA_PP_MAX_DET) n = (uint32_t)NIA_PP_MAX_DET;

  uint32_t bl = 0;
  s_boxes[bl++] = 0;   /* count placeholder */
  uint8_t kept = 0;

  for (uint32_t i = 0; i < n; i++) {
    const nia_det_t *d = &out->pOutBuff[i];

    if (params && d->conf < params->conf_threshold) continue;

    /* NN-normalised to camera-normalised, through the two corners. */
    float32_t nn_l = d->x_center - 0.5f * d->width;
    float32_t nn_r = d->x_center + 0.5f * d->width;
    float32_t nn_t = d->y_center - 0.5f * d->height;
    float32_t nn_b = d->y_center + 0.5f * d->height;

    float32_t cam_l = vnd_meta_clamp01(nn_norm_to_cam_norm_x(nn_l));
    float32_t cam_r = vnd_meta_clamp01(nn_norm_to_cam_norm_x(nn_r));
    float32_t cam_t = vnd_meta_clamp01(nn_norm_to_cam_norm_y(nn_t));
    float32_t cam_b = vnd_meta_clamp01(nn_norm_to_cam_norm_y(nn_b));

    float32_t xc_f = 0.5f * (cam_l + cam_r);
    float32_t yc_f = 0.5f * (cam_t + cam_b);
    float32_t w_f  = cam_r - cam_l;
    float32_t h_f  = cam_b - cam_t;

    uint16_t xc = vnd_meta_f01_to_u16(xc_f);
    uint16_t yc = vnd_meta_f01_to_u16(yc_f);
    uint16_t ww = vnd_meta_f01_to_u16(w_f);
    uint16_t hh = vnd_meta_f01_to_u16(h_f);
    uint8_t  confq = vnd_meta_conf_to_q8(d->conf);

    /* obj_id is the box's index in META_BOXES. */
    const char *name = nia_class_name((uint32_t)d->class_index);
    uint16_t name_len = (uint16_t)strlen(name);
    if (name_len > 200) name_len = 200;

    if (emit_frame_global) {
      uint8_t tmp[1 + 200];
      tmp[0] = kept;
      memcpy(&tmp[1], name, name_len);
      if (!vnd_meta_builder_put_tlv(&meta, META_ISEG_CLASSSTR, tmp,
                                    (uint16_t)(1u + name_len))) {
        /* slot full */
        break;
      }
    }

    vnd_meta_put_u16_le(&s_boxes[bl], xc); bl += 2;
    vnd_meta_put_u16_le(&s_boxes[bl], yc); bl += 2;
    vnd_meta_put_u16_le(&s_boxes[bl], ww); bl += 2;
    vnd_meta_put_u16_le(&s_boxes[bl], hh); bl += 2;
    s_boxes[bl++] = confq;
    s_boxes[bl++] = (uint8_t)d->class_index;
    kept++;
  }

  s_boxes[0] = kept;
  if (emit_frame_global) {
    (void)vnd_meta_builder_put_tlv(&meta, META_BOXES, s_boxes, (uint16_t)bl);
  }
  vnd_meta_builder_finish(&meta);
}

void Display_NS_Nia_OnSlot(vnd_meta_slot_t slot)
{
  vnd_meta_build_nia_on_slot(&nia_pp_output, ts[1] - ts[0], &nia_pp_params, slot);
}
