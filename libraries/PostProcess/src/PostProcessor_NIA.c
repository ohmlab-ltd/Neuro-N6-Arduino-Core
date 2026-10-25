/**
 ******************************************************************************
 * @file    PostProcessor_NIA.c
 * @brief   Singleton IPostProcessor factory for the Nia detector.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "PostProcessor_NIA.h"

#include "app_postprocess_nia_ui.h"
#include "vnd_meta_nia.h"
#include "LCD_Overlay_Nia.h"

typedef struct {
  float           conf;
  float           iou;
  vnd_meta_slot_t slot;
} nia_state_t;

static nia_state_t g_nia_state = {
  .conf = NIA_PP_CONF_THRESHOLD,
  .iou  = NIA_PP_IOU_THRESHOLD,
  .slot = VND_META_SLOT_PRIMARY,
};

static int32_t nia_pp_init(void *ctx, NN_Instance_TypeDef *model)
{
  (void)model;
  nia_state_t *s = (nia_state_t *)ctx;
  return app_postprocess_nia_ui_init(s->conf, s->iou);
}

static int32_t nia_pp_run(void *ctx, NN_Instance_TypeDef *model)
{
  (void)ctx;
  return app_postprocess_nia_ui_run(model);
}

static void nia_pp_emit_metadata(void *ctx)
{
  nia_state_t *s = (nia_state_t *)ctx;
  Display_NS_Nia_OnSlot(s->slot);
}

static void nia_pp_draw_lcd(void *ctx, OVL_Mode_t mode)
{
  (void)ctx;
  LCD_Overlay_Nia(mode);
}

static IPostProcessor g_nia_pp = {
  .init          = nia_pp_init,
  .run           = nia_pp_run,
  .draw_lcd      = nia_pp_draw_lcd,
  .emit_metadata = nia_pp_emit_metadata,
  .ctx           = &g_nia_state,
};

IPostProcessor *PostProcess_NIA(float conf, float iou)
{
  return PostProcess_NIA_OnSlot(conf, iou, VND_META_SLOT_PRIMARY);
}

IPostProcessor *PostProcess_NIA_OnSlot(float conf, float iou, vnd_meta_slot_t slot)
{
  g_nia_state.conf = conf;
  g_nia_state.iou  = iou;
  g_nia_state.slot = slot;
  return &g_nia_pp;
}
