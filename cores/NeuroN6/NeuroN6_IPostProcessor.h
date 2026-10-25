/**
 ******************************************************************************
 * @file    NeuroN6_IPostProcessor.h
 * @brief   Post-processor interface for Vision.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef NEURON6_IPOSTPROCESSOR_H
#define NEURON6_IPOSTPROCESSOR_H

#include <stdint.h>

#include "ll_aton_runtime.h"

/* Also in LCD_Overlay_Iseg.h and LCD_Overlay_MPE.h. The copies must match. */
#ifndef OVL_MODE_DEFINED
#define OVL_MODE_DEFINED
typedef enum {
  OVL_RELOAD_CLEAR,         /* clear, commit after drawing */
  OVL_RELOAD,               /* no clear, commit after drawing */
  OVL_CLEAR,                /* clear, no commit */
  OVL_RELOAD_CLEAR_NONE     /* no clear, no commit */
} OVL_Mode_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Vision calls these in the order listed. */
typedef struct IPostProcessor {
  /* Once, before the first run(). Returns 0, or a negative value on error. */
  int32_t (*init)(void *ctx, NN_Instance_TypeDef *model);

  /* Per frame, after inference. Returns 0 on success. */
  int32_t (*run)(void *ctx, NN_Instance_TypeDef *model);

  /* Per frame, after run(). Draws to the LCD overlay. May be NULL. */
  void    (*draw_lcd)(void *ctx, OVL_Mode_t mode);

  /* Per frame, after draw_lcd(). Publishes the TLV metadata. May be NULL. */
  void    (*emit_metadata)(void *ctx);

  /* Passed back to every call. */
  void   *ctx;
} IPostProcessor;

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_IPOSTPROCESSOR_H */
