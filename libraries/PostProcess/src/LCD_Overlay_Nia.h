/*
 * LCD_Overlay_Nia.h
 *
 * LCD overlay for the Nia detector.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Also defined in other headers. The copies must match. */
#ifndef OVL_MODE_DEFINED
#define OVL_MODE_DEFINED
typedef enum {
    OVL_RELOAD_CLEAR,       /* clear, commit after drawing */
    OVL_RELOAD,             /* no clear, commit after drawing */
    OVL_CLEAR,              /* clear, no commit */
    OVL_RELOAD_CLEAR_NONE   /* no clear, no commit */
} OVL_Mode_t;
#endif

/* Draws nia_pp_output. PostProcess_NIA's draw_lcd hook calls it. */
void LCD_Overlay_Nia(OVL_Mode_t mode);

struct nia_pp_out;
void LCD_Overlay_Nia_From(const struct nia_pp_out *out, OVL_Mode_t mode);

#ifdef __cplusplus
}
#endif
