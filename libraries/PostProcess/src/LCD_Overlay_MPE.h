/*
 * LCD_Overlay_MPE.h
 *
 * LCD overlay for YOLOv8 multi-person pose estimation.
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

void LCD_Overlay_MPE(OVL_Mode_t mode);

#ifdef __cplusplus
}
#endif
