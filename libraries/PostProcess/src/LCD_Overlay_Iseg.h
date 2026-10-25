/*
 * LCD_Overlay_Iseg.h
 *
 * LCD overlay for YOLOv8 instance segmentation.
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

void LCD_Overlay_Iseg(OVL_Mode_t mode);

/* Banner drawn top centre by LCD_Overlay_Iseg. The text is copied and cut to
 * 39 characters, and NULL or an empty string turns it off. */
void LCD_Overlay_SetBanner(const char *text);

#ifdef __cplusplus
}
#endif
