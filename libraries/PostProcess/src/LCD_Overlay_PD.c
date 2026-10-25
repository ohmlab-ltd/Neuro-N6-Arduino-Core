/**
 * @file    LCD_Overlay_PD.c
 * @brief   LCD overlay for the MediaPipe palm detector.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "LCD_Overlay_PD.h"
#include "NeuroN6.h"
#include "app_postprocess_pd_ui.h"
#include "ohmlab_lcd_draw.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Inference start and end ticks, ms. */
extern uint32_t ts[2];

#define LCD_OVL_W   800u
#define LCD_OVL_H   480u

#define PD_ARGB4444(a, r, g, b) \
    ((uint16_t)(((a) & 0xFu) << 12 | ((r) & 0xFu) << 8 | \
                ((g) & 0xFu) <<  4 | ((b) & 0xFu)))

#define PD_CLEAR        PD_ARGB4444(0x0, 0x0, 0x0, 0x0)
#define PD_HUD_BG       PD_ARGB4444(0x8, 0x0, 0x0, 0x0)

#define HUD_PAD_X   4
#define HUD_PAD_Y   3
#define PD_KP_RADIUS    2
#define BINDING_WIDTH   3   /* must be odd */

#define COLOR_GREEN   0
#define COLOR_BLUE    1
#define COLOR_MAGENTA 2
#define COLOR_ORANGE  3

static inline uint16_t color_id_to_argb4444(int color_id)
{
    switch (color_id)
    {
        default:
        case COLOR_GREEN:   return PD_ARGB4444(0xF, 0x0, 0xF, 0x0);
        case COLOR_BLUE:    return PD_ARGB4444(0xF, 0x0, 0x0, 0xF);
        case COLOR_MAGENTA: return PD_ARGB4444(0xF, 0xF, 0x0, 0xF);
        case COLOR_ORANGE:  return PD_ARGB4444(0xF, 0xF, 0x8, 0x0);
    }
}

/* Keypoint a, keypoint b, colour id. Matches s_bindings in vnd_meta_pd.c. */
static const int s_bindings[][3] = {
    { 0, 1, COLOR_BLUE    },
    { 0, 2, COLOR_BLUE    },
    { 0, 3, COLOR_BLUE    },
    { 0, 4, COLOR_BLUE    },
    { 1, 2, COLOR_GREEN   },
    { 2, 3, COLOR_GREEN   },
    { 3, 4, COLOR_GREEN   },
    { 0, 5, COLOR_MAGENTA },
    { 5, 6, COLOR_MAGENTA },
};
#define NUM_BINDINGS (sizeof(s_bindings) / sizeof(s_bindings[0]))

static const int s_kp_color[AI_PD_PP_NB_KEYPOINTS] = {
    COLOR_ORANGE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE,
    COLOR_BLUE,   COLOR_MAGENTA, COLOR_MAGENTA,
};

static void pd_fill_rect(uint16_t *buf, int x0, int y0, int x1, int y1, uint16_t color)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > (int)LCD_OVL_W) x1 = (int)LCD_OVL_W;
    if (y1 > (int)LCD_OVL_H) y1 = (int)LCD_OVL_H;

    for (int y = y0; y < y1; y++)
    {
        uint16_t *row = buf + (uint32_t)y * LCD_OVL_W;
        for (int x = x0; x < x1; x++)
            row[x] = color;
    }
}

static void pd_draw_bbox(int x0, int y0, int x1, int y1)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > (int)LCD_OVL_W) x1 = (int)LCD_OVL_W;
    if (y1 > (int)LCD_OVL_H) y1 = (int)LCD_OVL_H;

    const int w = x1 - x0;
    const int h = y1 - y0;
    if (w <= 0 || h <= 0) return;

    ohm_lcd_rect_white((uint32_t)x0, (uint32_t)y0, (uint32_t)w, (uint32_t)h);
}

static void pd_draw_keypoint(uint16_t *buf, int cx, int cy, uint16_t color)
{
    pd_fill_rect(buf, cx - PD_KP_RADIUS, cy - PD_KP_RADIUS,
                 cx + PD_KP_RADIUS + 1, cy + PD_KP_RADIUS + 1, color);
}

/* Bresenham line. */
static void pd_draw_line(uint16_t *buf, int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    dy = -dy;
    int err = dx + dy;

    for (;;)
    {
        if (x0 >= 0 && x0 < (int)LCD_OVL_W && y0 >= 0 && y0 < (int)LCD_OVL_H)
            buf[(uint32_t)y0 * LCD_OVL_W + (uint32_t)x0] = color;
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void pd_draw_thick_line(uint16_t *buf, int x0, int y0, int x1, int y1,
                               int width, uint16_t color)
{
    pd_draw_line(buf, x0, y0, x1, y1, color);
    if (width <= 1) return;

    const int half = width / 2;
    const int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    const int dy = (y1 > y0) ? (y1 - y0) : (y0 - y1);

    for (int i = 1; i <= half; i++)
    {
        if (dy > dx) {
            pd_draw_line(buf, x0 + i, y0, x1 + i, y1, color);
            pd_draw_line(buf, x0 - i, y0, x1 - i, y1, color);
        } else {
            pd_draw_line(buf, x0, y0 + i, x1, y1 + i, color);
            pd_draw_line(buf, x0, y0 - i, x1, y1 - i, color);
        }
    }
}

void LCD_Overlay_PD(OVL_Mode_t mode)
{
    uint16_t *buf = OVL_GetBackBuffer();
    if (buf == NULL) return;

    ohm_lcd_bind_argb4444(buf, LCD_OVL_W, LCD_OVL_H, LCD_OVL_W);

    if (mode == OVL_RELOAD_CLEAR || mode == OVL_CLEAR)
    {
        const uint32_t n_pixels = LCD_OVL_W * LCD_OVL_H;
        for (uint32_t i = 0; i < n_pixels; i++)
            buf[i] = PD_CLEAR;
    }

    {
        char hud[40];
        const uint32_t inf_ms = ts[1] - ts[0];
        snprintf(hud, sizeof(hud), "palms:%lu  %lums",
                 (unsigned long)pd_ui_nb_detections, (unsigned long)inf_ms);
        const int box_w = ((int)strlen(hud) * (int)Font16.Width) + (2 * HUD_PAD_X);
        const int box_h = (int)Font16.Height + (2 * HUD_PAD_Y);
        pd_fill_rect(buf, 8, 8, 8 + box_w, 8 + box_h, PD_HUD_BG);
        ohm_lcd_text((uint32_t)(8 + HUD_PAD_X), (uint32_t)(8 + HUD_PAD_Y), hud);
    }

    for (uint32_t d = 0; d < pd_ui_nb_detections; d++)
    {
        const pd_ui_detection_t *det = &pd_ui_detections[d];

        const int bx0 = nn_norm_to_lcd_x(det->x_center - 0.5f * det->width);
        const int bx1 = nn_norm_to_lcd_x(det->x_center + 0.5f * det->width);
        const int by0 = nn_norm_to_lcd_y(det->y_center - 0.5f * det->height);
        const int by1 = nn_norm_to_lcd_y(det->y_center + 0.5f * det->height);
        pd_draw_bbox(bx0, by0, bx1, by1);

        for (uint32_t b = 0; b < NUM_BINDINGS; b++)
        {
            const int ka = s_bindings[b][0];
            const int kb = s_bindings[b][1];
            const uint16_t col = color_id_to_argb4444(s_bindings[b][2]);

            const int ax = nn_norm_to_lcd_x(det->kps[ka].x);
            const int ay = nn_norm_to_lcd_y(det->kps[ka].y);
            const int bx = nn_norm_to_lcd_x(det->kps[kb].x);
            const int by = nn_norm_to_lcd_y(det->kps[kb].y);

            pd_draw_thick_line(buf, ax, ay, bx, by, BINDING_WIDTH, col);
        }

        for (uint32_t k = 0; k < (uint32_t)AI_PD_PP_NB_KEYPOINTS; k++)
        {
            const int kx = nn_norm_to_lcd_x(det->kps[k].x);
            const int ky = nn_norm_to_lcd_y(det->kps[k].y);
            pd_draw_keypoint(buf, kx, ky, color_id_to_argb4444(s_kp_color[k]));
        }
    }

    if (mode == OVL_RELOAD_CLEAR || mode == OVL_RELOAD)
    {
        OVL_Commit();
    }
}
