/*
 * LCD_Overlay_MPE.c
 *
 * LCD overlay for YOLOv8 multi-person pose estimation.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "LCD_Overlay_MPE.h"
#include "NeuroN6.h"
#include "app_postprocess_mpe_yolo_v8_ui.h"
#include "ohmlab_lcd_draw.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Inference start and end ticks, ms. */
extern uint32_t ts[2];

#define LCD_OVL_W   800u
#define LCD_OVL_H   480u

#define MPE_ARGB4444(a, r, g, b) \
    ((uint16_t)(((a) & 0xFu) << 12 | ((r) & 0xFu) << 8 | \
                ((g) & 0xFu) <<  4 | ((b) & 0xFu)))

#define MPE_WHITE       MPE_ARGB4444(0xF, 0xF, 0xF, 0xF)
#define MPE_CLEAR       MPE_ARGB4444(0x0, 0x0, 0x0, 0x0)
#define MPE_HUD_BG      MPE_ARGB4444(0x8, 0x0, 0x0, 0x0)

#define HUD_PAD_X  4
#define HUD_PAD_Y  3

#define MPE_KP_RADIUS   2
#define BINDING_WIDTH   3   /* must be odd */

#define COLOR_HEAD  0
#define COLOR_ARMS  1
#define COLOR_TRUNK 2
#define COLOR_LEGS  3

static inline uint16_t color_id_to_argb4444(int color_id)
{
    switch (color_id)
    {
        default:
        case COLOR_HEAD:  return MPE_ARGB4444(0xF, 0x0, 0xF, 0x0); /* green   */
        case COLOR_ARMS:  return MPE_ARGB4444(0xF, 0x0, 0x0, 0xF); /* blue    */
        case COLOR_TRUNK: return MPE_ARGB4444(0xF, 0xF, 0x0, 0xF); /* magenta */
        case COLOR_LEGS:  return MPE_ARGB4444(0xF, 0xF, 0x8, 0x0); /* orange  */
    }
}

/* Keypoint a, keypoint b, colour id. COCO-17 keypoints: 0 nose, 1 and 2 eyes,
 * 3 and 4 ears, 5 and 6 shoulders, 7 and 8 elbows, 9 and 10 wrists, 11 and 12
 * hips, 13 and 14 knees, 15 and 16 ankles, each pair left then right. */
static const int s_bindings[][3] = {
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

#define MPE_BINDINGS_NB ((int)(sizeof(s_bindings) / sizeof(s_bindings[0])))

static void mpe_fill_rect(uint16_t *buf,
                          int x0, int y0, int x1, int y1,
                          uint16_t color)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > (int)LCD_OVL_W) x1 = (int)LCD_OVL_W;
    if (y1 > (int)LCD_OVL_H) y1 = (int)LCD_OVL_H;

    for (int y = y0; y < y1; y++)
    {
        uint16_t *row = buf + y * LCD_OVL_W;
        for (int x = x0; x < x1; x++)
            row[x] = color;
    }
}

static void mpe_draw_bbox(uint16_t *buf, int x0, int y0, int x1, int y1)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > (int)LCD_OVL_W) x1 = (int)LCD_OVL_W;
    if (y1 > (int)LCD_OVL_H) y1 = (int)LCD_OVL_H;

    int w = x1 - x0;
    int h = y1 - y0;
    if (w <= 0 || h <= 0) return;

    ohm_lcd_rect_white((uint32_t)x0, (uint32_t)y0, (uint32_t)w, (uint32_t)h);
}

static void mpe_draw_keypoint(uint16_t *buf, int cx, int cy, uint16_t color)
{
    mpe_fill_rect(buf,
                  cx - MPE_KP_RADIUS, cy - MPE_KP_RADIUS,
                  cx + MPE_KP_RADIUS + 1, cy + MPE_KP_RADIUS + 1,
                  color);
}

/* Bresenham line. */
static void mpe_draw_line(uint16_t *buf, int x0, int y0, int x1, int y1,
                          uint16_t color)
{
    int dx  =  (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int dy  = -((y1 > y0) ? (y1 - y0) : (y0 - y1));
    int sx  = (x0 < x1) ? 1 : -1;
    int sy  = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (1)
    {
        if (x0 >= 0 && x0 < (int)LCD_OVL_W &&
            y0 >= 0 && y0 < (int)LCD_OVL_H)
        {
            buf[y0 * LCD_OVL_W + x0] = color;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void mpe_draw_thick_line(uint16_t *buf,
                                int x0, int y0, int x1, int y1,
                                int thickness, uint16_t color)
{
    if (thickness < 1) thickness = 1;
    if ((thickness & 1) == 0) thickness += 1;
    int half = (thickness - 1) / 2;

    int adx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int ady = (y1 > y0) ? (y1 - y0) : (y0 - y1);

    mpe_draw_line(buf, x0, y0, x1, y1, color);

    for (int i = 1; i <= half; i++)
    {
        if (ady > adx)
        {
            mpe_draw_line(buf, x0 + i, y0, x1 + i, y1, color);
            mpe_draw_line(buf, x0 - i, y0, x1 - i, y1, color);
        }
        else
        {
            mpe_draw_line(buf, x0, y0 + i, x1, y1 + i, color);
            mpe_draw_line(buf, x0, y0 - i, x1, y1 - i, color);
        }
    }
}

void LCD_Overlay_MPE(OVL_Mode_t mode)
{
    uint16_t *buf = OVL_GetBackBuffer();
    if (buf == NULL) return;

    ohm_lcd_bind_argb4444(buf, LCD_OVL_W, LCD_OVL_H, LCD_OVL_W);

    if (mode == OVL_RELOAD_CLEAR || mode == OVL_CLEAR)
    {
        uint32_t n_pixels = LCD_OVL_W * LCD_OVL_H;
        for (uint32_t i = 0; i < n_pixels; i++)
            buf[i] = MPE_CLEAR;
    }

    {
        char inf_str[32];
        uint32_t inf_ms = ts[1] - ts[0];
        snprintf(inf_str, sizeof(inf_str), "inference: %lums", (unsigned long)inf_ms);

        int tx = 8;
        int ty = 8;
        int len = (int)strlen(inf_str);
        int box_w = (len * (int)Font16.Width)  + (2 * HUD_PAD_X);
        int box_h = ((int)Font16.Height)       + (2 * HUD_PAD_Y);

        if (tx + box_w > (int)LCD_OVL_W) box_w = (int)LCD_OVL_W - tx;
        if (ty + box_h > (int)LCD_OVL_H) box_h = (int)LCD_OVL_H - ty;

        if (box_w > 0 && box_h > 0)
            mpe_fill_rect(buf, tx, ty, tx + box_w, ty + box_h, MPE_HUD_BG);

        ohm_lcd_text((uint32_t)(tx + HUD_PAD_X),
                     (uint32_t)(ty + HUD_PAD_Y),
                     inf_str);
    }

    int n_det = mpe_pp_output.nb_detect;

    for (int d = 0; d < n_det; d++)
    {
        const mpe_pp_outBuffer_t *det = &mpe_pp_output.pOutBuff[d];
        if (det == NULL || det->pKeyPoints == NULL) continue;

        int bx0 = nn_norm_to_lcd_x(det->x_center - det->width  * 0.5f);
        int by0 = nn_norm_to_lcd_y(det->y_center - det->height * 0.5f);
        int bx1 = nn_norm_to_lcd_x(det->x_center + det->width  * 0.5f);
        int by1 = nn_norm_to_lcd_y(det->y_center + det->height * 0.5f);

        mpe_draw_bbox(buf, bx0, by0, bx1, by1);

        for (int bi = 0; bi < MPE_BINDINGS_NB; bi++)
        {
            int ia = s_bindings[bi][0];
            int ib = s_bindings[bi][1];
            uint16_t col = color_id_to_argb4444(s_bindings[bi][2]);

            if (ia >= AI_POSE_PP_POSE_KEYPOINTS_NB ||
                ib >= AI_POSE_PP_POSE_KEYPOINTS_NB) continue;

            const mpe_pp_keyPoints_t *ka = &det->pKeyPoints[ia];
            const mpe_pp_keyPoints_t *kb = &det->pKeyPoints[ib];

            if (ka->conf < AI_MPE_YOLOV8_PP_CONF_THRESHOLD ||
                kb->conf < AI_MPE_YOLOV8_PP_CONF_THRESHOLD) continue;

            int ax  = nn_norm_to_lcd_x(ka->x);
            int ay  = nn_norm_to_lcd_y(ka->y);
            int bxp = nn_norm_to_lcd_x(kb->x);
            int byp = nn_norm_to_lcd_y(kb->y);

            mpe_draw_thick_line(buf, ax, ay, bxp, byp, BINDING_WIDTH, col);
            mpe_draw_keypoint(buf, ax, ay, col);
            mpe_draw_keypoint(buf, bxp, byp, col);
        }
    }

    if (mode == OVL_RELOAD_CLEAR || mode == OVL_RELOAD)
    {
        OVL_Commit();
    }
}
