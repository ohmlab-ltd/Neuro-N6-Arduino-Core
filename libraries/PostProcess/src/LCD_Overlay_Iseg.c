/*
 * LCD_Overlay_Iseg.c
 *
 * LCD overlay for YOLOv8 instance segmentation.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "LCD_Overlay_Iseg.h"
#include "NeuroN6.h"
#include "app_postprocess_iseg_yolo_v8_ui.h"
#include "ohmlab_lcd_draw.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Inference start and end ticks, ms. */
extern uint32_t ts[2];

#define LCD_OVL_W   800u
#define LCD_OVL_H   480u

#define ISEG_ARGB4444(a, r, g, b) \
    ((uint16_t)(((a) & 0xFu) << 12 | ((r) & 0xFu) << 8 | \
                ((g) & 0xFu) <<  4 | ((b) & 0xFu)))

#define ISEG_ALPHA_HALF   0x8u
#define ISEG_ALPHA_OPAQUE 0xFu

#define ISEG_WHITE      ISEG_ARGB4444(0xF, 0xF, 0xF, 0xF)
#define ISEG_CLEAR      ISEG_ARGB4444(0x0, 0x0, 0x0, 0x0)

#define ISEG_BLACK_BG   ISEG_ARGB4444(0xA, 0x0, 0x0, 0x0)
#define ISEG_LABEL_PAD  3

#define ISEG_HUD_BG     ISEG_ARGB4444(0x8, 0x0, 0x0, 0x0)
#define ISEG_HUD_PAD_X  4
#define ISEG_HUD_PAD_Y  3

#define ISEG_NUM_COLORS 8u

static const uint16_t s_class_colors[ISEG_NUM_COLORS] = {
    ISEG_ARGB4444(ISEG_ALPHA_HALF, 0xF, 0x0, 0x0), /* red     */
    ISEG_ARGB4444(ISEG_ALPHA_HALF, 0x0, 0xF, 0x0), /* green   */
    ISEG_ARGB4444(ISEG_ALPHA_HALF, 0x0, 0x4, 0xF), /* blue    */
    ISEG_ARGB4444(ISEG_ALPHA_HALF, 0xF, 0xF, 0x0), /* yellow  */
    ISEG_ARGB4444(ISEG_ALPHA_HALF, 0xF, 0x0, 0xF), /* magenta */
    ISEG_ARGB4444(ISEG_ALPHA_HALF, 0x0, 0xF, 0xF), /* cyan    */
    ISEG_ARGB4444(ISEG_ALPHA_HALF, 0xF, 0x8, 0x0), /* orange  */
    ISEG_ARGB4444(ISEG_ALPHA_HALF, 0x8, 0x0, 0xF), /* purple  */
};

static void iseg_fill_rect(uint16_t *buf,
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
        {
            row[x] = color;
        }
    }
}

/* Two-pixel white outline. x1 and y1 are exclusive. */
static void iseg_draw_bbox(int x0, int y0, int x1, int y1)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > (int)LCD_OVL_W) x1 = (int)LCD_OVL_W;
    if (y1 > (int)LCD_OVL_H) y1 = (int)LCD_OVL_H;

    int w = x1 - x0;
    int h = y1 - y0;
    if (w <= 0 || h <= 0) return;

    ohm_lcd_rect_white((uint32_t)x0, (uint32_t)y0,
                       (uint32_t)w,  (uint32_t)h);
    if (w > 2 && h > 2)
    {
        ohm_lcd_rect_white((uint32_t)(x0 + 1), (uint32_t)(y0 + 1),
                           (uint32_t)(w - 2),  (uint32_t)(h - 2));
    }
}

static char s_banner[40] = "";

void LCD_Overlay_SetBanner(const char *text)
{
    if (text == NULL)
    {
        s_banner[0] = '\0';
        return;
    }
    strncpy(s_banner, text, sizeof(s_banner) - 1u);
    s_banner[sizeof(s_banner) - 1u] = '\0';
}

void LCD_Overlay_Iseg(OVL_Mode_t mode)
{
    uint16_t *buf = OVL_GetBackBuffer();
    if (buf == NULL) return;

    ohm_lcd_bind_argb4444(buf, LCD_OVL_W, LCD_OVL_H, LCD_OVL_W);

    if (mode == OVL_RELOAD_CLEAR || mode == OVL_CLEAR)
    {
        uint32_t n_pixels = LCD_OVL_W * LCD_OVL_H;
        for (uint32_t i = 0; i < n_pixels; i++)
        {
            buf[i] = ISEG_CLEAR;
        }
    }

    {
        char inf_str[32];
        uint32_t inf_ms = ts[1] - ts[0];
        snprintf(inf_str, sizeof(inf_str), "inference: %lums", (unsigned long)inf_ms);

        int tx = 8;
        int ty = 8;
        int len = (int)strlen(inf_str);
        int box_w = (len * (int)Font16.Width)  + (2 * ISEG_HUD_PAD_X);
        int box_h = ((int)Font16.Height)       + (2 * ISEG_HUD_PAD_Y);

        if (tx + box_w > (int)LCD_OVL_W) box_w = (int)LCD_OVL_W - tx;
        if (ty + box_h > (int)LCD_OVL_H) box_h = (int)LCD_OVL_H - ty;

        if (box_w > 0 && box_h > 0)
            iseg_fill_rect(buf, tx, ty, tx + box_w, ty + box_h, ISEG_HUD_BG);

        ohm_lcd_text((uint32_t)(tx + ISEG_HUD_PAD_X),
                     (uint32_t)(ty + ISEG_HUD_PAD_Y),
                     inf_str);
    }

    int n_det = pp_output.nb_detect;

    for (int d = 0; d < n_det; d++)
    {
        const iseg_pp_outBuffer_t *det = &pp_output.pOutBuff[d];
        if (det == NULL) continue;

        uint16_t mask_color = s_class_colors[
            (uint32_t)det->class_index % ISEG_NUM_COLORS];

        /* pMask: 64x64 uint8, full frame, no useful signal outside the box. */
        const uint32_t M = AI_YOLOV8_SEG_PP_MASK_SIZE;

        int m_x0 = (int)((det->x_center - det->width  * 0.5f) * (float)M);
        int m_x1 = (int)((det->x_center + det->width  * 0.5f) * (float)M);
        int m_y0 = (int)((det->y_center - det->height * 0.5f) * (float)M);
        int m_y1 = (int)((det->y_center + det->height * 0.5f) * (float)M);
        if (m_x0 < 0)      m_x0 = 0;
        if (m_y0 < 0)      m_y0 = 0;
        if (m_x1 > (int)M) m_x1 = (int)M;
        if (m_y1 > (int)M) m_y1 = (int)M;

        const uint8_t *mask = det->pMask;

        for (uint32_t mr = 0; mr < M; mr++)
        {
            if ((int)mr < m_y0 || (int)mr >= m_y1) continue;
            for (uint32_t mc = 0; mc < M; mc++)
            {
                if ((int)mc < m_x0 || (int)mc >= m_x1) continue;
                if (mask[mr * M + mc] == 0u) continue;

                int px0 = nn_norm_to_lcd_x((float)mc          / (float)M);
                int py0 = nn_norm_to_lcd_y((float)mr          / (float)M);
                int px1 = nn_norm_to_lcd_x((float)(mc + 1u)   / (float)M);
                int py1 = nn_norm_to_lcd_y((float)(mr + 1u)   / (float)M);
                if (px1 <= px0) px1 = px0 + 1;
                if (py1 <= py0) py1 = py0 + 1;

                iseg_fill_rect(buf, px0, py0, px1, py1, mask_color);
            }
        }

        int bx0 = nn_norm_to_lcd_x(det->x_center - det->width  * 0.5f);
        int by0 = nn_norm_to_lcd_y(det->y_center - det->height * 0.5f);
        int bx1 = nn_norm_to_lcd_x(det->x_center + det->width  * 0.5f);
        int by1 = nn_norm_to_lcd_y(det->y_center + det->height * 0.5f);

        iseg_draw_bbox(bx0, by0, bx1, by1);

        char label[48];
        const char *class_name = "?";
        if (det->class_index >= 0 && det->class_index < NB_CLASSES)
        {
            class_name = classes_table[det->class_index];
        }
        int conf_pct = (int)(det->conf * 100.0f + 0.5f);
        snprintf(label, sizeof(label), "%s %d%%", class_name, conf_pct);

        int label_text_w = (int)(strlen(label) * Font16.Width);
        int label_text_h = (int)Font16.Height;

        int label_bg_h = label_text_h + 2 * ISEG_LABEL_PAD;
        int lbg_y0     = by0 - label_bg_h;
        if (lbg_y0 < 0) lbg_y0 = 0;
        int lbg_x0     = bx0;
        if (lbg_x0 < 0) lbg_x0 = 0;
        int lbg_x1     = lbg_x0 + label_text_w + 2 * ISEG_LABEL_PAD;

        iseg_fill_rect(buf, lbg_x0, lbg_y0,
                       lbg_x1, lbg_y0 + label_bg_h,
                       ISEG_BLACK_BG);

        ohm_lcd_text((uint32_t)(lbg_x0 + ISEG_LABEL_PAD),
                     (uint32_t)(lbg_y0 + ISEG_LABEL_PAD),
                     label);
    }

    if (s_banner[0] != '\0')
    {
        int len   = (int)strlen(s_banner);
        int box_w = (len * (int)Font16.Width) + (2 * ISEG_HUD_PAD_X);
        int box_h = ((int)Font16.Height)      + (2 * ISEG_HUD_PAD_Y);
        int tx    = ((int)LCD_OVL_W - box_w) / 2;
        int ty    = 8;

        if (tx < 0) tx = 0;
        if (tx + box_w > (int)LCD_OVL_W) box_w = (int)LCD_OVL_W - tx;

        if (box_w > 0 && box_h > 0)
        {
            iseg_fill_rect(buf, tx, ty, tx + box_w, ty + box_h, ISEG_HUD_BG);
            ohm_lcd_text((uint32_t)(tx + ISEG_HUD_PAD_X),
                         (uint32_t)(ty + ISEG_HUD_PAD_Y),
                         s_banner);
        }
    }

    if (mode == OVL_RELOAD_CLEAR || mode == OVL_RELOAD)
    {
        OVL_Commit();
    }
}
