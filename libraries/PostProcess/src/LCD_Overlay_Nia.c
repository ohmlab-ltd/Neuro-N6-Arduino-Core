/*
 * LCD_Overlay_Nia.c
 *
 * LCD overlay for the Nia detector.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "LCD_Overlay_Nia.h"
#include "NeuroN6.h"
#include "app_postprocess_nia_ui.h"
#include "ohmlab_lcd_draw.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Inference start and end ticks, ms. */
extern uint32_t ts[2];

#ifndef NB_CLASSES
#define NB_CLASSES (80)
#endif
extern const char *classes_table[NB_CLASSES];

#define LCD_OVL_W   800u
#define LCD_OVL_H   480u

#define NIA_ARGB4444(a, r, g, b) \
    ((uint16_t)(((a) & 0xFu) << 12 | ((r) & 0xFu) << 8 | \
                ((g) & 0xFu) <<  4 | ((b) & 0xFu)))

#define NIA_CLEAR     NIA_ARGB4444(0x0, 0x0, 0x0, 0x0)
#define NIA_BLACK_BG  NIA_ARGB4444(0xA, 0x0, 0x0, 0x0)
#define NIA_HUD_BG    NIA_ARGB4444(0x8, 0x0, 0x0, 0x0)
#define NIA_LABEL_PAD 3
#define NIA_HUD_PAD_X 4
#define NIA_HUD_PAD_Y 3
#define NIA_BOX_THICK 2

#define NIA_NUM_COLORS 8u
static const uint16_t s_class_colors[NIA_NUM_COLORS] = {
    NIA_ARGB4444(0xF, 0xF, 0x0, 0x0), /* red     */
    NIA_ARGB4444(0xF, 0x0, 0xF, 0x0), /* green   */
    NIA_ARGB4444(0xF, 0x2, 0x8, 0xF), /* blue    */
    NIA_ARGB4444(0xF, 0xF, 0xF, 0x0), /* yellow  */
    NIA_ARGB4444(0xF, 0xF, 0x0, 0xF), /* magenta */
    NIA_ARGB4444(0xF, 0x0, 0xF, 0xF), /* cyan    */
    NIA_ARGB4444(0xF, 0xF, 0x8, 0x0), /* orange  */
    NIA_ARGB4444(0xF, 0xB, 0x6, 0xF), /* violet  */
};

static void nia_fill_rect(uint16_t *buf, int x0, int y0, int x1, int y1, uint16_t color)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > (int)LCD_OVL_W) x1 = (int)LCD_OVL_W;
    if (y1 > (int)LCD_OVL_H) y1 = (int)LCD_OVL_H;
    for (int y = y0; y < y1; y++) {
        uint16_t *row = buf + (uint32_t)y * LCD_OVL_W;
        for (int x = x0; x < x1; x++) row[x] = color;
    }
}

/* x1 and y1 are exclusive. */
static void nia_box_outline(uint16_t *buf, int x0, int y0, int x1, int y1, uint16_t color)
{
    if (x1 <= x0 || y1 <= y0) return;
    const int t = NIA_BOX_THICK;
    nia_fill_rect(buf, x0, y0,        x1,     y0 + t, color);  /* top    */
    nia_fill_rect(buf, x0, y1 - t,    x1,     y1,     color);  /* bottom */
    nia_fill_rect(buf, x0, y0,        x0 + t, y1,     color);  /* left   */
    nia_fill_rect(buf, x1 - t, y0,    x1,     y1,     color);  /* right  */
}

void LCD_Overlay_Nia_From(const nia_pp_out_t *out, OVL_Mode_t mode)
{
    uint16_t *buf = OVL_GetBackBuffer();
    if (buf == NULL) return;

    ohm_lcd_bind_argb4444(buf, LCD_OVL_W, LCD_OVL_H, LCD_OVL_W);

    if (mode == OVL_RELOAD_CLEAR || mode == OVL_CLEAR) {
        const uint32_t n = LCD_OVL_W * LCD_OVL_H;
        for (uint32_t i = 0; i < n; i++) buf[i] = NIA_CLEAR;
    }

    {
        char s[32];
        uint32_t inf_ms = ts[1] - ts[0];
        snprintf(s, sizeof(s), "inference: %lums", (unsigned long)inf_ms);
        int tx = 8, ty = 8;
        int bw = (int)strlen(s) * (int)Font16.Width + 2 * NIA_HUD_PAD_X;
        int bh = (int)Font16.Height + 2 * NIA_HUD_PAD_Y;
        if (tx + bw > (int)LCD_OVL_W) bw = (int)LCD_OVL_W - tx;
        if (ty + bh > (int)LCD_OVL_H) bh = (int)LCD_OVL_H - ty;
        if (bw > 0 && bh > 0) nia_fill_rect(buf, tx, ty, tx + bw, ty + bh, NIA_HUD_BG);
        ohm_lcd_text((uint32_t)(tx + NIA_HUD_PAD_X), (uint32_t)(ty + NIA_HUD_PAD_Y), s);
    }

    const int n_det = (out && out->pOutBuff) ? out->nb_detect : 0;
    for (int d = 0; d < n_det; d++) {
        const nia_det_t *det = &out->pOutBuff[d];

        const uint16_t color = s_class_colors[(uint32_t)det->class_index % NIA_NUM_COLORS];

        int bx0 = nn_norm_to_lcd_x(det->x_center - det->width  * 0.5f);
        int by0 = nn_norm_to_lcd_y(det->y_center - det->height * 0.5f);
        int bx1 = nn_norm_to_lcd_x(det->x_center + det->width  * 0.5f);
        int by1 = nn_norm_to_lcd_y(det->y_center + det->height * 0.5f);
        if (bx0 < 0) bx0 = 0;
        if (by0 < 0) by0 = 0;
        if (bx1 > (int)LCD_OVL_W) bx1 = (int)LCD_OVL_W;
        if (by1 > (int)LCD_OVL_H) by1 = (int)LCD_OVL_H;

        nia_box_outline(buf, bx0, by0, bx1, by1, color);

        char label[48];
        const char *name = "?";
        if (det->class_index >= 0 && det->class_index < NB_CLASSES)
            name = classes_table[det->class_index];
        int conf_pct = (int)(det->conf * 100.0f + 0.5f);
        snprintf(label, sizeof(label), "%s %d%%", name, conf_pct);

        int tw = (int)strlen(label) * (int)Font16.Width;
        int th = (int)Font16.Height;
        int bg_h = th + 2 * NIA_LABEL_PAD;
        int ly0  = by0 - bg_h; if (ly0 < 0) ly0 = 0;
        int lx0  = bx0;        if (lx0 < 0) lx0 = 0;
        int lx1  = lx0 + tw + 2 * NIA_LABEL_PAD;

        nia_fill_rect(buf, lx0, ly0, lx1, ly0 + bg_h, NIA_BLACK_BG);
        ohm_lcd_text((uint32_t)(lx0 + NIA_LABEL_PAD), (uint32_t)(ly0 + NIA_LABEL_PAD), label);
    }

    if (mode == OVL_RELOAD_CLEAR || mode == OVL_RELOAD) {
        OVL_Commit();
    }
}

void LCD_Overlay_Nia(OVL_Mode_t mode)
{
    LCD_Overlay_Nia_From(&nia_pp_output, mode);
}
