/**
 ******************************************************************************
 * @file    zoo_pp_lcd.c
 * @brief   LCD overlay for the model zoo post-processors.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "zoo_pp.h"
#include "NeuroN6.h"
#include "ohmlab_lcd_draw.h"

#include <stdio.h>
#include <string.h>

/* Inference timestamps in ms: ts[0] start, ts[1] end. */
extern uint32_t ts[];

#define ZOO_LCD_W   800
#define ZOO_LCD_H   480

#define ZOO_ARGB4444(a, r, g, b) \
  ((uint16_t)(((a) & 0xFu) << 12 | ((r) & 0xFu) << 8 | ((g) & 0xFu) << 4 | ((b) & 0xFu)))

#define ZOO_CLEAR     ZOO_ARGB4444(0x0, 0x0, 0x0, 0x0)
#define ZOO_LABEL_BG  ZOO_ARGB4444(0xA, 0x0, 0x0, 0x0)
#define ZOO_HUD_BG    ZOO_ARGB4444(0x8, 0x0, 0x0, 0x0)
#define ZOO_PAD       3
#define ZOO_BOX_THICK 2
#define ZOO_KP_RADIUS 2

/* Class colours. A translucent tint is (colour & 0x0FFF) | alpha << 12. */
#define ZOO_NUM_COLORS 8u
static const uint16_t s_colors[ZOO_NUM_COLORS] = {
  ZOO_ARGB4444(0xF, 0xF, 0x0, 0x0),   /* red     */
  ZOO_ARGB4444(0xF, 0x0, 0xF, 0x0),   /* green   */
  ZOO_ARGB4444(0xF, 0x2, 0x8, 0xF),   /* blue    */
  ZOO_ARGB4444(0xF, 0xF, 0xF, 0x0),   /* yellow  */
  ZOO_ARGB4444(0xF, 0xF, 0x0, 0xF),   /* magenta */
  ZOO_ARGB4444(0xF, 0x0, 0xF, 0xF),   /* cyan    */
  ZOO_ARGB4444(0xF, 0xF, 0x8, 0x0),   /* orange  */
  ZOO_ARGB4444(0xF, 0xB, 0x6, 0xF),   /* violet  */
};

/* Body-part colours of the COCO skeleton. */
static const uint16_t s_part_colors[4] = {
  ZOO_ARGB4444(0xF, 0x0, 0xF, 0x0),   /* head:  green   */
  ZOO_ARGB4444(0xF, 0x0, 0x0, 0xF),   /* arms:  blue    */
  ZOO_ARGB4444(0xF, 0xF, 0x0, 0xF),   /* trunk: magenta */
  ZOO_ARGB4444(0xF, 0xF, 0x8, 0x0),   /* legs:  orange  */
};

static inline uint16_t class_color(int32_t cls)
{
  return s_colors[(uint32_t)cls % ZOO_NUM_COLORS];
}

/* Drawing helpers, clipped to the panel */

static void fill_rect(uint16_t *buf, int x0, int y0, int x1, int y1, uint16_t c)
{
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 > ZOO_LCD_W) x1 = ZOO_LCD_W;
  if (y1 > ZOO_LCD_H) y1 = ZOO_LCD_H;
  for (int y = y0; y < y1; y++) {
    uint16_t *row = buf + y * ZOO_LCD_W;
    for (int x = x0; x < x1; x++) row[x] = c;
  }
}

static void outline(uint16_t *buf, int x0, int y0, int x1, int y1, uint16_t c)
{
  if (x1 <= x0 || y1 <= y0) return;
  const int t = ZOO_BOX_THICK;
  fill_rect(buf, x0, y0, x1, y0 + t, c);
  fill_rect(buf, x0, y1 - t, x1, y1, c);
  fill_rect(buf, x0, y0, x0 + t, y1, c);
  fill_rect(buf, x1 - t, y0, x1, y1, c);
}

static void line(uint16_t *buf, int x0, int y0, int x1, int y1, uint16_t c)
{
  const int dx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
  const int dy = -((y1 > y0) ? (y1 - y0) : (y0 - y1));
  const int sx = (x0 < x1) ? 1 : -1;
  const int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    if (x0 >= 0 && x0 < ZOO_LCD_W && y0 >= 0 && y0 < ZOO_LCD_H) buf[y0 * ZOO_LCD_W + x0] = c;
    if (x0 == x1 && y0 == y1) break;
    const int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

/* Three pixels wide. */
static void thick_line(uint16_t *buf, int x0, int y0, int x1, int y1, uint16_t c)
{
  const int adx = (x1 > x0) ? (x1 - x0) : (x0 - x1);
  const int ady = (y1 > y0) ? (y1 - y0) : (y0 - y1);
  line(buf, x0, y0, x1, y1, c);
  if (ady > adx) {
    line(buf, x0 + 1, y0, x1 + 1, y1, c);
    line(buf, x0 - 1, y0, x1 - 1, y1, c);
  } else {
    line(buf, x0, y0 + 1, x1, y1 + 1, c);
    line(buf, x0, y0 - 1, x1, y1 - 1, c);
  }
}

/* Text in a box whose top-left corner is (x, y). */
static void text_box(uint16_t *buf, int x, int y, const char *s, uint16_t bg)
{
  const int w = (int)strlen(s) * (int)Font16.Width + 2 * ZOO_PAD;
  const int h = (int)Font16.Height + 2 * ZOO_PAD;
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x + w > ZOO_LCD_W) x = (ZOO_LCD_W - w > 0) ? (ZOO_LCD_W - w) : 0;
  if (y + h > ZOO_LCD_H) y = ZOO_LCD_H - h;
  fill_rect(buf, x, y, x + w, y + h, bg);
  ohm_lcd_text((uint32_t)(x + ZOO_PAD), (uint32_t)(y + ZOO_PAD), s);
}

/* Label on top of a box whose top-left corner is (x, y). */
static void box_label(uint16_t *buf, int x, int y, const char *s)
{
  const int h = (int)Font16.Height + 2 * ZOO_PAD;
  text_box(buf, x, (y - h >= 0) ? (y - h) : y, s, ZOO_LABEL_BG);
}

static int clip_to(int v, int lo, int hi)
{
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* The box on the panel, clipped to the camera picture: a letterboxed input
 * reaches past it. */
static void box_to_lcd(const od_pp_outBuffer_t *d, int *x0, int *y0, int *x1, int *y1)
{
  const int l = g_lcd_camera_x, r = g_lcd_camera_x + (int)g_lcd_camera_w - 1;
  const int t = g_lcd_camera_y, b = g_lcd_camera_y + (int)g_lcd_camera_h - 1;
  *x0 = clip_to(nn_norm_to_lcd_x(d->x_center - 0.5f * d->width),  l, r);
  *y0 = clip_to(nn_norm_to_lcd_y(d->y_center - 0.5f * d->height), t, b);
  *x1 = clip_to(nn_norm_to_lcd_x(d->x_center + 0.5f * d->width),  l, r);
  *y1 = clip_to(nn_norm_to_lcd_y(d->y_center + 0.5f * d->height), t, b);
}

/* Height of the inference-time line at the top left. */
static int hud_height(void)
{
  return (int)Font16.Height + 2 * ZOO_PAD;
}

/* Returns the back buffer, or NULL when there is no overlay. */
static uint16_t *lcd_begin(OVL_Mode_t mode)
{
  uint16_t *buf = OVL_GetBackBuffer();
  if (buf == NULL) return NULL;
  ohm_lcd_bind_argb4444(buf, ZOO_LCD_W, ZOO_LCD_H, ZOO_LCD_W);
  ohm_lcd_set_font(&Font16);
  if (mode == OVL_RELOAD_CLEAR || mode == OVL_CLEAR) {
    for (uint32_t i = 0; i < (uint32_t)(ZOO_LCD_W * ZOO_LCD_H); i++) buf[i] = ZOO_CLEAR;
  }
  char s[32];
  snprintf(s, sizeof(s), "inference: %lums", (unsigned long)(ts[1] - ts[0]));
  text_box(buf, 8, 8, s, ZOO_HUD_BG);
  return buf;
}

static void lcd_end(OVL_Mode_t mode)
{
  if (mode == OVL_RELOAD_CLEAR || mode == OVL_RELOAD) OVL_Commit();
}

void zoo_lcd_detections(const od_pp_outBuffer_t *d, int32_t n,
                        const char *const *labels, int32_t nlabels,
                        int32_t nb_classes, OVL_Mode_t mode)
{
  uint16_t *buf = lcd_begin(mode);
  if (buf == NULL) return;
  for (int32_t i = 0; i < n && d != NULL; i++) {
    int x0, y0, x1, y1;
    box_to_lcd(&d[i], &x0, &y0, &x1, &y1);
    outline(buf, x0, y0, x1, y1, class_color(d[i].class_index));
    char s[64];
    snprintf(s, sizeof(s), "%s %d%%",
             zoo_label(labels, nlabels, nb_classes, d[i].class_index),
             (int)(d[i].conf * 100.0f + 0.5f));
    box_label(buf, x0, y0, s);
  }
  lcd_end(mode);
}

void zoo_lcd_coco_pose(const od_pp_outBuffer_t *boxes, const zoo_kp_t *kps,
                       int32_t n, float32_t min_conf, OVL_Mode_t mode)
{
  uint16_t *buf = lcd_begin(mode);
  if (buf == NULL) return;
  for (int32_t p = 0; p < n && boxes != NULL && kps != NULL; p++) {
    const zoo_kp_t *k = &kps[p * 17];
    for (uint32_t b = 0; b < 19u; b++) {
      const zoo_kp_t *a = &k[zoo_coco_bindings[b][0]];
      const zoo_kp_t *c = &k[zoo_coco_bindings[b][1]];
      if (a->conf < min_conf || c->conf < min_conf) continue;
      thick_line(buf, nn_norm_to_lcd_x(a->x), nn_norm_to_lcd_y(a->y),
                 nn_norm_to_lcd_x(c->x), nn_norm_to_lcd_y(c->y),
                 s_part_colors[zoo_coco_bindings[b][2] & 3u]);
    }
    for (uint32_t i = 0; i < 17u; i++) {
      if (k[i].conf < min_conf) continue;
      const int x = nn_norm_to_lcd_x(k[i].x);
      const int y = nn_norm_to_lcd_y(k[i].y);
      fill_rect(buf, x - ZOO_KP_RADIUS, y - ZOO_KP_RADIUS,
                x + ZOO_KP_RADIUS + 1, y + ZOO_KP_RADIUS + 1,
                s_part_colors[zoo_coco_kp_color[i] & 3u]);
    }
    int x0, y0, x1, y1;
    box_to_lcd(&boxes[p], &x0, &y0, &x1, &y1);
    char s[32];
    snprintf(s, sizeof(s), "person %d%%", (int)(boxes[p].conf * 100.0f + 0.5f));
    box_label(buf, x0, y0, s);
  }
  lcd_end(mode);
}

/* LCD pixel to label-map column or row, -1 outside the network's picture. */
static int16_t s_col[ZOO_LCD_W];
static int16_t s_row[ZOO_LCD_H];

static void build_axis(int16_t *lut, int n, int16_t cam_off, uint16_t cam_len,
                       uint16_t full, int16_t crop_off, uint16_t crop_len,
                       uint32_t map_len)
{
  for (int i = 0; i < n; i++) {
    lut[i] = -1;
    if (cam_len == 0u || crop_len == 0u) continue;
    const float32_t cn = ((float32_t)i + 0.5f - (float32_t)cam_off) / (float32_t)cam_len;
    if (cn < 0.f || cn >= 1.f) continue;
    const float32_t nn = (cn * (float32_t)full - (float32_t)crop_off) / (float32_t)crop_len;
    if (nn < 0.f || nn >= 1.f) continue;
    lut[i] = (int16_t)(nn * (float32_t)map_len);
  }
}

void zoo_lcd_regions(const zoo_region_t *r, int32_t n, const uint8_t *map,
                     uint32_t w, uint32_t h, const char *const *labels,
                     int32_t nlabels, int32_t nb_classes, OVL_Mode_t mode)
{
  uint16_t *buf = lcd_begin(mode);
  if (buf == NULL) return;

  if (map != NULL && w > 0u && h > 0u) {
    build_axis(s_col, ZOO_LCD_W, g_lcd_camera_x, g_lcd_camera_w, g_camera_full_w,
               g_nn_crop_x, g_nn_crop_w, w);
    build_axis(s_row, ZOO_LCD_H, g_lcd_camera_y, g_lcd_camera_h, g_camera_full_h,
               g_nn_crop_y, g_nn_crop_h, h);
    /* Starts below the inference-time line. */
    for (int y = hud_height() + 8; y < ZOO_LCD_H; y++) {
      if (s_row[y] < 0) continue;
      const uint8_t *mrow = &map[(uint32_t)s_row[y] * w];
      uint16_t *out = &buf[y * ZOO_LCD_W];
      for (int x = 0; x < ZOO_LCD_W; x++) {
        if (s_col[x] < 0) continue;
        const uint8_t cls = mrow[s_col[x]];
        if (cls != 0u) out[x] = (uint16_t)((class_color(cls) & 0x0FFFu) | 0x6000u);
      }
    }
  }

  for (int32_t i = 0; i < n && r != NULL; i++) {
    int x0, y0, x1, y1;
    box_to_lcd(&r[i].box, &x0, &y0, &x1, &y1);
    char s[64];
    snprintf(s, sizeof(s), "%s %d%%",
             zoo_label(labels, nlabels, nb_classes, r[i].box.class_index),
             (int)(r[i].box.conf * 100.0f + 0.5f));
    box_label(buf, x0, y0, s);
  }
  lcd_end(mode);
}

void zoo_lcd_text_lines(const char *const *text, int32_t lines, OVL_Mode_t mode)
{
  uint16_t *buf = lcd_begin(mode);
  if (buf == NULL) return;
  int y = 8 + hud_height() + 4;
  for (int32_t i = 0; i < lines && text != NULL; i++) {
    if (text[i] == NULL) continue;
    text_box(buf, 8, y, text[i], ZOO_HUD_BG);
    y += hud_height();
  }
  lcd_end(mode);
}
