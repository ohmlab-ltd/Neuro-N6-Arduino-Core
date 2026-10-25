/*
 * ohmlab_lcd_draw.c
 *
 * ARGB4444 overlay drawing primitives.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "ohmlab_lcd_draw.h"
#include <stddef.h>
#include <string.h>

typedef struct
{
  uint16_t *base;
  uint32_t  width;
  uint32_t  height;
  uint32_t  stride;     /* pixels per row */
} ohm_lcd_ctx_t;

static ohm_lcd_ctx_t g_ctx = {0};

static const sFONT *g_font = &Font16;

#ifndef OHM_ARGB4444
#define OHM_ARGB4444(a,r,g,b) ( (uint16_t)((((a)&0xFu)<<12) | (((r)&0xFu)<<8) | (((g)&0xFu)<<4) | (((b)&0xFu)<<0)) )
#endif

#define OHM_A_TRANSPARENT (0x0u)
#define OHM_A_OPAQUE      (0xFu)

#define OHM_WHITE_ARGB4444 OHM_ARGB4444(OHM_A_OPAQUE, 0xF, 0xF, 0xF)
#define OHM_CLEAR_ARGB4444 OHM_ARGB4444(OHM_A_TRANSPARENT, 0x0, 0x0, 0x0)

static inline uint32_t ohm_min_u32(uint32_t a, uint32_t b) { return (a < b) ? a : b; }

static inline int ohm_in_bounds(uint32_t x, uint32_t y)
{
  return (g_ctx.base != NULL) && (x < g_ctx.width) && (y < g_ctx.height);
}

static inline void ohm_set_px(uint32_t x, uint32_t y, uint16_t argb4444)
{
  if (!ohm_in_bounds(x, y)) return;
  g_ctx.base[y * g_ctx.stride + x] = argb4444;
}

void ohm_lcd_bind_argb4444(uint16_t *base,
                           uint32_t width,
                           uint32_t height,
                           uint32_t stride_pixels)
{
  g_ctx.base   = base;
  g_ctx.width  = width;
  g_ctx.height = height;
  g_ctx.stride = stride_pixels;
}

static void ohm_clear_to(uint16_t c)
{
  if (g_ctx.base == NULL || g_ctx.width == 0 || g_ctx.height == 0 || g_ctx.stride == 0)
    return;

  for (uint32_t y = 0; y < g_ctx.height; y++)
  {
    uint16_t *row = g_ctx.base + y * g_ctx.stride;
    for (uint32_t x = 0; x < g_ctx.width; x++)
    {
      row[x] = c;
    }
  }
}

void ohm_lcd_clear_transparent(void) { ohm_clear_to(OHM_CLEAR_ARGB4444); }
void ohm_lcd_clear_white_opaque(void) { ohm_clear_to(OHM_WHITE_ARGB4444); }

/* Current colour. The ISEG, MPE, PD and Nia overlays depend on the default. */
static uint16_t g_color = OHM_WHITE_ARGB4444;

void ohm_lcd_set_color(uint8_t r, uint8_t g, uint8_t b)
{
  g_color = OHM_ARGB4444(OHM_A_OPAQUE, (r >> 4), (g >> 4), (b >> 4));
}

void ohm_lcd_set_color_alpha(uint8_t a, uint8_t r, uint8_t g, uint8_t b)
{
  g_color = OHM_ARGB4444((a >> 4), (r >> 4), (g >> 4), (b >> 4));
}

void ohm_lcd_reset_color(void) { g_color = OHM_WHITE_ARGB4444; }

uint16_t ohm_lcd_get_color(void) { return g_color; }
void ohm_lcd_set_color_packed(uint16_t argb4444) { g_color = argb4444; }

/* Primitives */

static void hline_c(uint32_t x, uint32_t y, uint32_t len, uint16_t c)
{
  if (g_ctx.base == NULL) return;
  if (y >= g_ctx.height) return;
  if (x >= g_ctx.width) return;
  if (len == 0) return;

  uint32_t end = ohm_min_u32(x + len, g_ctx.width);

  uint16_t *row = g_ctx.base + y * g_ctx.stride;
  for (uint32_t xx = x; xx < end; xx++)
  {
    row[xx] = c;
  }
}

static void vline_c(uint32_t x, uint32_t y, uint32_t len, uint16_t c)
{
  if (g_ctx.base == NULL) return;
  if (x >= g_ctx.width) return;
  if (y >= g_ctx.height) return;
  if (len == 0) return;

  uint32_t end = ohm_min_u32(y + len, g_ctx.height);

  for (uint32_t yy = y; yy < end; yy++)
  {
    g_ctx.base[yy * g_ctx.stride + x] = c;
  }
}

static void rect_c(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t c)
{
  if (w == 0 || h == 0) return;

  hline_c(x, y, w, c);
  if (h > 1) hline_c(x, y + h - 1u, w, c);

  if (h > 2)
  {
    vline_c(x, y + 1u, h - 2u, c);
    if (w > 1) vline_c(x + w - 1u, y + 1u, h - 2u, c);
  }
}

static void fillrect_c(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t c)
{
  if (g_ctx.base == NULL) return;
  if (w == 0 || h == 0) return;
  if (x >= g_ctx.width || y >= g_ctx.height) return;

  uint32_t x_end = ohm_min_u32(x + w, g_ctx.width);
  uint32_t y_end = ohm_min_u32(y + h, g_ctx.height);

  for (uint32_t yy = y; yy < y_end; yy++)
  {
    uint16_t *row = g_ctx.base + yy * g_ctx.stride;
    for (uint32_t xx = x; xx < x_end; xx++)
    {
      row[xx] = c;
    }
  }
}

void ohm_lcd_putpixel_white(uint32_t x, uint32_t y) { ohm_set_px(x, y, OHM_WHITE_ARGB4444); }
void ohm_lcd_hline_white(uint32_t x, uint32_t y, uint32_t len)            { hline_c(x, y, len, OHM_WHITE_ARGB4444); }
void ohm_lcd_vline_white(uint32_t x, uint32_t y, uint32_t len)            { vline_c(x, y, len, OHM_WHITE_ARGB4444); }
void ohm_lcd_rect_white(uint32_t x, uint32_t y, uint32_t w, uint32_t h)   { rect_c(x, y, w, h, OHM_WHITE_ARGB4444); }
void ohm_lcd_fillrect_white(uint32_t x, uint32_t y, uint32_t w, uint32_t h) { fillrect_c(x, y, w, h, OHM_WHITE_ARGB4444); }

void ohm_lcd_putpixel(uint32_t x, uint32_t y) { ohm_set_px(x, y, g_color); }
void ohm_lcd_hline(uint32_t x, uint32_t y, uint32_t len)            { hline_c(x, y, len, g_color); }
void ohm_lcd_vline(uint32_t x, uint32_t y, uint32_t len)            { vline_c(x, y, len, g_color); }
void ohm_lcd_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h)   { rect_c(x, y, w, h, g_color); }
void ohm_lcd_fillrect(uint32_t x, uint32_t y, uint32_t w, uint32_t h) { fillrect_c(x, y, w, h, g_color); }

void ohm_lcd_rect_thick(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t t)
{
  if (t == 0) t = 1;
  for (uint32_t i = 0; i < t; i++)
  {
    if (w <= 2u * i || h <= 2u * i) break;
    rect_c(x + i, y + i, w - 2u * i, h - 2u * i, g_color);
  }
}

/* Text */

void ohm_lcd_set_font(const sFONT *font)
{
  if (font != NULL) g_font = font;
}

static inline const sFONT *ohm_font(void)
{
  return (g_font != NULL) ? g_font : &Font16;
}

/* The font table is 1 bit per pixel, row-major, most significant bit first. */
static void ohm_draw_glyph(uint32_t x, uint32_t y, const uint8_t *glyph_bits)
{
  const sFONT *f = ohm_font();
  const uint32_t w = (uint32_t)f->Width;
  const uint32_t h = (uint32_t)f->Height;

  const uint32_t bytes_per_row = (w + 7u) / 8u;

  for (uint32_t row = 0; row < h; row++)
  {
    const uint8_t *row_bits = glyph_bits + row * bytes_per_row;

    for (uint32_t col = 0; col < w; col++)
    {
      uint32_t byte_i = col / 8u;
      uint32_t bit_i  = 7u - (col % 8u);
      uint8_t  bit    = (row_bits[byte_i] >> bit_i) & 1u;

      if (bit)
      {
        ohm_set_px(x + col, y + row, g_color);
      }
    }
  }
}

void ohm_lcd_char(uint32_t x, uint32_t y, uint8_t ascii)
{
  const sFONT *f = ohm_font();

  if (ascii < 0x20 || ascii > 0x7Eu) return;

  uint32_t w = (uint32_t)f->Width;
  uint32_t h = (uint32_t)f->Height;
  uint32_t bytes_per_row = (w + 7u) / 8u;
  uint32_t glyph_size = h * bytes_per_row;

  const uint8_t *glyph = &f->table[(ascii - 0x20u) * glyph_size];
  ohm_draw_glyph(x, y, glyph);
}

void ohm_lcd_text(uint32_t x, uint32_t y, const char *s)
{
  if (s == NULL) return;

  const sFONT *f = ohm_font();
  uint32_t cursor_x = x;

  while (*s != '\0')
  {
    char c = *s++;

    if (c == '\n')
    {
      cursor_x = x;
      y += (uint32_t)f->Height;
      continue;
    }

    ohm_lcd_char(cursor_x, y, (uint8_t)c);
    cursor_x += (uint32_t)f->Width;
  }
}
