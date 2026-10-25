/*
 * ohmlab_lcd_draw.h
 *
 * ARGB4444 overlay drawing primitives.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef OHMLAB_LCD_DRAW_H
#define OHMLAB_LCD_DRAW_H

#include <stdint.h>
#include "Fonts/fonts.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Binds the buffer the primitives draw into. stride_pixels is pixels per row. */
void ohm_lcd_bind_argb4444(uint16_t *base,
                           uint32_t width,
                           uint32_t height,
                           uint32_t stride_pixels);

void ohm_lcd_clear_transparent(void);
void ohm_lcd_clear_white_opaque(void);

void ohm_lcd_putpixel_white(uint32_t x, uint32_t y);
void ohm_lcd_hline_white(uint32_t x, uint32_t y, uint32_t len);
void ohm_lcd_vline_white(uint32_t x, uint32_t y, uint32_t len);
void ohm_lcd_rect_white(uint32_t x, uint32_t y, uint32_t w, uint32_t h);
void ohm_lcd_fillrect_white(uint32_t x, uint32_t y, uint32_t w, uint32_t h);

/* Colour of the unsuffixed primitives. Channels are 0..255, kept to 4 bits. */
void ohm_lcd_set_color(uint8_t r, uint8_t g, uint8_t b);
void ohm_lcd_set_color_alpha(uint8_t a, uint8_t r, uint8_t g, uint8_t b);
void ohm_lcd_reset_color(void);  /* opaque white */

uint16_t ohm_lcd_get_color(void);
void     ohm_lcd_set_color_packed(uint16_t argb4444);

void ohm_lcd_putpixel(uint32_t x, uint32_t y);
void ohm_lcd_hline(uint32_t x, uint32_t y, uint32_t len);
void ohm_lcd_vline(uint32_t x, uint32_t y, uint32_t len);
void ohm_lcd_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h);
void ohm_lcd_fillrect(uint32_t x, uint32_t y, uint32_t w, uint32_t h);

/* Border t px thick, drawn inward from the box. */
void ohm_lcd_rect_thick(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t t);

/* Text in the current colour. The background is not drawn. */
void ohm_lcd_set_font(const sFONT *font);
void ohm_lcd_char(uint32_t x, uint32_t y, uint8_t ascii);
void ohm_lcd_text(uint32_t x, uint32_t y, const char *s);

#ifdef __cplusplus
}
#endif

#endif
