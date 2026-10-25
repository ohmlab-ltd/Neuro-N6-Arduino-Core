/**
 ******************************************************************************
 * AprilTagSynthetic.ino
 *
 * Runs the AprilTag detector on rendered tag36h11 images, with no camera in
 * the path, and prints the detector's statistics for each case. The VERDICT
 * line says whether the ideal tag of case A decoded.
 *
 * Hardware: Neuro N6.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#include <n6_apriltag.h>
#include <n6_tag36h11_codes.h>
#include <string.h>
#include <stdio.h>

#define WIN 160

static uint8_t g_img[WIN * WIN];
static uint8_t g_dark[WIN * WIN];

#define LEVEL_BLACK 16
#define LEVEL_WHITE 240

/* px is the side of the black border square in pixels, a multiple of 8.
 * quiet adds a ring one cell wide, and bg fills everything outside the tag. */
static bool render_tag_rot(uint8_t *dst, int w, int h, int id, int px,
                           bool quiet, bool invert,
                           uint8_t black, uint8_t white, uint8_t bg, int rot90)
{
  const int WB = N6_TAG36H11_WIDTH_AT_BORDER;      /* 8 */
  if (id < 0 || id >= N6_TAG36H11_NCODES) return false;
  if (px <= 0 || (px % WB) != 0) return false;
  const int cell = px / WB;

  memset(dst, bg, (size_t)w * (size_t)h);

  /* 0 is black. bit_x and bit_y are in 1..6, so the border ring stays black. */
  uint8_t tag[8][8];
  memset(tag, 0, sizeof(tag));
  const uint64_t code = n6_tag36h11_codes[id];
  for (int b = 0; b < N6_TAG36H11_NBITS; b++) {
    const int bit = (int)((code >> (N6_TAG36H11_NBITS - 1 - b)) & 1u);
    tag[n6_tag36h11_bit_y[b]][n6_tag36h11_bit_x[b]] = (uint8_t)bit;
  }

  /* Clockwise quarter turns. */
  for (int r = 0; r < (rot90 & 3); r++) {
    uint8_t tmp[8][8];
    for (int y = 0; y < WB; y++)
      for (int x = 0; x < WB; x++) tmp[x][WB - 1 - y] = tag[y][x];
    memcpy(tag, tmp, sizeof(tag));
  }

  const int x0 = (w - px) / 2;
  const int y0 = (h - px) / 2;

  if (quiet) {
    const uint8_t q = invert ? black : white;
    for (int y = y0 - cell; y < y0 + px + cell; y++) {
      if (y < 0 || y >= h) continue;
      for (int x = x0 - cell; x < x0 + px + cell; x++) {
        if (x < 0 || x >= w) continue;
        dst[(size_t)y * (size_t)w + (size_t)x] = q;
      }
    }
  }

  for (int cy = 0; cy < WB; cy++) {
    for (int cx = 0; cx < WB; cx++) {
      const bool is_white = (tag[cy][cx] != 0);
      const uint8_t v = (is_white != invert) ? white : black;
      for (int y = 0; y < cell; y++) {
        const int py = y0 + cy * cell + y;
        if (py < 0 || py >= h) continue;
        for (int x = 0; x < cell; x++) {
          const int qx = x0 + cx * cell + x;
          if (qx < 0 || qx >= w) continue;
          dst[(size_t)py * (size_t)w + (size_t)qx] = v;
        }
      }
    }
  }
  return true;
}

static bool render_tag(uint8_t *dst, int w, int h, int id, int px,
                       bool quiet, bool invert,
                       uint8_t black, uint8_t white, uint8_t bg)
{
  return render_tag_rot(dst, w, h, id, px, quiet, invert, black, white, bg, 0);
}

static void dump_cells(const uint8_t *img, int w, int h, int px)
{
  const int cell = px / N6_TAG36H11_WIDTH_AT_BORDER;
  const int x0 = (w - px) / 2, y0 = (h - px) / 2;
  Serial.println("  cell map (10x10, '#'=dark '.'=light, ring 1 must be all #):");
  for (int cy = -1; cy <= N6_TAG36H11_WIDTH_AT_BORDER; cy++) {
    char row[16];
    int n = 0;
    for (int cx = -1; cx <= N6_TAG36H11_WIDTH_AT_BORDER; cx++) {
      const int sy = y0 + cy * cell + cell / 2;
      const int sx = x0 + cx * cell + cell / 2;
      uint8_t v = 0;
      if (sy >= 0 && sy < h && sx >= 0 && sx < w)
        v = img[(size_t)sy * (size_t)w + (size_t)sx];
      row[n++] = (v < 128) ? '#' : '.';
    }
    row[n] = '\0';
    Serial.print("    ");
    Serial.println(row);
  }
}

static void dump_dark(void)
{
  n6_apriltag_debug_dark(g_dark, WIN, WIN);
  Serial.println("  thresholded 'dark' bitmap, 4x down ('#'=dark seen by fill):");
  for (int y = 0; y < WIN; y += 4) {
    char row[WIN / 4 + 1];
    int n = 0;
    for (int x = 0; x < WIN; x += 4)
      row[n++] = g_dark[(size_t)y * WIN + (size_t)x] ? '#' : '.';
    row[n] = '\0';
    Serial.print("    ");
    Serial.println(row);
  }
}

static int run(const char *name, int expect_id)
{
  n6_apriltag_cfg_t cfg = N6_APRILTAG_CFG_DEFAULT;
  n6_apriltag_t tags[4];
  const int n = n6_apriltag_find_gray(g_img, WIN, WIN, &cfg, tags, 4);

  n6_apriltag_stats_t st;
  n6_apriltag_last_stats(&st);

  char line[160];
  snprintf(line, sizeof(line),
           "  %-22s blobs=%-3ld quads=%-3ld dec=%-2ld | rej c=%ld b=%ld code=%ld "
           "| besth=%ld DROP s=%ld b=%ld",
           name, (long)st.blobs, (long)st.quads, (long)st.decoded,
           (long)st.rej_contrast, (long)st.rej_border, (long)st.rej_code,
           (long)st.best_hamming, (long)st.drop_stack, (long)st.drop_boundary);
  Serial.println(line);

  for (int i = 0; i < n && i < 4; i++) {
    snprintf(line, sizeof(line),
             "      -> id=%ld hamming=%ld margin=%.2f angle=%.1f%s",
             (long)tags[i].id, (long)tags[i].hamming, tags[i].margin,
             tags[i].angle,
             (expect_id >= 0 && tags[i].id == expect_id) ? "   MATCHES EXPECTED"
                                                         : "");
    Serial.println(line);
  }
  if (n <= 0) {
    Serial.println("      -> nothing decoded; grid the decoder SAMPLED:");
    uint8_t c[64];
    n6_apriltag_debug_cells(c);
    for (int y = 0; y < 8; y++) {
      char row[9];
      for (int x = 0; x < 8; x++) row[x] = c[y * 8 + x] ? '.' : '#';
      row[8] = '\0';
      Serial.print("         ");
      Serial.println(row);
    }
  }
  return n;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 4000) { }
}

void loop()
{
  Serial.println();
  Serial.println("=== AprilTag synthetic detection (no camera) ===");
  Serial.print("window ");
  Serial.print(WIN);
  Serial.print("x");
  Serial.print(WIN);
  Serial.print("  codebook ");
  Serial.print((int)N6_TAG36H11_NCODES);
  Serial.println(" ids, tag36h11");
  Serial.println();

  render_tag(g_img, WIN, WIN, 0, 96, true, false,
             LEVEL_BLACK, LEVEL_WHITE, LEVEL_WHITE);
  dump_cells(g_img, WIN, WIN, 96);
  Serial.println();
  const int ok_a = run("A ideal id=0 96px", 0);
  dump_dark();

  render_tag(g_img, WIN, WIN, 17, 96, true, false,
             LEVEL_BLACK, LEVEL_WHITE, LEVEL_WHITE);
  run("B ideal id=17 96px", 17);

  render_tag(g_img, WIN, WIN, 0, 96, false, false,
             LEVEL_BLACK, LEVEL_WHITE, LEVEL_BLACK);
  run("C no quiet, dark bg", 0);

  render_tag(g_img, WIN, WIN, 0, 96, true, false,
             LEVEL_BLACK, LEVEL_WHITE, LEVEL_BLACK);
  run("D quiet ring, dark bg", 0);

  render_tag(g_img, WIN, WIN, 0, 96, true, true,
             LEVEL_BLACK, LEVEL_WHITE, LEVEL_BLACK);
  run("E inverted", 0);

  /* 48 px is 6 px per cell, the decoder's minimum, over min_side_px = 40. */
  render_tag(g_img, WIN, WIN, 0, 48, true, false,
             LEVEL_BLACK, LEVEL_WHITE, LEVEL_WHITE);
  run("F small 48px", 0);

  /* 32 px is under min_side_px. */
  render_tag(g_img, WIN, WIN, 0, 32, true, false,
             LEVEL_BLACK, LEVEL_WHITE, LEVEL_WHITE);
  run("G tiny 32px", 0);

  render_tag(g_img, WIN, WIN, 0, 96, true, false, 110, 145, 145);
  run("H low contrast", 0);

  for (int r = 1; r <= 3; r++) {
    char nm[24];
    snprintf(nm, sizeof(nm), "%c rotated %d deg", 'H' + r, r * 90);
    render_tag_rot(g_img, WIN, WIN, 0, 96, true, false,
                   LEVEL_BLACK, LEVEL_WHITE, LEVEL_WHITE, r);
    run(nm, 0);
  }

  Serial.println();
  if (ok_a > 0) {
    Serial.println("VERDICT: detector decodes a perfect tag. The fault is in the");
    Serial.println("         IMAGING chain -- compare the camera's stats against the");
    Serial.println("         cases above and take the one whose signature matches.");
  } else {
    Serial.println("VERDICT: the detector FAILED on a perfect synthetic tag. Check the");
    Serial.println("         cell map above first: if the ring-1 border is not all '#'");
    Serial.println("         the renderer is wrong, otherwise the decoder is.");
  }
  Serial.println("=== end ===");

  delay(5000);
}
