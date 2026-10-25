/**
 ******************************************************************************
 * ImgprocSelfTest.ino
 *
 * Checks each image primitive against small images whose answers are known by
 * construction, with no camera. Each case prints ok or FAIL, and the RESULT
 * line gives the number of failures.
 *
 * Hardware: Neuro N6.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#include <n6_imgproc.h>

#define W 64
#define H 48

static uint8_t  a[W * H], b[W * H], scratch[W * H];
static uint16_t labels[W * H];
static int failures = 0;

static void ok(const char *name, bool pass, const char *detail)
{
  if (!pass) failures++;
  Serial.print("  ");
  Serial.print(pass ? "ok   " : "FAIL ");
  Serial.print(name);
  if (detail && *detail) { Serial.print("   "); Serial.print(detail); }
  Serial.println();
}

static void fill(uint8_t *p, uint8_t v) { memset(p, v, W * H); }

static void rect(uint8_t *p, int x0, int y0, int rw, int rh, uint8_t v)
{
  for (int y = y0; y < y0 + rh; y++)
    for (int x = x0; x < x0 + rw; x++)
      if (x >= 0 && x < W && y >= 0 && y < H) p[y * W + x] = v;
}

static void run(void)
{
  failures = 0;
  char msg[72];
  Serial.println();
  Serial.println("=== imgproc self-test (synthetic images, known answers) ===");

  fill(a, 100);
  n6img_stats_t st;
  n6img_statistics(a, W, H, NULL, &st);
  snprintf(msg, sizeof(msg), "min=%u max=%u mean=%u stdev=%lu px=%lu",
           st.min, st.max, st.mean, (unsigned long)st.stdev,
           (unsigned long)st.pixels);
  ok("flat field", st.min == 100 && st.max == 100 && st.mean == 100 &&
                   st.stdev == 0 && st.pixels == (uint32_t)(W * H), msg);

  fill(a, 0);
  rect(a, 0, 0, W / 2, H, 254);
  n6img_statistics(a, W, H, NULL, &st);
  uint32_t bins[256];
  n6img_histogram(a, W, H, NULL, bins);
  const uint8_t t = n6img_otsu(bins);
  snprintf(msg, sizeof(msg), "mean=%u median=%u otsu=%u bins[0]=%lu bins[254]=%lu",
           st.mean, st.median, t, (unsigned long)bins[0], (unsigned long)bins[254]);
  /* Every threshold in 0..253 scores identically on this image. The plateau
   * must resolve to its middle. */
  ok("bimodal", st.mean == 127 && bins[0] == (uint32_t)(W * H / 2) &&
                bins[254] == (uint32_t)(W * H / 2) && t > 100 && t < 160, msg);

  n6img_rect_t roi = { 0, 0, W / 2, H };
  n6img_statistics(a, W, H, &roi, &st);
  snprintf(msg, sizeof(msg), "mean=%u px=%lu", st.mean, (unsigned long)st.pixels);
  ok("roi restricts", st.mean == 254 && st.pixels == (uint32_t)(W / 2 * H), msg);

  n6img_threshold(a, b, W, H, 200, 255);
  uint32_t set = 0;
  for (int i = 0; i < W * H; i++) if (b[i]) set++;
  snprintf(msg, sizeof(msg), "set=%lu of %d", (unsigned long)set, W * H);
  ok("threshold", set == (uint32_t)(W * H / 2), msg);

  /* Overwrites the 0/254 image the cases above share, so it runs after them. */
  fill(a, 50);
  rect(a, 0, 0, W / 2, H, 200);
  n6img_histogram(a, W, H, NULL, bins);
  const uint8_t t2 = n6img_otsu(bins);
  snprintf(msg, sizeof(msg), "otsu=%u (modes at 50 and 200)", t2);
  ok("otsu between modes", t2 > 60 && t2 < 190, msg);

  fill(b, 0);
  rect(b, 20, 16, 10, 10, 255);
  n6img_erode(b, scratch, W, H, 1);
  uint32_t after_erode = 0;
  for (int i = 0; i < W * H; i++) if (b[i]) after_erode++;
  n6img_dilate(b, scratch, W, H, 1);
  uint32_t after_dilate = 0;
  for (int i = 0; i < W * H; i++) if (b[i]) after_dilate++;
  snprintf(msg, sizeof(msg), "100 -> erode %lu -> dilate %lu",
           (unsigned long)after_erode, (unsigned long)after_dilate);
  ok("erode/dilate", after_erode == 64 && after_dilate == 100, msg);

  fill(a, 90); fill(b, 90);
  n6img_absdiff(a, b, scratch, W, H);
  uint32_t nz = 0;
  for (int i = 0; i < W * H; i++) if (scratch[i]) nz++;
  rect(b, 5, 5, 4, 4, 200);
  n6img_absdiff(a, b, scratch, W, H);
  uint32_t nz2 = 0;
  for (int i = 0; i < W * H; i++) if (scratch[i]) nz2++;
  snprintf(msg, sizeof(msg), "identical=%lu moved=%lu",
           (unsigned long)nz, (unsigned long)nz2);
  ok("absdiff", nz == 0 && nz2 == 16, msg);

  fill(b, 0);
  rect(b,  2,  2,  4,  4, 255);
  rect(b, 20, 10, 10, 10, 255);
  rect(b, 45, 30,  6,  5, 255);
  n6img_component_t comp[8];
  const int n = n6img_find_components(b, labels, W, H, 4, comp, 8);
  snprintf(msg, sizeof(msg), "n=%d sizes %lu/%lu/%lu", n,
           (unsigned long)(n > 0 ? comp[0].pixels : 0),
           (unsigned long)(n > 1 ? comp[1].pixels : 0),
           (unsigned long)(n > 2 ? comp[2].pixels : 0));
  ok("components", n == 3 && comp[0].pixels == 100 && comp[1].pixels == 30 &&
                   comp[2].pixels == 16, msg);

  /* A diagonal streak is one component under 8-connectivity. */
  fill(b, 0);
  for (int i = 0; i < 40; i++) rect(b, i, i, 1, 1, 255);
  const int nd = n6img_find_components(b, labels, W, H, 1, comp, 8);
  snprintf(msg, sizeof(msg), "n=%d px=%lu", nd,
           (unsigned long)(nd > 0 ? comp[0].pixels : 0));
  ok("diagonal is one blob", nd == 1 && comp[0].pixels == 40, msg);

  fill(b, 0);
  rect(b, 2, 2, 2, 2, 255);
  rect(b, 20, 10, 10, 10, 255);
  const int nf = n6img_find_components(b, labels, W, H, 50, comp, 8);
  snprintf(msg, sizeof(msg), "n=%d", nf);
  ok("min_px filters", nf == 1 && comp[0].pixels == 100, msg);

  static uint8_t rgb[16 * 3];
  for (int i = 0; i < 8; i++)  { rgb[i*3] = 255; rgb[i*3+1] = 255; rgb[i*3+2] = 255; }
  for (int i = 8; i < 16; i++) { rgb[i*3] = 0;   rgb[i*3+1] = 0;   rgb[i*3+2] = 0; }
  uint8_t g[16];
  n6img_rgb888_to_gray(rgb, g, 16, 1);
  snprintf(msg, sizeof(msg), "white=%u black=%u", g[0], g[15]);
  ok("rgb888->gray", g[0] >= 254 && g[15] == 0, msg);

  Serial.println();
  Serial.print("RESULT: ");
  if (failures == 0) Serial.println("all primitives match their known answers");
  else { Serial.print(failures); Serial.println(" FAILED"); }
}

void setup() { Serial.begin(115200); delay(1500); }
void loop()  { run(); delay(5000); }
