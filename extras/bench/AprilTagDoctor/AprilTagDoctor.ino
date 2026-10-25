/**
 ******************************************************************************
 * AprilTagDoctor.ino
 *
 * Reports the camera's image statistics and where the AprilTag pipeline stops:
 * blobs found, quads fitted, tags decoded. Sweeps the detector config.
 *
 * Hardware: Neuro N6 with an OV5640 camera.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#include <OV5640_Arduino.h>
#include <PostProcess.h>
#include <n6_apriltag.h>
#include <n6_imgproc.h>

#define AT_DIM 320

static uint8_t g_snapshot[AT_DIM * AT_DIM * 3]
  __attribute__((aligned(32), section(".ext_psram")));
static uint8_t g_gray[AT_DIM * AT_DIM] __attribute__((section(".ext_psram")));

OV5640 camera;

static void try_cfg(const char *name, int hamming, int min_side, int min_contrast)
{
  n6_apriltag_cfg_t cfg;
  cfg.max_hamming  = hamming;
  cfg.min_side_px  = min_side;
  cfg.min_contrast = min_contrast;

  n6_apriltag_t tags[4];
  const int n = n6_apriltag_find_rgb888(g_snapshot, AT_DIM, AT_DIM, g_gray,
                                        &cfg, tags, 4);
  n6_apriltag_stats_t st;
  n6_apriltag_last_stats(&st);

  char line[110];
  snprintf(line, sizeof(line),
           "  %-16s blobs=%-3ld quads=%-3ld dec=%-2ld | rej c=%ld b=%ld code=%ld besth=%ld%s",
           name, (long)st.blobs, (long)st.quads, (long)st.decoded,
           (long)st.rej_contrast, (long)st.rej_border, (long)st.rej_code,
           (long)st.best_hamming, (n > 0) ? "  <-- FOUND" : "");
  Serial.println(line);

  for (int i = 0; i < n && i < 4; i++) {
    snprintf(line, sizeof(line), "      id=%ld hamming=%ld margin=%.2f",
             (long)tags[i].id, (long)tags[i].hamming, (double)tags[i].margin);
    Serial.println(line);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1500);

  VisionConfig cfg;
  cfg.camera    = CAMERA_VGA;
  cfg.fps       = FPS_30;
  cfg.transport = TRANSPORT_USB;
  cfg.nn.width  = AT_DIM;
  cfg.nn.height = AT_DIM;
  cfg.nn.aspect = ASPECT_CROP_CENTER;
  cfg.af        = AF_CONTINUOUS;
  Vision.begin(cfg);

  Serial.println();
  Serial.println("=== AprilTag doctor ===");
  Serial.println("point the camera at your tag; the window is a CENTRE crop");

  camera.setAutoExposure(false);
  camera.setAutoGain(false);
  camera.setExposureUs(33000);      /* the most a 30 fps frame allows */
  camera.setGain(8.0f);
  Serial.println("forced: exposure 33 ms, gain 8x, auto OFF");
}

void loop()
{
  if (!NN_capture_snapshot(g_snapshot, sizeof(g_snapshot))) { delay(50); return; }

  n6img_rgb888_to_gray(g_snapshot, g_gray, AT_DIM, AT_DIM);
  n6img_stats_t is;
  n6img_statistics(g_gray, AT_DIM, AT_DIM, NULL, &is);
  uint32_t bins[256];
  n6img_histogram(g_gray, AT_DIM, AT_DIM, NULL, bins);
  const uint8_t t = n6img_otsu(bins);

  char line[110];
  Serial.println();
  snprintf(line, sizeof(line),
           "  image  min=%u max=%u mean=%u median=%u stdev=%lu(x256)  otsu=%u",
           is.min, is.max, is.mean, is.median, (unsigned long)is.stdev, t);
  Serial.println(line);
  if (is.max < 60)  Serial.println("  ! very dark - nothing will threshold");
  if (is.min > 200) Serial.println("  ! washed out - auto-exposure is blowing the screen out");
  if (is.stdev < 256u * 10u)
    Serial.println("  ! very low contrast - is the tag actually in the centre crop?");

  static int passes = 0;
  if (++passes == 6) {
    camera.setAutoExposure(true);
    camera.setAutoGain(true);
    Serial.println("  (auto exposure/gain restored)");
  }

  try_cfg("default",            2, 40, 20);
  try_cfg("smaller tags",       2, 15, 20);
  try_cfg("lower contrast",     2, 40,  8);
  try_cfg("small + low",        2, 15,  8);
  try_cfg("strict hamming",     0, 15,  8);

  delay(2000);
}
