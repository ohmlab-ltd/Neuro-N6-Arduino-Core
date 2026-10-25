/**
 ******************************************************************************
 * WiFiTwoWay.ino
 *
 * Bench sketch for the network link in n6_host_contract.h, driven by
 * tools/bench/n6bench.py wifitest. The board hosts a SoftAP with its own
 * password, printed at start-up, streams the camera and publishes the OV5640's
 * camera-control announce, two tuning parameters, a constant result with a
 * thumbnail and a beat line.
 *
 * Hardware: Neuro N6 with the MAYA-W476 wireless module and an OV5640 camera.
 *
 * Another name or password: a file named wifi_secrets.h next to this sketch
 * containing
 *
 *     #define BENCH_AP_SSID "your network name"
 *     #define BENCH_AP_PSK  "your passphrase"
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include <MayaW476.h>
#include <OV5640_Arduino.h>
#include <PostProcess.h>

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

#ifndef BENCH_AP_SSID
#define BENCH_AP_SSID  "NeuroN6-Bench"
#endif

#define PATTERN_DIM   64     // the test pattern the thumbnail is cut from
#define VISION_KIND   0x01   // shown in Neuro Studio's QR panel
#define RESULT_TEXT   "bench"

OV5640 camera;

// Tuning parameters, registered in this order: "level" is id 0, "flag" id 1.
static int32_t g_level = 50;
static uint8_t g_flag  = 0;

// Test pattern, RGB888, and the result scratch buffer.
static uint8_t g_pattern[PATTERN_DIM * PATTERN_DIM * 3]
  __attribute__((section(".ext_psram")));
static uint8_t g_payload[VND_VISION_CROP_DIM * VND_VISION_CROP_DIM * 2 + 64]
  __attribute__((section(".ext_psram")));

static void terminal(const char *line)
{
  vnd_meta_publish_debug_text(line);
  Serial.println(line);
}

void setup() {
  Serial.begin(115200);

  MayaW476.setTimeout(15000);
#if defined(BENCH_AP_PSK)
  const wl_status_t st = MayaW476.beginAP(BENCH_AP_SSID, BENCH_AP_PSK);
#else
  const wl_status_t st = MayaW476.beginAP(BENCH_AP_SSID);
  Serial.print("[bench] password ");
  Serial.println(MayaW476.devicePassword());
#endif
  char line[96];
  snprintf(line, sizeof(line), "[bench] WiFi status %d, address %s",
           (int)st, MayaW476.ipv4String());
  Serial.println(line);

  n6_tune_int("level", &g_level, 0, 100);
  n6_tune_bool("flag", &g_flag);

  for (int y = 0; y < PATTERN_DIM; y++) {
    for (int x = 0; x < PATTERN_DIM; x++) {
      uint8_t *p = &g_pattern[(y * PATTERN_DIM + x) * 3];
      p[0] = (uint8_t)(x * 4);
      p[1] = (uint8_t)(y * 4);
      p[2] = (((x / 8) + (y / 8)) & 1) ? 255 : 0;
    }
  }

  VisionConfig cfg;
  cfg.camera    = CAMERA_VGA;
  cfg.fps       = FPS_15;
  cfg.transport = TRANSPORT_WIFI_SOFTAP;
  Vision.begin(cfg);   // camera and stream only: no model, no run() needed

  terminal("[bench] WiFiTwoWay ready");
}

void loop() {
  static int32_t  shown_level = -1;
  static int      shown_flag  = -1;
  static int      shown_gate  = -1;
  static uint32_t beats = 0, t_beat = 0, t_result = 0, t_kind = 0;
  const uint32_t now = millis();
  char line[64];

  if (g_level != shown_level || (int)g_flag != shown_flag) {
    shown_level = g_level;
    shown_flag  = g_flag;
    snprintf(line, sizeof(line), "[tune] level=%ld flag=%d",
             (long)shown_level, shown_flag);
    terminal(line);
  }

  const int gate = n6_host_evidence_wanted(N6_EVIDENCE_CROP) ? 1 : 0;
  if (gate != shown_gate) {
    shown_gate = gate;
    terminal(gate ? "[gate] crops wanted: yes" : "[gate] crops wanted: no");
  }

  // The thumbnail is cut only while a host holds a subscription, SUB0.
  if (now - t_result >= 200) {
    t_result = now;
    const uint16_t text_len = sizeof(RESULT_TEXT) - 1;
    uint8_t extra[4 + sizeof(RESULT_TEXT) - 1];
    extra[0] = 1;                                   // "version"
    extra[1] = 0;                                   // "ecc level"
    extra[2] = (uint8_t)(text_len & 0xFF);
    extra[3] = (uint8_t)(text_len >> 8);
    memcpy(&extra[4], RESULT_TEXT, text_len);

    vnd_meta_vision_t vis;
    vnd_meta_vision_begin(&vis, VND_META_SLOT_QR, META_QR_FRAME,
                          g_payload, sizeof(g_payload));
    (void)vnd_meta_vision_add(&vis, 0.5f, 0.5f, 0.4f, 0.4f, RESULT_TEXT,
                              extra, sizeof(extra),
                              g_pattern, PATTERN_DIM, PATTERN_DIM);
    vnd_meta_vision_finish(&vis);
  }

  if (now - t_kind >= 2000) {
    t_kind = now;
    vnd_meta_vision_announce(VISION_KIND);
  }

  if (now - t_beat >= 1000) {
    t_beat = now;
    snprintf(line, sizeof(line), "[beat] %lu", (unsigned long)++beats);
    terminal(line);
  }

  delay(20);
}
