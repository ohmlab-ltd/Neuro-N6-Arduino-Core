/*
  Global shutter motion

  Finds what moves between frames of the global-shutter camera, with no
  neural network, and streams a box around the largest moving region to
  Neuro Studio with its speed. A moving object keeps its shape: the whole
  frame is exposed at one instant.

  Hardware: Neuro N6 with a VD66GY camera on the Neuro Vision carrier.

  This example code is in the public domain.
*/

#include <VD66GY_Arduino.h>
#include <PostProcess.h>
#include <n6_imgproc.h>
#include <math.h>

#define MO_W         160   // analysed picture: the 640x480 frame at a quarter size
#define MO_H         120
#define VISION_KIND  0x04  // blob, for Neuro Studio

// Neuro Studio shows a slider for each.
static int32_t DIFF_MIN   = 25;   // brightness change that counts as motion, 0 to 255
static int32_t MIN_PIXELS = 40;   // smallest moving region, in analysed pixels

// Box smoothing, and the still frames allowed before the region counts as gone.
static const float   TRACK_SMOOTHING  = 0.3f;
static const uint8_t TRACK_MAX_MISSES = 3;

// The capture buffer must be 32-byte aligned.
static uint8_t g_snapshot[MO_W * MO_H * 3]
  __attribute__((aligned(32), section(".ext_psram")));
static uint8_t g_payload[VND_VISION_CROP_DIM * VND_VISION_CROP_DIM * 2 + 64]
  __attribute__((section(".ext_psram")));

// Working images in internal RAM: each pass reads them many times.
static uint8_t  g_gray[MO_W * MO_H];
static uint8_t  g_prev[MO_W * MO_H];
static uint8_t  g_bin[MO_W * MO_H];
static uint8_t  g_scratch[MO_W * MO_H];
static uint16_t g_labels[MO_W * MO_H];

// VD66GY camera(VD66GY_CARRIER_THERMAL); on the Thermal carrier.
VD66GY camera;

static n6_tracker_t      g_tracker;
static n6img_component_t g_last;           // the largest moving region
static bool              g_last_valid = false;
static bool              g_have_prev  = false;
static float             g_speed = 0.0f;   // camera pixels per second

static uint32_t g_frames = 0, g_moving = 0, g_last_beat_ms = 0;

void setup() {
  Serial.begin(115200);

  // No model runs. cfg.nn sets the snapshot: the whole frame, scaled down.
  VisionConfig cfg;
  cfg.camera      = CAMERA_VGA;              // 88 frames per second
  cfg.isp_enabled = VD66_isp_tuning_auto;    // automatic white balance
  cfg.transport   = TRANSPORT_USB;
  cfg.nn.width    = MO_W;
  cfg.nn.height   = MO_H;
  cfg.nn.aspect   = ASPECT_STRETCH;
  Vision.begin(cfg);

  n6_tracker_init(&g_tracker, TRACK_SMOOTHING, TRACK_MAX_MISSES);

  n6_tune_int("diff min",   &DIFF_MIN,   5, 128);
  n6_tune_int("min pixels", &MIN_PIXELS, 5, 2000);
}

void loop() {
  if (NN_capture_snapshot(g_snapshot, sizeof(g_snapshot))) {
    const uint32_t now = millis();
    g_frames++;

    n6img_rgb888_to_gray(g_snapshot, g_gray, MO_W, MO_H);
    int n = 0;
    n6img_component_t comp;
    if (g_have_prev) {
      // Pixels that changed, without single-pixel noise, joined into regions.
      n6img_absdiff(g_prev, g_gray, g_bin, MO_W, MO_H);
      n6img_threshold(g_bin, g_bin, MO_W, MO_H, (uint8_t)DIFF_MIN, 255);
      n6img_erode(g_bin, g_scratch, MO_W, MO_H, 1);
      n6img_dilate(g_bin, g_scratch, MO_W, MO_H, 2);
      n = n6img_find_components(g_bin, g_labels, MO_W, MO_H,
                                (uint32_t)MIN_PIXELS, &comp, 1);
    }
    memcpy(g_prev, g_gray, sizeof(g_prev));
    g_have_prev = true;

    static uint32_t prevMs = 0;
    if (n > 0) {
      g_moving++;
      // Speed from the centre's step since the last frame with motion.
      if (g_last_valid && now > prevMs) {
        const float dx = (comp.cx - g_last.cx) * (float)g_camera_full_w;
        const float dy = (comp.cy - g_last.cy) * (float)g_camera_full_h;
        const float v  = sqrtf(dx * dx + dy * dy) * 1000.0f / (float)(now - prevMs);
        g_speed = 0.7f * g_speed + 0.3f * v;
      }
      prevMs = now;
      g_last = comp;
      g_last_valid = true;
      n6_tracker_update_box(&g_tracker, comp.cx, comp.cy, comp.w, comp.h);
    } else {
      g_last_valid = false;
      if (!n6_tracker_miss(&g_tracker)) g_speed = 0.0f;   // the region is gone
    }
  }

  // Reports what the tracker holds, so the box stays through a brief pause.
  vnd_meta_vision_t vis;
  vnd_meta_vision_begin(&vis, VND_META_SLOT_BLOB, META_BLOB_FRAME,
                        g_payload, sizeof(g_payload));
  if (n6_tracker_is_tracking(&g_tracker)) {
    const n6_roi_t *r = n6_tracker_roi(&g_tracker);

    // Extra: u16 area, u16 hue at the centre, u16 moving pixels.
    const int px = constrain((int)(r->cx * (float)(MO_W - 1)), 0, MO_W - 1);
    const int py = constrain((int)(r->cy * (float)(MO_H - 1)), 0, MO_H - 1);
    uint8_t extra[6];
    vnd_meta_put_u16_le(&extra[0], vnd_meta_f01_to_u16(r->w * r->h));
    vnd_meta_put_u16_le(&extra[2], (uint16_t)n6_hue_at_rgb888(g_snapshot, MO_W, px, py));
    vnd_meta_put_u16_le(&extra[4], (uint16_t)((g_last_valid && g_last.pixels < 65535u)
                                              ? g_last.pixels : 0u));

    char label[32];
    snprintf(label, sizeof(label), "moving %.0f px/s", (double)g_speed);
    (void)vnd_meta_vision_add(&vis, r->cx, r->cy, r->w, r->h, label,
                              extra, sizeof(extra), g_snapshot, MO_W, MO_H);
  }
  // An empty report clears the last box from the video.
  vnd_meta_vision_finish(&vis);

  const uint32_t now = millis();
  if (now - g_last_beat_ms >= 2000) {
    g_last_beat_ms = now;
    vnd_meta_vision_announce(VISION_KIND);
    char beat[80];
    snprintf(beat, sizeof(beat), "[MOTION] %lu frames, %lu with motion, %.0f px/s",
             (unsigned long)g_frames, (unsigned long)g_moving, (double)g_speed);
    Serial.println(beat);
    vnd_meta_publish_debug_text(beat);
    g_frames = 0;
    g_moving = 0;
  }
}
