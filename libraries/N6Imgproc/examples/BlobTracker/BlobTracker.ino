/*
  Blob tracker

  Follows the largest patch of a chosen colour in the camera picture, with no
  neural network. The colour settings can be adjusted from Neuro Studio.

  Hardware: Neuro N6 with an OV5640 camera.

  This example code is in the public domain.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>
#include <n6_imgproc.h>

#define BLOB_DIM     256   // side of the square picture searched, in pixels
#define VISION_KIND  0x04  // colour blobs

// TRACK_SMOOTHING: 0 follows the blob exactly, nearer 1 is steadier but lags.
// TRACK_MAX_MISSES: empty frames in a row before the blob counts as lost.
static const float   TRACK_SMOOTHING  = 0.5f;
static const uint8_t TRACK_MAX_MISSES = 4;

// The colour to look for. Neuro Studio puts a slider on each value.
static int32_t TARGET_HUE = 25;   // degrees: 0 red, 60 yellow, 120 green, 240 blue
static int32_t HUE_TOL    = 18;   // degrees either side of that hue
static int32_t SAT_MIN    = 55;   // how colourful a pixel has to be, 0 to 255
static int32_t VAL_MIN    = 50;   // how bright a pixel has to be, 0 to 255

static n6_blob_cfg_t BLOB_CFG = {
  .target_hue = 0, .hue_tol = 0, .sat_min = 0, .val_min = 0,  // set each frame
  // false follows the largest strongly coloured patch of any colour.
  .gate_hue = true,
  .min_cell_hits = 2,   // matching pixels a grid cell needs
  .min_blob_px   = 20,  // matching pixels the whole patch needs
};

// The camera snapshot. It must be aligned to 32 bytes, or every grab fails.
static uint8_t g_snapshot[BLOB_DIM * BLOB_DIM * 3]
  __attribute__((aligned(32), section(".ext_psram")));

static uint8_t g_payload[VND_VISION_CROP_DIM * VND_VISION_CROP_DIM * 2 + 64]
  __attribute__((section(".ext_psram")));

OV5640 camera;

static n6_tracker_t g_tracker;

static n6_blob_t g_last;
static bool      g_last_valid = false;

static uint32_t g_frames = 0, g_hits = 0, g_last_beat_ms = 0;

void setup() {
  Serial.begin(115200);

  // No model is used. cfg.nn sets the size of the snapshot.
  VisionConfig cfg;
  cfg.camera    = CAMERA_VGA;
  cfg.fps       = FPS_30;
  cfg.transport = TRANSPORT_USB;
  cfg.nn.width  = BLOB_DIM;
  cfg.nn.height = BLOB_DIM;
  cfg.nn.aspect = ASPECT_CROP_CENTER;
  Vision.begin(cfg);

  n6_tracker_init(&g_tracker, TRACK_SMOOTHING, TRACK_MAX_MISSES);

  n6_tune_hue("target hue", &TARGET_HUE);
  n6_tune_int("hue tol",    &HUE_TOL, 0, 180);
  n6_tune_int("sat min",    &SAT_MIN, 0, 255);
  n6_tune_int("val min",    &VAL_MIN, 0, 255);

  Serial.println("[BLOB] ready");
}

void loop() {
  if (NN_capture_snapshot(g_snapshot, sizeof(g_snapshot))) {
    g_frames++;

    BLOB_CFG.target_hue = TARGET_HUE;
    BLOB_CFG.hue_tol    = HUE_TOL;
    BLOB_CFG.sat_min    = SAT_MIN;
    BLOB_CFG.val_min    = VAL_MIN;

    n6_blob_t b;
    if (n6_blob_find_rgb888(g_snapshot, BLOB_DIM, BLOB_DIM, &BLOB_CFG, &b)) {
      n6_tracker_update_box(&g_tracker, b.cx, b.cy, b.w, b.h);
      g_last = b;
      g_last_valid = true;
      g_hits++;
    } else {
      // The result is ignored: the check below decides what to draw.
      (void)n6_tracker_miss(&g_tracker);
      g_last_valid = false;
    }
  }

  // Reports what the tracker holds, so the box stays through a brief loss.
  vnd_meta_vision_t v;
  vnd_meta_vision_begin(&v, VND_META_SLOT_BLOB, META_BLOB_FRAME,
                        g_payload, sizeof(g_payload));

  if (n6_tracker_is_tracking(&g_tracker)) {
    const n6_roi_t *r = n6_tracker_roi(&g_tracker);

    // Extras: u16 area, u16 hue, u16 matched pixels.
    const bool have = g_last_valid;
    uint8_t extra[6];
    vnd_meta_put_u16_le(&extra[0],
        vnd_meta_f01_to_u16(have ? g_last.area : 0.0f));
    vnd_meta_put_u16_le(&extra[2], (uint16_t)(have ? g_last.hue : 0));
    vnd_meta_put_u16_le(&extra[4],
        (uint16_t)((have && g_last.pixels < 65535u) ? g_last.pixels : 0u));

    char label[24];
    snprintf(label, sizeof(label), "blob %d deg", have ? (int)g_last.hue : 0);

    (void)vnd_meta_vision_add(&v, r->cx, r->cy, r->w, r->h, label,
                              extra, sizeof(extra),
                              g_snapshot, BLOB_DIM, BLOB_DIM);
  }
  // An empty report clears the last box off the video.
  vnd_meta_vision_finish(&v);

  const uint32_t now = millis();
  if (now - g_last_beat_ms >= 2000) {
    g_last_beat_ms = now;
    vnd_meta_vision_announce(VISION_KIND);
    char beat[80];
    snprintf(beat, sizeof(beat), "[BLOB] frames=%lu hits=%lu%s",
             (unsigned long)g_frames, (unsigned long)g_hits,
             n6_tracker_is_tracking(&g_tracker) ? " tracking" : "");
    Serial.println(beat);
  }
}
