/*
  AprilTag reader

  Finds tag36h11 markers in the camera image and streams each one's id,
  position and rotation to Neuro Studio over USB. With a stored camera
  calibration it also reports the distance to the tag.

  Hardware: Neuro N6 with an OV5640 camera. A printed tag36h11 marker with a
  white margin around its black border.

  This example code is in the public domain.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>
#include <n6_apriltag.h>
#include <n6_apriltag_pose.h>

#define AT_DIM      320      // detector input, pixels square
#define MAX_TAGS    4        // tags reported per frame
#define VISION_KIND 0x08     // AprilTag, for Neuro Studio

// Detector settings. n6_tune_int() gives each a slider in Neuro Studio.
static int32_t MAX_HAMMING  = 2;    // bit errors accepted, 0 to 2
static int32_t MIN_SIDE     = 40;   // smallest tag, pixels
static int32_t MIN_CONTRAST = 20;   // contrast a tag edge needs

// Box smoothing, and the empty frames allowed before the tag counts as lost.
static const float   TRACK_SMOOTHING  = 0.5f;
static const uint8_t TRACK_MAX_MISSES = 4;

// Camera snapshot. The hardware that fills it needs 32-byte alignment.
static uint8_t g_snapshot[AT_DIM * AT_DIM * 3]
  __attribute__((aligned(32), section(".ext_psram")));

static uint8_t g_gray[AT_DIM * AT_DIM] __attribute__((section(".ext_psram")));
static uint8_t g_payload[MAX_TAGS * (VND_VISION_CROP_DIM * VND_VISION_CROP_DIM * 2 + 64) + 8]
  __attribute__((section(".ext_psram")));

// Width of the tag's black square in metres. Distance scales with it.
#define TAG_SIZE_M  0.080f

static n6_camera_cal_t g_cal;
static bool            g_have_cal = false;

static void load_calibration(void)
{
  n6_camera_cal_t stored;
  g_have_cal = n6_camera_cal_load(&stored);
  if (!g_have_cal) {
    Serial.println("[cal] none stored - tags will be reported without pose");
    Serial.println("[cal] push one with tools/push_calibration.py");
    return;
  }
  Serial.print("[cal] loaded: fx="); Serial.print(stored.fx, 1);
  Serial.print(" fy="); Serial.print(stored.fy, 1);
  Serial.print(" cx="); Serial.print(stored.cx, 1);
  Serial.print(" cy="); Serial.print(stored.cy, 1);
  Serial.print("  measured at "); Serial.print(stored.width);
  Serial.print("x"); Serial.println(stored.height);
  n6_camera_cal_scale(&stored, AT_DIM, AT_DIM, &g_cal);
}

OV5640 camera;

static n6_tracker_t g_tracker;

static uint32_t g_frames = 0, g_hits = 0, g_last_beat_ms = 0;
static int32_t  g_last_id = -1;

void setup() {
  Serial.begin(115200);
  load_calibration();

  // No model runs. cfg.nn sets the snapshot's size and crop.
  VisionConfig cfg;
  cfg.camera    = CAMERA_VGA;
  cfg.fps       = FPS_30;
  cfg.transport = TRANSPORT_USB;
  cfg.nn.width  = AT_DIM;
  cfg.nn.height = AT_DIM;
  cfg.nn.aspect = ASPECT_CROP_CENTER;
  cfg.af        = AF_CONTINUOUS;
  Vision.begin(cfg);

  n6_tracker_init(&g_tracker, TRACK_SMOOTHING, TRACK_MAX_MISSES);

  n6_tune_int("max hamming",  &MAX_HAMMING,  0, 2);
  n6_tune_int("min side px",  &MIN_SIDE,     10, 160);
  n6_tune_int("min contrast", &MIN_CONTRAST, 0, 128);

  Serial.println("[APRILTAG] ready -- tag36h11, ids 0..586");
  vnd_meta_publish_debug_text("[APRILTAG] detector ready");
}

void loop() {
  n6_apriltag_t tags[MAX_TAGS];
  int n = 0;

  if (NN_capture_snapshot(g_snapshot, sizeof(g_snapshot))) {
    g_frames++;

    const n6_apriltag_cfg_t at_cfg = { MAX_HAMMING, MIN_SIDE, MIN_CONTRAST };
    n = n6_apriltag_find_rgb888(g_snapshot, AT_DIM, AT_DIM, g_gray,
                                &at_cfg, tags, MAX_TAGS);

    if (n > 0) {
      // Tags are returned best first.
      n6_tracker_update_box(&g_tracker, tags[0].cx, tags[0].cy,
                                        tags[0].w,  tags[0].h);
      g_hits++;

      if (tags[0].id != g_last_id) {
        g_last_id = tags[0].id;
        char line[64];
        snprintf(line, sizeof(line), "[APRILTAG] id=%d angle=%.0f deg",
                 (int)tags[0].id, (double)tags[0].angle);
        Serial.println(line);
        vnd_meta_publish_debug_text(line);
      }
    } else {
      (void)n6_tracker_miss(&g_tracker);
      if (!n6_tracker_is_tracking(&g_tracker)) g_last_id = -1;
    }
  }

  vnd_meta_vision_t vis;
  vnd_meta_vision_begin(&vis, VND_META_SLOT_APRILTAG, META_APRILTAG,
                        g_payload, sizeof(g_payload));

  for (int i = 0; i < n; i++) {
    const n6_apriltag_t *t = &tags[i];

    uint8_t extra[8 + 1 + 12];
    vnd_meta_put_u16_le(&extra[0], (uint16_t)t->id);
    extra[2] = 0;                                   // family, 0 is tag36h11
    extra[3] = (uint8_t)t->hamming;
    vnd_meta_put_u16_le(&extra[4], vnd_meta_f01_to_u16(t->margin));
    vnd_meta_put_u16_le(&extra[6], (uint16_t)(int16_t)(t->angle * 100.0f));

    // extra[8] is 1 when a pose follows.
    extra[8] = 0;
    memset(&extra[9], 0, 12);
    if (g_have_cal) {
      n6_apriltag_pose_t p;
      if (n6_apriltag_pose(t, AT_DIM, AT_DIM, &g_cal, TAG_SIZE_M, &p)) {
        extra[8] = 1;

        static uint32_t last_pose_ms = 0;
        if (i == 0 && millis() - last_pose_ms >= 500u) {
          last_pose_ms = millis();
          char pl[96];
          snprintf(pl, sizeof(pl),
                   "[pose] id=%d  x=%+.3f y=%+.3f z=%.3f m  rx=%+.0f ry=%+.0f rz=%+.0f deg",
                   (int)t->id, (double)p.tx, (double)p.ty, (double)p.tz,
                   (double)p.rx, (double)p.ry, (double)p.rz);
          Serial.println(pl);
        }
        vnd_meta_put_u16_le(&extra[9],  (uint16_t)(int16_t)(p.tx * 1000.0f));
        vnd_meta_put_u16_le(&extra[11], (uint16_t)(int16_t)(p.ty * 1000.0f));
        vnd_meta_put_u16_le(&extra[13], (uint16_t)(int16_t)(p.tz * 1000.0f));
        vnd_meta_put_u16_le(&extra[15], (uint16_t)(int16_t)(p.rx * 100.0f));
        vnd_meta_put_u16_le(&extra[17], (uint16_t)(int16_t)(p.ry * 100.0f));
        vnd_meta_put_u16_le(&extra[19], (uint16_t)(int16_t)(p.rz * 100.0f));
      }
    }

    char label[24];
    snprintf(label, sizeof(label), "tag %d", (int)t->id);

    (void)vnd_meta_vision_add(&vis, t->cx, t->cy, t->w, t->h, label,
                              extra, sizeof(extra),
                              g_snapshot, AT_DIM, AT_DIM);
  }
  // An empty report clears the last box from the video.
  vnd_meta_vision_finish(&vis);

  const uint32_t now = millis();
  if (now - g_last_beat_ms >= 2000) {
    g_last_beat_ms = now;
    vnd_meta_vision_announce(VISION_KIND);
    char beat[80];
    snprintf(beat, sizeof(beat), "[APRILTAG] frames=%lu hits=%lu%s pose=%s",
             (unsigned long)g_frames, (unsigned long)g_hits,
             n6_tracker_is_tracking(&g_tracker) ? " tracking" : "",
             g_have_cal ? "calibrated" : "no-calibration");
    Serial.println(beat);
  }
}
