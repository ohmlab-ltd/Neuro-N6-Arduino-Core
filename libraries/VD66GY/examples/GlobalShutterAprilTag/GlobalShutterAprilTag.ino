/*
  Global shutter AprilTag

  Finds tag36h11 markers in the global-shutter camera's picture and streams
  each one's id, position and rotation to Neuro Studio over USB. A tag on a
  moving object keeps its square shape. With a stored camera calibration the
  sketch also reports each tag's distance and angles.

  Hardware: Neuro N6 with a VD66GY camera on the Neuro Vision carrier. A
  printed tag36h11 marker with a white margin around its black border.
  For distances, store a calibration measured at 640x480 with
  tools/push_calibration.py.

  This example code is in the public domain.
*/

#include <VD66GY_Arduino.h>
#include <PostProcess.h>
#include <n6_apriltag.h>
#include <n6_apriltag_pose.h>

#define AT_DIM      320      // detector input, pixels square
#define MAX_TAGS    4        // tags reported per frame
#define VISION_KIND 0x08     // AprilTag, for Neuro Studio

// Side of the tag's black square in metres. Distance scales with it.
#define TAG_SIZE_M  0.080f

// Detector settings, each with a slider in Neuro Studio.
static int32_t MAX_HAMMING  = 2;    // bit errors accepted, 0 to 2
static int32_t MIN_SIDE     = 40;   // smallest tag, pixels
static int32_t MIN_CONTRAST = 20;   // contrast a tag edge needs

// Box smoothing, and the empty frames allowed before the tag counts as lost.
static const float   TRACK_SMOOTHING  = 0.5f;
static const uint8_t TRACK_MAX_MISSES = 4;

// The capture buffer must be 32-byte aligned.
static uint8_t g_snapshot[AT_DIM * AT_DIM * 3]
  __attribute__((aligned(32), section(".ext_psram")));
static uint8_t g_gray[AT_DIM * AT_DIM] __attribute__((section(".ext_psram")));
static uint8_t g_payload[MAX_TAGS * (VND_VISION_CROP_DIM * VND_VISION_CROP_DIM * 2 + 64) + 8]
  __attribute__((section(".ext_psram")));

// VD66GY camera(VD66GY_CARRIER_THERMAL); on the Thermal carrier.
VD66GY camera;

static n6_camera_cal_t g_stored;    // as measured
static n6_camera_cal_t g_cal;       // for the detector's square
static bool g_have_stored = false;
static bool g_have_cal    = false;

static n6_tracker_t g_tracker;

static uint32_t g_frames = 0, g_hits = 0, g_last_beat_ms = 0;
static int32_t  g_last_id = -1;

// The detector sees a square cut from the camera frame and scaled to AT_DIM,
// so the stored intrinsics are moved and scaled the same way. The cut is
// known once the camera streams.
static void windowCalibration() {
  if (g_nn_crop_w == 0 || g_nn_crop_h == 0) return;
  n6_camera_cal_t frame;
  n6_camera_cal_scale(&g_stored, g_camera_full_w, g_camera_full_h, &frame);
  const float sx = (float)AT_DIM / (float)g_nn_crop_w;
  const float sy = (float)AT_DIM / (float)g_nn_crop_h;
  g_cal = frame;
  g_cal.fx = frame.fx * sx;
  g_cal.fy = frame.fy * sy;
  g_cal.cx = (frame.cx - (float)g_nn_crop_x) * sx;
  g_cal.cy = (frame.cy - (float)g_nn_crop_y) * sy;
  g_cal.width  = AT_DIM;
  g_cal.height = AT_DIM;
  g_have_cal = true;
}

void setup() {
  Serial.begin(115200);

  g_have_stored = n6_camera_cal_load(&g_stored);
  if (g_have_stored) {
    char line[96];
    snprintf(line, sizeof(line), "[cal] fx=%.1f fy=%.1f cx=%.1f cy=%.1f, measured at %ux%u",
             (double)g_stored.fx, (double)g_stored.fy, (double)g_stored.cx,
             (double)g_stored.cy, (unsigned)g_stored.width, (unsigned)g_stored.height);
    Serial.println(line);
  } else {
    Serial.println("[cal] none stored: tags are reported without distance");
  }

  // No model runs. cfg.nn sets the snapshot: a centred square, AT_DIM wide.
  VisionConfig cfg;
  cfg.camera      = CAMERA_VGA;              // 88 frames per second
  cfg.isp_enabled = VD66_isp_tuning_auto;    // automatic white balance
  cfg.transport   = TRANSPORT_USB;
  cfg.nn.width    = AT_DIM;
  cfg.nn.height   = AT_DIM;
  cfg.nn.aspect   = ASPECT_CROP_CENTER;
  Vision.begin(cfg);

  n6_tracker_init(&g_tracker, TRACK_SMOOTHING, TRACK_MAX_MISSES);

  n6_tune_int("max hamming",  &MAX_HAMMING,  0, 2);
  n6_tune_int("min side px",  &MIN_SIDE,     10, 160);
  n6_tune_int("min contrast", &MIN_CONTRAST, 0, 128);
}

void loop() {
  n6_apriltag_t tags[MAX_TAGS];
  int n = 0;

  if (NN_capture_snapshot(g_snapshot, sizeof(g_snapshot))) {
    g_frames++;
    if (g_have_stored && !g_have_cal) windowCalibration();

    const n6_apriltag_cfg_t at_cfg = { MAX_HAMMING, MIN_SIDE, MIN_CONTRAST };
    n = n6_apriltag_find_rgb888(g_snapshot, AT_DIM, AT_DIM, g_gray,
                                &at_cfg, tags, MAX_TAGS);
    if (n < 0) n = 0;

    if (n > 0) {
      // Tags are returned best first.
      n6_tracker_update_box(&g_tracker, tags[0].cx, tags[0].cy, tags[0].w, tags[0].h);
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

    // Extra: u16 id, u8 family, u8 hamming, u16 margin, i16 angle in
    // centi-degrees, u8 pose_valid, i16 tx, ty, tz in mm, i16 rx, ry, rz in
    // centi-degrees.
    uint8_t extra[21] = {0};
    vnd_meta_put_u16_le(&extra[0], (uint16_t)t->id);
    extra[2] = 0;                                   // tag36h11
    extra[3] = (uint8_t)t->hamming;
    vnd_meta_put_u16_le(&extra[4], vnd_meta_f01_to_u16(t->margin));
    vnd_meta_put_u16_le(&extra[6], (uint16_t)(int16_t)(t->angle * 100.0f));

    n6_apriltag_pose_t p;
    if (g_have_cal && n6_apriltag_pose(t, AT_DIM, AT_DIM, &g_cal, TAG_SIZE_M, &p)) {
      extra[8] = 1;
      vnd_meta_put_u16_le(&extra[9],  (uint16_t)(int16_t)(p.tx * 1000.0f));
      vnd_meta_put_u16_le(&extra[11], (uint16_t)(int16_t)(p.ty * 1000.0f));
      vnd_meta_put_u16_le(&extra[13], (uint16_t)(int16_t)(p.tz * 1000.0f));
      vnd_meta_put_u16_le(&extra[15], (uint16_t)(int16_t)(p.rx * 100.0f));
      vnd_meta_put_u16_le(&extra[17], (uint16_t)(int16_t)(p.ry * 100.0f));
      vnd_meta_put_u16_le(&extra[19], (uint16_t)(int16_t)(p.rz * 100.0f));

      static uint32_t lastPoseMs = 0;
      if (i == 0 && millis() - lastPoseMs >= 500) {
        lastPoseMs = millis();
        char line[96];
        snprintf(line, sizeof(line),
                 "[pose] id=%d x=%+.3f y=%+.3f z=%.3f m rx=%+.0f ry=%+.0f rz=%+.0f deg",
                 (int)t->id, (double)p.tx, (double)p.ty, (double)p.tz,
                 (double)p.rx, (double)p.ry, (double)p.rz);
        Serial.println(line);
      }
    }

    char label[24];
    snprintf(label, sizeof(label), "tag %d", (int)t->id);
    (void)vnd_meta_vision_add(&vis, t->cx, t->cy, t->w, t->h, label,
                              extra, sizeof(extra), g_snapshot, AT_DIM, AT_DIM);
  }
  // An empty report clears the last box from the video.
  vnd_meta_vision_finish(&vis);

  const uint32_t now = millis();
  if (now - g_last_beat_ms >= 2000) {
    g_last_beat_ms = now;
    vnd_meta_vision_announce(VISION_KIND);
    char beat[80];
    snprintf(beat, sizeof(beat), "[APRILTAG] %lu frames, %lu with a tag%s",
             (unsigned long)g_frames, (unsigned long)g_hits,
             g_have_stored ? "" : ", no calibration");
    Serial.println(beat);
    vnd_meta_publish_debug_text(beat);
    g_frames = 0;
    g_hits = 0;
  }
}
