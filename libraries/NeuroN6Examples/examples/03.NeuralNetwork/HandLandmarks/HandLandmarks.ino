/*
  Hand landmarks

  A palm detector finds a hand, then a second model runs on a crop of that
  hand and returns 21 points across the palm and fingers. The results stream
  to Neuro Studio over USB.

  Hardware: Neuro N6 with an OV5640 camera.

  This example code is in the public domain. The models it embeds, MediaPipe's
  palm detector and hand landmark network, are licensed under Apache-2.0; see
  models/README.md.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>

#pragma neuron6 model="033_palm_detection_full_quant_pc_uf_od.tflite"    name=palm_det
#pragma neuron6 model="033_hand_landmark_full_quant_pc_uf_handl.tflite"  name=hand_lm

// Palm confidence and overlap thresholds, then the hand model's score
// threshold. The hand model's score tops out at 0.50, so 0.50 keeps only its
// surest results.
static const float PALM_CONF  = 0.50f;
static const float PALM_IOU   = 0.30f;
static const float HAND_SCORE = 0.50f;

// Smoothing of 0 follows the hand exactly, nearer 1 is steadier but lags.
// TRACK_MAX_MISSES is how many frames in a row the hand model may find nothing
// before the palm detector runs again.
static const float   TRACK_SMOOTHING  = 0.5f;
static const uint8_t TRACK_MAX_MISSES = 5;

// Print interval.
static const uint32_t HEARTBEAT_MS = 1000;

// Palm box to hand region. The picture inside the crop is not turned upright.
static const n6_roi_spec_t PALM_TO_HAND = {
  .rot_kp_start = 0, .rot_kp_end = 2, .rot_kp_end2 = -1, .rot_kp_end3 = -1,
  .target_angle_deg = 90.0f,
  .scale_x = 2.6f, .scale_y = 2.6f,
  .shift_x = 0.0f, .shift_y = -0.5f,
  .square_long = true,
};

// Hand points to the next region. The indices count along HAND_STEADY_POINTS:
// 0 is the wrist, and 4, 6 and 8 are the index, middle and ring knuckles.
static const n6_roi_spec_t HAND_TO_HAND = {
  .rot_kp_start = 0, .rot_kp_end = 4, .rot_kp_end2 = 6, .rot_kp_end3 = 8,
  .target_angle_deg = 90.0f,
  .scale_x = 2.0f, .scale_y = 2.0f,
  .shift_x = 0.0f, .shift_y = -0.1f,
  .square_long = true,
};

// The model's points that stay put while the fingers move.
static const uint8_t HAND_STEADY_POINTS[] = {
  0,              // wrist
  1, 2, 3,        // thumb, leaving out its tip
  5, 6,           // index finger knuckle and middle joint
  9, 10,          // middle finger
  13, 14,         // ring finger
  17, 18,         // little finger
};
static const int HAND_STEADY_COUNT =
    sizeof(HAND_STEADY_POINTS) / sizeof(HAND_STEADY_POINTS[0]);

// OV5640 camera(CARRIER_TFT); on the TFT carrier.
OV5640 camera;
NEURON6_DECLARE_MODEL(palm_det);
NEURON6_DECLARE_MODEL(hand_lm);

static n6_tracker_t hand_tracker;

void setup() {
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera      = CAMERA_1080P;        // 1920x1080
  cfg.fps         = FPS_30;              // the fastest this resolution manages
  cfg.mirror_flip = CAMERA_AUTO_FLIP;

  // The palm detector's input size. The hand model's comes from the model.
  cfg.nn.width    = 192;
  cfg.nn.height   = 192;
  cfg.nn.aspect   = ASPECT_CROP_CENTER;  // a centred square crop

  cfg.transport   = TRANSPORT_USB;
  cfg.af          = AF_CONTINUOUS;
  cfg.strobe      = STROBE_DISABLED;

  Vision.begin(cfg);

  n6_tracker_init(&hand_tracker, TRACK_SMOOTHING, TRACK_MAX_MISSES);

  Serial.println("HandLandmarks: palm detection, then hand landmark tracking");
}

void loop() {
  if (!n6_tracker_is_tracking(&hand_tracker)) {
    Vision.runWith(&NN_Instance_palm_det,
                   PostProcess_PalmDetector_OnSlot(PALM_CONF, PALM_IOU,
                                                   VND_META_SLOT_MODEL_0),
                   OVL_CLEAR);

    if (pd_ui_nb_detections > 0) {
      const pd_ui_detection_t *d = &pd_ui_detections[0];
      n6_roi_t roi = n6_roi_build(d->x_center, d->y_center, d->width, d->height,
                                  &d->kps[0].x, 2, AI_PD_PP_NB_KEYPOINTS,
                                  &PALM_TO_HAND);
      n6_tracker_update(&hand_tracker, &roi);

      // Clear the palm box, or it stays on screen while the hand is followed.
      vnd_meta_invalidate_slot(VND_META_SLOT_MODEL_0);
    }
  }

  if (n6_tracker_is_tracking(&hand_tracker)) {
    const bool ran = Vision.runRoi(*n6_tracker_roi(&hand_tracker),
                                   &NN_Instance_hand_lm,
                                   PostProcess_HandLandmarks_OnSlot(
                                       HAND_SCORE, VND_META_SLOT_MODEL_1),
                                   OVL_RELOAD_CLEAR);

    if (ran && handlm_output.valid) {
      // x and y of each steady point, so the step through the list is 2.
      float steady[2 * HAND_STEADY_COUNT];
      for (int i = 0; i < HAND_STEADY_COUNT; i++) {
        const handlm_point_t &p = handlm_output.kps[HAND_STEADY_POINTS[i]];
        steady[2 * i]     = p.x;
        steady[2 * i + 1] = p.y;
      }
      n6_roi_t next = n6_roi_from_nn_keypoints(steady, 2, HAND_STEADY_COUNT,
                                               &HAND_TO_HAND);
      n6_tracker_update(&hand_tracker, &next);
    } else if (!n6_tracker_miss(&hand_tracker)) {
      // The hand is lost. Clear its points from the screen.
      vnd_meta_invalidate_slot(VND_META_SLOT_MODEL_1);
    }
  }

  // The fingertips are points 4, 8, 12, 16 and 20, thumb to little finger.
  // Each x and y runs from 0 to 1 across the region being followed.
  static uint32_t lastBeat = 0;
  const uint32_t now = millis();
  if (now - lastBeat >= HEARTBEAT_MS) {
    lastBeat = now;
    if (n6_tracker_is_tracking(&hand_tracker) && handlm_output.valid) {
      Serial.print("[hand] score=");    Serial.print(handlm_output.score, 2);
      Serial.print(" indexTip=");       Serial.print(handlm_output.kps[8].x, 2);
      Serial.print(",");                Serial.print(handlm_output.kps[8].y, 2);
      Serial.println();
    } else {
      Serial.println("[hand] searching");
    }
  }
}
