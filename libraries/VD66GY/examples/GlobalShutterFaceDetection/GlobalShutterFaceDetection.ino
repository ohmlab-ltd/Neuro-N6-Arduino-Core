/*
  Global shutter face detection

  Finds faces in the global-shutter camera's picture and streams a box for
  each to Neuro Studio over USB. A moving face keeps its shape: the whole
  frame is exposed at one instant. Once a second the sketch reports how many
  frames the model ran on and the face it is surest of.

  Hardware: Neuro N6 with a VD66GY camera on the Neuro Vision carrier.

  This example code is in the public domain. The model it embeds,
  centerface-hwc.tflite, is Star-Clouds' CenterFace and is licensed under MIT.
  It was trained on the WIDER FACE training set, whose terms are not part of
  that licence. See models/README.md.
*/

#include <VD66GY_Arduino.h>
#include <PostProcess.h>
#include <n6_face.h>

#pragma neuron6 model="centerface-hwc.tflite" name=face_det

static const float DETECT_CONF = 0.50f;   // detector score needed

// VD66GY camera(VD66GY_CARRIER_THERMAL); on the Thermal carrier.
VD66GY camera;
NEURON6_DECLARE_MODEL(face_det);

IPostProcessor *detector = nullptr;   // the FaceID detector
static uint32_t passes = 0;           // detector passes since the last report

// The FaceID detector publishes nothing. This post-processor runs it, then
// sends Neuro Studio a box labelled "face" for each face.
static int32_t facesInit(void *ctx, NN_Instance_TypeDef *model) {
  (void)ctx;
  return detector->init(detector->ctx, model);
}

static int32_t facesRun(void *ctx, NN_Instance_TypeDef *model) {
  (void)ctx;
  passes++;
  return detector->run(detector->ctx, model);
}

static void facesEmit(void *ctx) {
  (void)ctx;
  const int n = n6_face_count();

  // u8 count, then per face u16 cx, cy, w, h as fractions of the frame, a u8
  // score and a u8 class. The corners are clipped to the frame.
  uint8_t boxes[1 + N6_FACE_MAX * 10];
  uint32_t len = 0;
  boxes[len++] = (uint8_t)n;
  for (int i = 0; i < n; i++) {
    const n6_face_t *f = n6_face_get(i);
    const float l = vnd_meta_clamp01(f->cx - 0.5f * f->w);
    const float r = vnd_meta_clamp01(f->cx + 0.5f * f->w);
    const float t = vnd_meta_clamp01(f->cy - 0.5f * f->h);
    const float b = vnd_meta_clamp01(f->cy + 0.5f * f->h);
    vnd_meta_put_u16_le(&boxes[len], vnd_meta_f01_to_u16(0.5f * (l + r))); len += 2;
    vnd_meta_put_u16_le(&boxes[len], vnd_meta_f01_to_u16(0.5f * (t + b))); len += 2;
    vnd_meta_put_u16_le(&boxes[len], vnd_meta_f01_to_u16(r - l));          len += 2;
    vnd_meta_put_u16_le(&boxes[len], vnd_meta_f01_to_u16(b - t));          len += 2;
    boxes[len++] = vnd_meta_conf_to_q8(f->score);
    boxes[len++] = 0;
  }

  // The model on VND_META_SLOT_MODEL_0 owns the frame's boxes and labels.
  vnd_meta_builder_t meta;
  vnd_meta_builder_begin(&meta, VND_META_SLOT_MODEL_0);
  (void)vnd_meta_builder_put_tlv(&meta, META_BOXES, boxes, (uint16_t)len);
  for (int i = 0; i < n; i++) {
    const uint8_t label[] = { (uint8_t)i, 'f', 'a', 'c', 'e' };   // box index, then the text
    (void)vnd_meta_builder_put_tlv(&meta, META_ISEG_CLASSSTR, label, sizeof(label));
  }
  vnd_meta_builder_finish(&meta);
}

static IPostProcessor faces_pp = {
  .init = facesInit, .run = facesRun,
  .draw_lcd = nullptr, .emit_metadata = facesEmit,
  .ctx = nullptr,
};

void setup() {
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera      = CAMERA_VGA;              // 88 frames per second
  cfg.isp_enabled = VD66_isp_tuning_auto;    // automatic white balance
  cfg.transport   = TRANSPORT_USB;

  // The model's input, a square from the middle of the frame.
  cfg.nn.width  = 128;
  cfg.nn.height = 128;
  cfg.nn.aspect = ASPECT_CROP_CENTER;

  detector = n6_face_detector();
  n6_face_detector_set_threshold(DETECT_CONF);
  Vision.begin(&NN_Instance_face_det, &faces_pp, cfg);
}

void loop() {
  Vision.run();

  static uint32_t lastReport = 0;
  if (millis() - lastReport < 1000) return;
  lastReport = millis();

  // The face the detector is surest of. Its centre runs from 0 to 1 across
  // the camera frame.
  const int n = n6_face_count();
  int best = -1;
  for (int i = 0; i < n; i++) {
    if (best < 0 || n6_face_get(i)->score > n6_face_get(best)->score) best = i;
  }

  char line[96];
  if (best >= 0) {
    const n6_face_t *f = n6_face_get(best);
    snprintf(line, sizeof(line), "[FACE] %lu passes/s, %d faces, best %.2f at %.2f,%.2f",
             (unsigned long)passes, n, (double)f->score, (double)f->cx, (double)f->cy);
  } else {
    // peak is the detector's best score, above DETECT_CONF or not.
    snprintf(line, sizeof(line), "[FACE] %lu passes/s, no faces, peak %.2f",
             (unsigned long)passes, (double)n6_face_last_peak());
  }
  passes = 0;
  Serial.println(line);
  vnd_meta_publish_debug_text(line);
}
