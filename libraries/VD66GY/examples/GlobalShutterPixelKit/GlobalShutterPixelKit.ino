/*
  Global shutter PixelKit

  Runs a detector trained in PixelKit on the global-shutter camera and
  streams labelled boxes to Neuro Studio, with a slider there for the score a
  box needs. The sample model finds printed circles and squares. Once a
  second the sketch reports how many of each it sees.

  Hardware: Neuro N6 with a VD66GY camera on the Neuro Vision carrier.

  To test it, print pixelkit_shapes_test_sheet.pdf from the PixelKitDetector
  example's folder at 100% on A4.

  This example code is in the public domain. The model it embeds,
  pixelkit_shapes_int8.onnx, is licensed under Apache-2.0: it was fine-tuned
  from Megvii's YOLOX-Nano. See models/README.md.
*/

#include <VD66GY_Arduino.h>
#include <PostProcess.h>

#include "shapes_labels.h"   // class names, input size and thresholds

#pragma neuron6 model="pixelkit_shapes_int8.onnx" name=shapes

// VD66GY camera(VD66GY_CARRIER_THERMAL); on the Thermal carrier.
VD66GY camera;
NEURON6_DECLARE_MODEL(shapes);

IPostProcessor *detector = nullptr;

// The score a box needs, shown as a slider in Neuro Studio.
static float confidence = SHAPES_CONF_THRESHOLD;

void setup() {
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera      = CAMERA_VGA;              // 88 frames per second
  cfg.isp_enabled = VD66_isp_tuning_auto;    // automatic white balance
  cfg.nn.width    = SHAPES_INPUT_W;
  cfg.nn.height   = SHAPES_INPUT_H;
  cfg.nn.aspect   = ASPECT_LETTERBOX;        // the whole picture on grey, as PixelKit trains
  cfg.transport   = TRANSPORT_USB;

  n6_tune_float("confidence", &confidence, 0.01f, 0.99f);
  detector = PostProcess_PixelKitYOLOX(confidence, SHAPES_NMS_THRESHOLD,
                                       SHAPES_CLASSES, SHAPES_NUM_CLASSES);
  Vision.begin(&NN_Instance_shapes, detector, cfg);
}

void loop() {
  // Calling the factory again changes the settings of its one instance.
  static float applied = SHAPES_CONF_THRESHOLD;
  if (confidence != applied) {
    applied = confidence;
    PostProcess_PixelKitYOLOX(confidence, SHAPES_NMS_THRESHOLD, SHAPES_CLASSES, SHAPES_NUM_CLASSES);
  }

  Vision.run();

  static uint32_t lastReport = 0;
  if (millis() - lastReport < 1000) return;
  lastReport = millis();

  // The boxes of the last frame: centre and size as fractions of the model's
  // picture, a score and a class.
  const od_pp_outBuffer_t *boxes = nullptr;
  const int n = PostProcess_ZooResults(detector, &boxes);

  char line[96];
  int len = snprintf(line, sizeof(line), "[SHAPES]");
  for (int c = 0; c < SHAPES_NUM_CLASSES && len > 0 && len < (int)sizeof(line); c++) {
    int found = 0;
    for (int i = 0; i < n; i++) {
      if (boxes[i].class_index == c) found++;
    }
    len += snprintf(line + len, sizeof(line) - (size_t)len, " %d %s", found, SHAPES_CLASSES[c]);
  }
  Serial.println(line);
  vnd_meta_publish_debug_text(line);
}
