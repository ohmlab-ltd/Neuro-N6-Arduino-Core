/*
  PixelKit detector

  Runs a detector trained in PixelKit and streams labelled boxes to Neuro
  Studio, with a slider there for the score a box needs. The sample model
  finds printed circles and squares. A PixelKit export is a sketch like this
  one.

  Hardware: Neuro N6 with an OV5640 camera.

  To test it, print pixelkit_shapes_test_sheet.pdf from this folder at 100%
  on A4 and hold a page so that it fills the camera's height. Pages 1 to 3
  hold 4, 6 and 8 shapes; page 4 holds none.

  This example code and its test sheet are in the public domain. The model
  it embeds, pixelkit_shapes_int8.onnx, is licensed under Apache-2.0: it was
  fine-tuned from Megvii's YOLOX-Nano. See models/README.md.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>

#include "shapes_labels.h"   // class names, input size and thresholds

#pragma neuron6 model="pixelkit_shapes_int8.onnx" name=shapes

// OV5640 camera(CARRIER_TFT); on the TFT carrier.
OV5640 camera;
NEURON6_DECLARE_MODEL(shapes);

IPostProcessor *detector = nullptr;

// The score a box needs, shown as a slider in Neuro Studio.
static float confidence = SHAPES_CONF_THRESHOLD;

void setup() {
  VisionConfig cfg;
  cfg.camera    = CAMERA_VGA;
  cfg.fps       = FPS_45;
  cfg.nn.width  = SHAPES_INPUT_W;
  cfg.nn.height = SHAPES_INPUT_H;
  cfg.nn.aspect = ASPECT_LETTERBOX;     // the whole picture on grey, as PixelKit trains
  cfg.transport = TRANSPORT_USB;

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

  // The boxes, to act on: centre and size as fractions of the model's
  // picture, a score and a class.
  const od_pp_outBuffer_t *boxes = nullptr;
  const int n = PostProcess_ZooResults(detector, &boxes);
  for (int i = 0; i < n; i++) {
    // SHAPES_CLASSES[boxes[i].class_index] found, with score boxes[i].conf
  }
}
