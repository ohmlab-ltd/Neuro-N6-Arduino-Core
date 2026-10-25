/*
  PixelKit detector display

  Runs a detector trained in PixelKit and draws a labelled box around each
  object on the TFT carrier's display. The sample model finds printed circles
  and squares.

  Hardware: Neuro N6 with an OV5640 camera on the TFT carrier.

  To test it, print pixelkit_shapes_test_sheet.pdf from the PixelKitDetector
  example at 100% on A4 and hold a page so that it fills the camera's height.
  Pages 1 to 3 hold 4, 6 and 8 shapes; page 4 holds none.

  This example code is in the public domain. The model it embeds,
  pixelkit_shapes_int8.onnx, is licensed under Apache-2.0: it was fine-tuned
  from Megvii's YOLOX-Nano. See models/README.md.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>

#include "shapes_labels.h"   // class names, input size and thresholds

#pragma neuron6 model="pixelkit_shapes_int8.onnx" name=shapes

OV5640 camera(CARRIER_TFT);
NEURON6_DECLARE_MODEL(shapes);

void setup() {
  VisionConfig cfg;
  cfg.camera    = CAMERA_WVGA;          // 800x480 fills the display
  cfg.fps       = FPS_60;
  cfg.nn.width  = SHAPES_INPUT_W;
  cfg.nn.height = SHAPES_INPUT_H;
  cfg.nn.aspect = ASPECT_LETTERBOX;     // the whole picture on grey, as PixelKit trains
  cfg.transport = TRANSPORT_LCD;

  Vision.begin(&NN_Instance_shapes,
               PostProcess_PixelKitYOLOX(SHAPES_CONF_THRESHOLD, SHAPES_NMS_THRESHOLD,
                                         SHAPES_CLASSES, SHAPES_NUM_CLASSES),
               cfg);
}

void loop() {
  Vision.run();
}
