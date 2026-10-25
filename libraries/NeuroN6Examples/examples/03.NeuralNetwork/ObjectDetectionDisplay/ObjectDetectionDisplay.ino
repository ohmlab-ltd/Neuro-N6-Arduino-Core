/*
  Object detection display

  Runs the Nia model on the camera and draws a labelled box for each object
  it finds, from 80 classes, on the TFT carrier's display.

  Hardware: Neuro N6 with an OV5640 camera on the TFT carrier.

  This example code is in the public domain. The Nia model it embeds is not:
  it is licensed under the Ohm Lab Model Licence
  (models/LICENSE-OhmLab-Model.txt), and a firmware image built from this
  sketch contains the model and may be run only on Ohm Lab hardware.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>

#pragma neuron6 model="nia_1_int8.onnx" name=nia

OV5640 camera(CARRIER_TFT);
NEURON6_DECLARE_MODEL(nia);

void setup() {
  VisionConfig cfg;
  cfg.camera      = CAMERA_WVGA;   // 800x480 fills the display exactly
  cfg.fps         = FPS_60;        // also sets how often the display refreshes
  cfg.transport   = TRANSPORT_LCD; // or TRANSPORT_USB for Neuro Studio
  cfg.nn.width    = 256;           // the model's input size
  cfg.nn.height   = 256;
  cfg.nn.aspect   = ASPECT_STRETCH; // or ASPECT_CROP_CENTER for a centred square

  // Confidence threshold, then an overlap threshold this model ignores.
  Vision.begin(&NN_Instance_nia,
               PostProcess_NIA(0.4f, 0.60f),
               cfg);
}

void loop() {
  // The hardware keeps the live picture moving without this loop.
  Vision.run();
}
