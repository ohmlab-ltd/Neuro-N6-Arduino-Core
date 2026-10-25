/*
  Object detection

  Runs the Nia model on the camera and streams a labelled box for each object
  it finds, from 80 classes, to Neuro Studio over USB.

  Hardware: Neuro N6 with an OV5640 camera.

  This example code is in the public domain. The Nia model it embeds is not:
  it is licensed under the Ohm Lab Model Licence
  (models/LICENSE-OhmLab-Model.txt), and a firmware image built from this
  sketch contains the model and may be run only on Ohm Lab hardware.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>

#pragma neuron6 model="nia_1_int8.onnx" name=nia

// OV5640 camera(CARRIER_TFT); on the TFT carrier.
OV5640 camera;

NEURON6_DECLARE_MODEL(nia);

IPostProcessor *nia_pp = nullptr;

void setup() {
  VisionConfig cfg;

  cfg.camera = CAMERA_VGA;
  cfg.fps = FPS_45;

  // The model's input size.
  cfg.nn.width  = 256;
  cfg.nn.height = 256;

  // TRANSPORT_LCD draws on the TFT carrier's display instead.
  cfg.transport = TRANSPORT_USB;

  // ASPECT_STRETCH scales the whole frame to the input. ASPECT_CROP_CENTER
  // takes a centred square.
  cfg.nn.aspect = ASPECT_STRETCH;

  // Confidence threshold, then an overlap threshold this model ignores.
  nia_pp = PostProcess_NIA_OnSlot(0.25f, 0.60f, VND_META_SLOT_MODEL_0);

  Vision.begin(&NN_Instance_nia, nia_pp, cfg);
}

void loop() {
  Vision.run();
}
