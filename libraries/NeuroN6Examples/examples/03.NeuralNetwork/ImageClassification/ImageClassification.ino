/*
  Image classification

  Classifies the camera picture into the thousand categories of ImageNet and
  streams the five most likely, with a score for each, to Neuro Studio over
  USB.

  Hardware: Neuro N6 with an OV5640 camera.

  This example code is in the public domain. The model it embeds,
  mobilenetv2_a035_128_int8.tflite, comes from ST's model zoo and is licensed
  under Apache-2.0; see models/README.md.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>

#include "imagenet_labels.h"

#pragma neuron6 model="mobilenetv2_a035_128_int8.tflite" name=classifier

// OV5640 camera(CARRIER_TFT); on the TFT carrier.
OV5640 camera;

NEURON6_DECLARE_MODEL(classifier);

IPostProcessor *classifier_pp = nullptr;

void setup() {
  VisionConfig cfg;

  cfg.camera = CAMERA_VGA;
  cfg.fps = FPS_45;

  // The model's input size.
  cfg.nn.width  = 128;
  cfg.nn.height = 128;

  // TRANSPORT_LCD draws on the TFT carrier's display instead.
  cfg.transport = TRANSPORT_USB;

  // A centred square. ASPECT_STRETCH scales the whole frame to the input.
  cfg.nn.aspect = ASPECT_CROP_CENTER;

  // The five most likely classes, ignoring any scoring below 20%.
  classifier_pp = PostProcess_Classifier(0.2f, imagenet_labels, IMAGENET_CLASSES);

  Vision.begin(&NN_Instance_classifier, classifier_pp, cfg);
}

void loop() {
  Vision.run();

  // The best class's position in imagenet_labels, or -1 when nothing reached
  // the threshold.
  float score = 0.0f;
  const int best = PostProcess_ClassifierTop(classifier_pp, &score);
  if (best >= 0 && score > 0.9f) {
    // A confident result: act on imagenet_labels[best] here.
  }
}
