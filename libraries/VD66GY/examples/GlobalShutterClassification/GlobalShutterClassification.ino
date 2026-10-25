/*
  Global shutter classification

  Classifies the global-shutter camera's picture into the thousand categories
  of ImageNet and streams the five most likely, with a score for each, to
  Neuro Studio over USB. A moving object keeps its shape: the whole frame is
  exposed at one instant. Once a second the sketch reports how many frames
  the model ran on and the most likely class.

  Hardware: Neuro N6 with a VD66GY camera on the Neuro Vision carrier.

  This example code is in the public domain. The model it embeds,
  mobilenetv2_a035_128_int8.tflite, comes from ST's model zoo and is licensed
  under Apache-2.0; see models/README.md.
*/

#include <VD66GY_Arduino.h>
#include <PostProcess.h>

#include "imagenet_labels.h"

#pragma neuron6 model="mobilenetv2_a035_128_int8.tflite" name=classifier

// VD66GY camera(VD66GY_CARRIER_THERMAL); on the Thermal carrier.
VD66GY camera;

NEURON6_DECLARE_MODEL(classifier);

IPostProcessor *classifier_pp = nullptr;   // the classifier
static IPostProcessor counted_pp;          // the same, counting the model's runs
static uint32_t passes = 0;                // model runs since the last report

static int32_t countedRun(void *ctx, NN_Instance_TypeDef *model) {
  passes++;
  return classifier_pp->run(ctx, model);
}

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

  // The five most likely classes, ignoring any scoring below 20%.
  classifier_pp = PostProcess_Classifier(0.2f, imagenet_labels, IMAGENET_CLASSES);
  counted_pp = *classifier_pp;
  counted_pp.run = countedRun;

  Vision.begin(&NN_Instance_classifier, &counted_pp, cfg);
}

void loop() {
  Vision.run();

  static uint32_t lastReport = 0;
  if (millis() - lastReport < 1000) return;
  lastReport = millis();

  // The best class's position in imagenet_labels, or -1 when nothing reached
  // the threshold.
  float score = 0.0f;
  const int best = PostProcess_ClassifierTop(classifier_pp, &score);

  char line[96];
  if (best >= 0) {
    const char *name = (best < IMAGENET_CLASSES) ? imagenet_labels[best] : "?";
    snprintf(line, sizeof(line), "[CLASSIFY] %lu passes/s, best %s %.2f",
             (unsigned long)passes, name, (double)score);
  } else {
    snprintf(line, sizeof(line), "[CLASSIFY] %lu passes/s, nothing above the threshold",
             (unsigned long)passes);
  }
  passes = 0;
  Serial.println(line);
  vnd_meta_publish_debug_text(line);
}
