/*
  Global shutter detection

  Runs the Nia model on the global-shutter camera's 88 frames per second and
  streams a labelled box for each object it finds, from 80 classes, to Neuro
  Studio over USB. Once a second it reports how many frames the model ran on
  and the object it is surest of.

  Hardware: Neuro N6 with a VD66GY camera on the Neuro Vision carrier.

  This example code is in the public domain. The Nia model it embeds is not:
  it is licensed under the Ohm Lab Model Licence
  (models/LICENSE-OhmLab-Model.txt), and a firmware image built from this
  sketch contains the model and may be run only on Ohm Lab hardware.
*/

#include <VD66GY_Arduino.h>
#include <PostProcess.h>

#pragma neuron6 model="nia_1_int8.onnx" name=nia

// VD66GY camera(VD66GY_CARRIER_THERMAL); on the Thermal carrier.
VD66GY camera;

NEURON6_DECLARE_MODEL(nia);

void setup() {
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera      = CAMERA_VGA;              // 88 frames per second
  cfg.isp_enabled = VD66_isp_tuning_auto;    // automatic white balance
  cfg.transport   = TRANSPORT_USB;

  // The model's input size. ASPECT_STRETCH scales the whole frame to it.
  cfg.nn.width  = 256;
  cfg.nn.height = 256;
  cfg.nn.aspect = ASPECT_STRETCH;

  // Confidence threshold, then an overlap threshold this model ignores.
  Vision.begin(&NN_Instance_nia, PostProcess_NIA(0.25f, 0.60f), cfg);
}

void loop() {
  Vision.run();

  static uint32_t passes = 0, lastReport = 0;
  passes++;
  if (millis() - lastReport < 1000) return;
  lastReport = millis();

  // nia_pp_output holds the boxes of the last pass, centre and size from 0 to
  // 1 across the model's input.
  int best = -1;
  for (int i = 0; i < (int)nia_pp_output.nb_detect; i++) {
    if (best < 0 || nia_pp_output.pOutBuff[i].conf > nia_pp_output.pOutBuff[best].conf) best = i;
  }

  char line[96];
  if (best >= 0) {
    const nia_det_t &d = nia_pp_output.pOutBuff[best];
    const char *name = (d.class_index >= 0 && d.class_index < NB_CLASSES)
                         ? classes_table[d.class_index] : "?";
    snprintf(line, sizeof(line), "[DETECT] %lu passes/s, %d objects, best %s %.2f at %.2f,%.2f",
             (unsigned long)passes, (int)nia_pp_output.nb_detect, name,
             (double)d.conf, (double)d.x_center, (double)d.y_center);
  } else {
    snprintf(line, sizeof(line), "[DETECT] %lu passes/s, no objects", (unsigned long)passes);
  }
  passes = 0;
  Serial.println(line);
  vnd_meta_publish_debug_text(line);
}
