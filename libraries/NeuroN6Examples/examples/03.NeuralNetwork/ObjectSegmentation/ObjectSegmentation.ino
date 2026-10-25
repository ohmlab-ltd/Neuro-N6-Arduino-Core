/*
  LICENSING NOTICE

  This example sketch embeds the YOLOv8 model file
  yolov8n_256_quant_pc_ii_seg_coco-st.tflite,
  which is licensed under the GNU Affero General Public License, version 3.0
  (AGPL-3.0), and not under the MIT licence that governs the remainder of
  this repository. Accordingly, this sketch is licensed under AGPL-3.0.

  A firmware image built from this sketch is a work based on that model.
  Any party who distributes such an image, or a product containing it,
  must comply with AGPL-3.0 in full, including the provision of
  Corresponding Source, or must hold an Ultralytics Enterprise License.
  Building and flashing the sketch for personal or internal use does not
  by itself trigger those obligations.

  The STMicroelectronics components that execute the model are supplied
  under ST licence SLA0044, whose terms in most cases forbid the
  distribution of firmware that combines them with AGPL-licensed
  software.

  The full notice and the licence text are in models/README.md and
  models/LICENSE-AGPL-3.0.txt.
*/

/*
  Object segmentation

  Finds objects in the camera picture, outlines the shape of each one, and
  streams the results to Neuro Studio over USB.

  Hardware: Neuro N6 with an OV5640 camera.

  This example code is licensed under AGPL-3.0. See the notice above.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>

#pragma neuron6 model="yolov8n_256_quant_pc_ii_seg_coco-st.tflite" name=yolov8_iseg

// OV5640 camera(CARRIER_TFT); on the TFT carrier.
OV5640 camera;
NEURON6_DECLARE_MODEL(yolov8_iseg);

void setup() {

  VisionConfig cfg;
  cfg.camera    = CAMERA_QSXGA;    // the sensor's full 5 megapixels
  cfg.fps       = FPS_DEFAULT;     // 15 fps, the most QSXGA runs at
  cfg.transport = TRANSPORT_USB;
  cfg.strobe    = STROBE_ENABLED;  // pulses the sensor's strobe pin with each exposure

  // Confidence threshold, then overlap threshold.
  Vision.begin(&NN_Instance_yolov8_iseg,
               PostProcess_YOLOv8_ISEG(0.4f, 0.5f),
               cfg);
}

void loop() {
  Vision.run();
}
