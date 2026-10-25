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
  WiFi video stream

  Streams camera video and object segmentation results to Neuro Studio over
  WiFi. The board hosts its own access point.

  Hardware: Neuro N6 with the MAYA-W476 wireless module and an OV5640 camera
  on the Neuro Vision carrier.

  This example code is licensed under AGPL-3.0. See the notice above.
*/

#include <MayaW476.h>
#include <OV5640_Arduino.h>
#include <PostProcess.h>
#pragma neuron6 model="yolov8n_256_quant_pc_ii_seg_coco-st.tflite" name=yolov8_iseg

// OV5640 camera(CARRIER_TFT); on the TFT carrier.
OV5640 camera;
NEURON6_DECLARE_MODEL(yolov8_iseg);

// The network the board hosts, with the board's own password.
const char *AP_NAME = "NeuroN6-Cam";

void setup() {
  Serial.begin(115200);
  MayaW476.setTimeout(10000);   // ms, default 7000
  MayaW476.setTcpPort(5000);    // Neuro Studio's expected port, the default

  // The first run stores the board's password and restarts the board once.
  MayaW476.beginAP(AP_NAME);

  // The stream runs on the board's own access point only.
  VisionConfig cfg;
  cfg.transport = TRANSPORT_WIFI_SOFTAP;

  Vision.begin(&NN_Instance_yolov8_iseg,
               PostProcess_YOLOv8_ISEG(0.4f, 0.5f),
               cfg);
}

void loop() {
  Vision.run();

  // How to join, once each time a Serial Monitor connects, over USB only.
  static bool shown = false;
  if (!Serial) {
    shown = false;
  } else if (!shown) {
    Serial.print("Join WiFi network ");
    Serial.print(AP_NAME);
    Serial.print(" with password ");
    Serial.println(MayaW476.devicePassword());
    shown = true;
  }
}
