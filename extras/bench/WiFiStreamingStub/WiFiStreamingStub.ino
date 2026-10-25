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
 * WiFiStreamingStub
 *
 * Streams the camera over WiFi. Vision.begin() hosts the "NeuroN6-AP" SoftAP
 * from the setSoftAP() settings, with the board's own password.
 *
 * Hardware: Neuro N6 with the MAYA-W476 wireless module and an OV5640 camera
 * on the TFT carrier.
 *
 * This example code is licensed under AGPL-3.0. See the notice above.
 */

#include <NeuroN6.h>
#include <MayaW476.h>
#include <OV5640_Arduino.h>
#include <PostProcess.h>
#pragma neuron6 model="yolov8n_256_quant_pc_ii_seg_coco-st.tflite" name=yolov8_iseg

OV5640 camera(CARRIER_TFT);
NEURON6_DECLARE_MODEL(yolov8_iseg);

void setup() {
  Serial.begin(115200);
  uint32_t serial_wait = millis();
  while (!Serial && (millis() - serial_wait) < 3000) {
    delay(10);
  }
  Serial.println("Neuro N6 WiFi streaming starting...");

  WifiSoftApConfig ap;
  ap.ssid     = "NeuroN6-AP";
  ap.psk      = nullptr;                     /* the board's own password */
  Serial.print("SoftAP \"NeuroN6-AP\" password: ");
  Serial.println(MayaW476.devicePassword());
  ap.security = WIFI_SECURITY_WPA2;
  ap.channel  = 6;
  ap.hidden   = false;
  MayaW476.setSoftAP(ap);

  MayaW476.setTcpPort(5000);

  VisionConfig cfg;
  cfg.transport = TRANSPORT_WIFI_SOFTAP;
  // cfg.transport = TRANSPORT_USB;

  Vision.begin(&NN_Instance_yolov8_iseg,
               PostProcess_YOLOv8_ISEG(0.4f, 0.5f),
               cfg);

  Serial.println("Vision.begin returned. If SoftAP is up, connect to NeuroN6-AP.");
}

void loop() {
  Vision.run();
}
