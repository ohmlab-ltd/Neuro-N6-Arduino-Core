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
  WiFi access point

  Hosts a WiFi access point and streams camera video and object segmentation
  results to Neuro Studio over it, while drawing the same picture on the TFT
  carrier's display.

  Hardware: Neuro N6 with the MAYA-W476 wireless module and an OV5640 camera
  on the TFT carrier.

  This example code is licensed under AGPL-3.0. See the notice above.
*/

#include <MayaW476.h>
#include <OV5640_Arduino.h>
#include <PostProcess.h>
#pragma neuron6 model="yolov8n_256_quant_pc_ii_seg_coco-st.tflite" name=yolov8_iseg

OV5640 camera(CARRIER_TFT);
NEURON6_DECLARE_MODEL(yolov8_iseg);

// The network the board hosts, with the board's own password.
const char *AP_NAME = "NeuroN6-Cam";

void setup() {
  Serial.begin(115200);

  // WiFi starts before Vision.begin() routes the video to it.
  MayaW476.setTimeout(15000);
  MayaW476.setTcpPort(5000);              // Neuro Studio's expected port

  // The first run stores the board's password and restarts the board once.
  wl_status_t st = MayaW476.beginAP(AP_NAME);

  if (st == WL_AP_LISTENING || st == WL_AP_CONNECTED) {
    // localIP() is 0.0.0.0 until the access point has its address.
    for (int i = 0; i < 30 && MayaW476.localIP()[0] == 0; i++) {
      delay(100);
    }
  } else {
    Serial.print("SoftAP FAILED, status ");
    Serial.print((int)st);
    Serial.println(" - falling back to USB streaming.");
  }

  VisionConfig cfg;
  cfg.camera      = CAMERA_WVGA;          // 800x480, the panel's size
  // The picture starts to tear at a higher frame rate.
  cfg.fps         = FPS_15;
  cfg.mirror_flip = CAMERA_AUTO_FLIP;

  // The model's input size.
  cfg.nn.width    = 256;
  cfg.nn.height   = 256;
  cfg.nn.aspect   = ASPECT_CROP_CENTER;

  cfg.transport   = TRANSPORT_WIFI_SOFTAP;
  cfg.lcd         = true;
  cfg.af          = AF_CONTINUOUS;

  Vision.begin(&NN_Instance_yolov8_iseg,
               PostProcess_YOLOv8_ISEG(0.4f, 0.5f),
               cfg);
}

// Prints how to join once each time a Serial Monitor connects, over USB only.
void printJoin() {
  static bool shown = false;
  const wl_status_t st = MayaW476.status();
  if (!Serial) {
    shown = false;
  } else if (!shown && (st == WL_AP_LISTENING || st == WL_AP_CONNECTED)) {
    Serial.print("SoftAP up - join \"");
    Serial.print(AP_NAME);
    Serial.print("\" with password ");
    Serial.print(MayaW476.devicePassword());
    Serial.print(", then point Neuro Studio at ");
    Serial.print(MayaW476.ipv4String());
    Serial.println(":5000");
    shown = true;
  }
}

void loop() {
  const uint32_t now = millis();

  Vision.run();
  printJoin();

  static uint32_t s_last_hb = 0;
  if (now - s_last_hb >= 1000u) {
    s_last_hb = now;

    const wl_status_t wst = MayaW476.status();
    const char *wifi_txt;
    switch (wst) {
      case WL_AP_CONNECTED: wifi_txt = "up/client";  break;
      case WL_AP_LISTENING: wifi_txt = "up/idle";    break;
      case WL_AP_FAILED:    wifi_txt = "AP-FAILED";  break;
      case WL_NO_MODULE:    wifi_txt = "NO-MODULE";  break;
      default:              wifi_txt = "not-AP";     break;
    }

    Serial.print("[HB] t=");        Serial.print(now / 1000u);
    Serial.print("s wifi=");        Serial.println(wifi_txt);
  }
}
