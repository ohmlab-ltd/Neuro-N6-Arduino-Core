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
  Global shutter vision

  Streams the global-shutter camera to Neuro Studio and runs YOLOv8 instance
  segmentation on it. The motion, magnetic field and distance readings are
  sent with the video.

  Hardware: Neuro N6 with a VD66GY camera on the Neuro Vision carrier, which
  has the VL53L1CB distance sensor.

  This example code is licensed under AGPL-3.0. See the notice above.
*/

#include <VD66GY_Arduino.h>
#include <PostProcess.h>
#pragma neuron6 model="yolov8n_256_quant_pc_ii_seg_coco-st.tflite" name=yolov8_iseg
#include <Wire.h>
#include <LSM6DSL_Arduino.h>
#include <MMC5603NJ.h>
#include <VL53L1CB_Arduino.h>

// VD66GY camera(VD66GY_CARRIER_THERMAL); on the Thermal carrier.
VD66GY camera;

NEURON6_DECLARE_MODEL(yolov8_iseg);

LSM6DSL   imu;
MMC5603NJ mag;
VL53L1CB  tof;
bool imuOK = false;
bool magOK = false;
bool tofOK = false;

void setup() {
  Serial.begin(115200);

  imuOK = imu.begin();
  magOK = mag.begin();
  tofOK = tof.begin();

  VisionConfig cfg;
  cfg.camera      = CAMERA_VD66_FULL;
  cfg.fps         = FPS_45;
  cfg.mirror_flip = CAMERA_AUTO_FLIP;

  // The model's input, a square from the middle of the frame.
  cfg.nn.width  = 256;
  cfg.nn.height = 256;
  cfg.nn.aspect = ASPECT_CROP_CENTER;

  cfg.transport   = TRANSPORT_USB;      // the VD66GY carriers have no display
  cfg.af          = AF_DISABLED;        // this camera has no autofocus
  cfg.strobe      = STROBE_DISABLED;
  cfg.isp_enabled = VD66_isp_tuning_auto;   // automatic white balance

  Vision.begin(&NN_Instance_yolov8_iseg, PostProcess_YOLOv8_ISEG(0.4f, 0.5f), cfg);
}

void loop() {
  Vision.run();

  if (imuOK) imu.sendNS();
  if (magOK) mag.sendNS();
  if (tofOK) tof.sendNS();
}
