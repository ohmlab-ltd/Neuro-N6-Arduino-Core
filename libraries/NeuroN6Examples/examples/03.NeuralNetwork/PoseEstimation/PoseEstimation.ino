/*
  LICENSING NOTICE

  This example sketch embeds the YOLOv8 model file
  yolov8n_256_quant_pc_uf_pose_coco-st.tflite,
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
  Pose estimation

  Finds people in the camera picture and returns 17 body points for each of
  them, from nose and eyes down to ankles. The results stream to Neuro Studio
  over USB.

  Hardware: Neuro N6 with an OV5640 camera.

  This example code is licensed under AGPL-3.0. See the notice above.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>

#pragma neuron6 model="yolov8n_256_quant_pc_uf_pose_coco-st.tflite" name=yolov8_mpe

// Confidence and overlap thresholds.
static const float MPE_CONF = 0.40f;
static const float MPE_IOU  = 0.50f;

// Print interval.
static const uint32_t HEARTBEAT_MS = 1000;

// OV5640 camera(CARRIER_TFT); on the TFT carrier.
OV5640 camera;
NEURON6_DECLARE_MODEL(yolov8_mpe);

void setup() {
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera      = CAMERA_VGA;
  cfg.fps         = FPS_30;
  cfg.mirror_flip = CAMERA_AUTO_FLIP;

  cfg.nn.width    = 256;               // the model's input size
  cfg.nn.height   = 256;
  // A centred square. ASPECT_STRETCH scales the whole frame to the input.
  cfg.nn.aspect   = ASPECT_CROP_CENTER;

  cfg.transport   = TRANSPORT_USB;
  cfg.af          = AF_CONTINUOUS;

  // Neuro Studio looks for the skeleton in the first results slot.
  Vision.begin(&NN_Instance_yolov8_mpe,
               PostProcess_YOLOv8_MPE_OnSlot(MPE_CONF, MPE_IOU,
                                             VND_META_SLOT_MODEL_0),
               cfg);

  Serial.println("=== PoseEstimation: pose estimation -> Neuro Studio ===");
}

void loop() {
  Vision.run();

  // The body points are numbered 0 nose, 1 and 2 eyes, 3 and 4 ears,
  // 5 and 6 shoulders, 7 and 8 elbows, 9 and 10 wrists, 11 and 12 hips,
  // 13 and 14 knees, 15 and 16 ankles. Each carries an x, a y from 0 to 1
  // across the picture, and its own confidence.
  static uint32_t lastBeat = 0;
  const uint32_t now = millis();
  if (now - lastBeat >= HEARTBEAT_MS) {
    lastBeat = now;
    const int n = (int)mpe_pp_output.nb_detect;
    Serial.print("[mpe] people=");
    Serial.print(n);
    if (n > 0 && mpe_pp_output.pOutBuff[0].pKeyPoints != nullptr) {
      const mpe_pp_keyPoints_t *nose = &mpe_pp_output.pOutBuff[0].pKeyPoints[0];
      Serial.print("  conf=");  Serial.print(mpe_pp_output.pOutBuff[0].conf, 2);
      Serial.print("  nose=");  Serial.print(nose->x, 2);
      Serial.print(",");        Serial.print(nose->y, 2);
      Serial.print(" (");       Serial.print(nose->conf, 2);
      Serial.print(")");
    }
    Serial.println();
  }
}
