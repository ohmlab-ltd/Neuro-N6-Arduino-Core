/*
  LICENSING NOTICE

  This example sketch embeds the YOLOv8 model files
  yolov8n_256_quant_pc_ii_seg_coco-st.tflite and
  yolov8n_256_quant_pc_uf_pose_coco-st.tflite,
  which are licensed under the GNU Affero General Public License, version 3.0
  (AGPL-3.0), and not under the MIT licence that governs the remainder of
  this repository. Accordingly, this sketch is licensed under AGPL-3.0.

  A firmware image built from this sketch is a work based on those models.
  Any party who distributes such an image, or a product containing it,
  must comply with AGPL-3.0 in full, including the provision of
  Corresponding Source, or must hold an Ultralytics Enterprise License.
  Building and flashing the sketch for personal or internal use does not
  by itself trigger those obligations.

  The STMicroelectronics components that execute the models are supplied
  under ST licence SLA0044, whose terms in most cases forbid the
  distribution of firmware that combines them with AGPL-licensed
  software.

  The full notice and the licence text are in models/README.md and
  models/LICENSE-AGPL-3.0.txt.
*/

/*
  Cascaded models

  Runs object segmentation and pose estimation one after the other and
  streams both results to Neuro Studio over USB.

  Hardware: Neuro N6 with an OV5640 camera.

  This example code is licensed under AGPL-3.0. See the notice above.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>

#pragma neuron6 model="yolov8n_256_quant_pc_ii_seg_coco-st.tflite"  name=yolov8_iseg
#pragma neuron6 model="yolov8n_256_quant_pc_uf_pose_coco-st.tflite" name=yolov8_mpe

// OV5640 camera(CARRIER_TFT); on the TFT carrier.
OV5640 camera;

NEURON6_DECLARE_MODEL(yolov8_iseg);
NEURON6_DECLARE_MODEL(yolov8_mpe);

IPostProcessor *iseg_pp = nullptr;
IPostProcessor *mpe_pp  = nullptr;

void setup() {
  VisionConfig cfg;

  // 800x480. CAMERA_VGA is 640x480.
  cfg.camera = CAMERA_WVGA;

  cfg.mirror_flip = CAMERA_AUTO_FLIP;

  // The input size. The two models have to agree on it.
  cfg.nn.width  = 256;
  cfg.nn.height = 256;

  //   ASPECT_STRETCH      the whole frame, scaled to the input
  //   ASPECT_CROP_CENTER  a centred square
  //   ASPECT_CROP_OFFSET  the crop below
  //   ASPECT_LETTERBOX    the whole frame, centred on grey bars
  cfg.nn.aspect = ASPECT_STRETCH;
  cfg.nn.crop_x = 0;   cfg.nn.crop_y = 0;
  cfg.nn.crop_w = 0;   cfg.nn.crop_h = 0;

  // TRANSPORT_LCD draws on the TFT carrier's display instead.
  cfg.transport = TRANSPORT_USB;

  // Confidence and overlap thresholds.
  const float iseg_conf = 0.4f, iseg_iou = 0.5f;
  const float mpe_conf  = 0.4f, mpe_iou  = 0.5f;

  // One results slot per model. There are four, VND_META_SLOT_MODEL_0 through
  // VND_META_SLOT_MODEL_3.
  iseg_pp = PostProcess_YOLOv8_ISEG_OnSlot(iseg_conf, iseg_iou,
                                            VND_META_SLOT_MODEL_0);
  mpe_pp  = PostProcess_YOLOv8_MPE_OnSlot (mpe_conf,  mpe_iou,
                                            VND_META_SLOT_MODEL_1);

  Vision.begin(cfg);
}

void loop() {
  // Each call captures a frame and runs one model. OVL_CLEAR clears the
  // drawing and draws without putting it on screen.
  Vision.runWith(&NN_Instance_yolov8_iseg, iseg_pp, OVL_CLEAR);

  // OVL_RELOAD draws on top and puts the picture on screen.
  Vision.runWith(&NN_Instance_yolov8_mpe, mpe_pp, OVL_RELOAD);
}
