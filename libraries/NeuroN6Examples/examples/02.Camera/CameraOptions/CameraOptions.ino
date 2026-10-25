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
  Camera options

  Runs object segmentation with the main camera and neural network settings
  written out, and shows the result on the TFT carrier's display.

  Hardware: Neuro N6 with an OV5640 camera on the TFT carrier.

  This example code is licensed under AGPL-3.0. See the notice above.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>

#pragma neuron6 model="yolov8n_256_quant_pc_ii_seg_coco-st.tflite" name=yolov8_iseg

// OV5640 camera; on the Neuro Vision carrier.
OV5640 camera(CARRIER_TFT);
NEURON6_DECLARE_MODEL(yolov8_iseg);

void setup() {
  VisionConfig cfg;

  //   CAMERA_WVGA  800x480  fills the display
  //   CAMERA_VGA   640x480  centred on the display
  cfg.camera = CAMERA_WVGA;

  cfg.fps    = FPS_15;

  //   CAMERA_AUTO_FLIP        upright on either carrier
  //   CAMERA_MIRROR_FLIP_NONE the sensor's own orientation
  //   CAMERA_FLIP             upside down
  //   CAMERA_MIRROR           left and right swapped
  //   CAMERA_MIRROR_FLIP      both
  cfg.mirror_flip = CAMERA_AUTO_FLIP;

  // The model's input size.
  cfg.nn.width  = 256;
  cfg.nn.height = 256;

  //   ASPECT_STRETCH      the whole frame, scaled to the input
  //   ASPECT_CROP_CENTER  the largest centred square
  //   ASPECT_CROP_OFFSET  the crop set below
  //   ASPECT_LETTERBOX    the whole frame, centred on grey bars
  cfg.nn.aspect = ASPECT_STRETCH;

  // The crop area, in camera pixels, used only with ASPECT_CROP_OFFSET.
  cfg.nn.crop_x = 14;
  cfg.nn.crop_y = 54;
  cfg.nn.crop_w = 600;  // 0 means no crop
  cfg.nn.crop_h = 426;

  //   TRANSPORT_USB           USB to Neuro Studio
  //   TRANSPORT_WIFI_SOFTAP   WiFi as an access point, needs the WiFi library
  //   TRANSPORT_LCD           the TFT carrier's display
  cfg.transport = TRANSPORT_LCD;

  const float conf_threshold = 0.4f;  // score a detection needs to be kept
  const float iou_threshold  = 0.5f;  // overlap at which two boxes are merged

  Vision.begin(&NN_Instance_yolov8_iseg,
               PostProcess_YOLOv8_ISEG(conf_threshold, iou_threshold),
               cfg);
}

void loop() {
  Vision.run();
}
