# Bundled models

Models the examples load through `#pragma neuron6 model="..."`. The pragma
searches the sketch folder, then `<sketch>/models/`, then this folder.

What each model costs on the board is in [CATALOG.md](CATALOG.md). Each
model has a `.license` file beside it that names its copyright holders and
licence.

| File | Network | Origin | Licence |
|---|---|---|---|
| `nia_1_int8.onnx` | Nia detector, COCO-80, 256x256 | Ohm Lab | Ohm Lab Model Licence, see the notice below |
| `yolov8n_256_quant_pc_ii_seg_coco-st.tflite` | YOLOv8n instance segmentation, 256x256 | ST Edge AI model zoo, YOLOv8 derived | AGPL-3.0, see the notice below |
| `yolov8n_256_quant_pc_uf_pose_coco-st.tflite` | YOLOv8n pose, 256x256 | ST Edge AI model zoo, YOLOv8 derived | AGPL-3.0, see the notice below |
| `aed_yamnet_int8.onnx` | Audio event detection, ESC-10, YAMNet derived | ST's STM32N6-GettingStarted-Audio package, `Projects/X-CUBE-AI/models/yamnet_1024_64x96_tl_qdq_int8.onnx`, unmodified | ST SLA0044, see `LICENSE-SLA0044.md` |
| `centerface-hwc.tflite` | CenterFace face detector, 128x128 | Star-Clouds/CenterFace, `models/onnx/centerface.onnx`, converted to int8 TFLite by Ohm Lab, its input then moved past the transpose by `tools/tflite_input_to_hwc.py` | MIT, see `LICENSE-MIT-CenterFace.txt` |
| `face4-int8.onnx` | MobileFaceNet embedder, 128-d | foamliu/MobileFaceNet, release v1.0 `mobilefacenet.pt` (2019, foamliu), exported to ONNX and quantised by Ohm Lab | Apache-2.0, see `face4-int8.onnx.license` and `LICENSE-Apache-2.0.txt`. Trained on data for non-commercial research only, see the notice below |
| `033_palm_detection_full_quant_pc_uf_od.tflite` | MediaPipe palm detection | Google MediaPipe, ST Edge AI packaging | Apache-2.0, see `LICENSE-Apache-2.0.txt` |
| `033_hand_landmark_full_quant_pc_uf_handl.tflite` | MediaPipe hand landmarks | Google MediaPipe, ST Edge AI packaging | Apache-2.0, see `LICENSE-Apache-2.0.txt` |
| `mobilenetv2_a035_128_int8.tflite` | MobileNetV2 image classification, ImageNet-1000, 128x128 | ST Edge AI model zoo, `image_classification/mobilenetv2/Public_pretrainedmodel_public_dataset/ImageNet/mobilenetv2_a035_128` | Apache-2.0, see `LICENSE-Apache-2.0.txt` |
| `pixelkit_shapes_int8.onnx` | YOLOX-Nano detector, circles and squares, 256x256 | Trained by Ohm Lab in PixelKit from Megvii's YOLOX-Nano | Apache-2.0, see `pixelkit_shapes_int8.onnx.license` and `LICENSE-Apache-2.0.txt` |

## Class names

| Classes | Source |
|---|---|
| COCO-80 | `libraries/PostProcess/src/classes_table_default.c`, a weak table a sketch can replace with its own `classes_table` |
| ImageNet-1000 | `imagenet_labels.h` in the `ImageClassification` and `GlobalShutterClassification` examples |
| Circles and squares | `shapes_labels.h` in the `PixelKitDetector` examples |
| Other | A list passed to the `_Ex` form of the post-processor |

## Licensing notice: the face models

`FaceRecognition` embeds `centerface-hwc.tflite` and `face4-int8.onnx`, and
`GlobalShutterFaceDetection` embeds `centerface-hwc.tflite`. Each matches its
upstream weights layer by layer, apart from int8 rounding.

- `face4-int8.onnx` is foamliu's MobileFaceNet, Copyright 2019 foamliu,
  which its author publishes under Apache-2.0. Ohm Lab's change: export to
  ONNX and int8 quantisation. `face4-int8.onnx.license` carries these lines.
  Its author trained it on InsightFace's refined MS-Celeb-1M,
  `faces_ms1m_112x112`. InsightFace provides that data, and
  models trained on it, for non-commercial research purposes only, and
  Microsoft withdrew MS-Celeb-1M in 2019. Ohm Lab has not established whether
  those terms bind the weights. Take legal advice before using this model in
  a product, or replace it.
- `centerface-hwc.tflite` is Star-Clouds' CenterFace, under MIT. It was trained
  on the WIDER FACE training set, whose terms are not part of that licence.

## Licensing notice: the Nia model

`nia_1_int8.onnx` is Ohm Lab's own network. It is licensed under the Ohm Lab
Model Licence in `LICENSE-OhmLab-Model.txt` in this directory, not under the
MIT licence that governs Ohm Lab's code.

- **Ohm Lab hardware only.** The model, and any model derived from it by
  training, fine-tuning, quantising or conversion, may be used, copied,
  modified and redistributed, but executed only on Ohm Lab hardware: the
  Neuro N6 board and the Neuro Vision modules and carriers. Training,
  fine-tuning and conversion may take place on any computer, for that purpose.
- **Every form.** The licence covers the model in every form: the `.onnx`
  file, the C source and weights that `#pragma neuron6 model="nia_1_int8.onnx"`
  generates at build time, and the weights written to a board.
- **Other hardware.** The model was made specifically for the instruction
  sets of the STM32N6 and of the Arm Ethos-U55 and Ethos-U65 neural
  processing units. Running it on any hardware that is not Ohm Lab's,
  whatever its processor, needs a separate licence: contact Ohm Lab at
  team@ohmlab.co.uk.
- **The sketches.** `ObjectDetection`, `ObjectDetectionDisplay`,
  `GlobalShutterDetection` and `ModelBench` embed the model. Their own code
  keeps its terms. A firmware image built from them contains the model and is
  bound by this licence.
- **Attribution and warranty.** A redistributed derived model must say that it
  derives from Nia and name Ohm Lab as its author. The model comes without
  warranty, and the licence grants no patent rights beyond the use it permits.

## Licensing notice: YOLOv8 model files

The files `yolov8n_256_quant_pc_ii_seg_coco-st.tflite` and
`yolov8n_256_quant_pc_uf_pose_coco-st.tflite` (the "Model Files") derive from
the YOLOv8 neural networks published by Ultralytics Inc. and are licensed under
the GNU Affero General Public License, version 3.0 (AGPL-3.0). The full text of
that licence is provided in `LICENSE-AGPL-3.0.txt` in this directory. The Model
Files, and the sketches enumerated below, are not licensed under the MIT licence
that governs the remainder of this repository.

1. **Scope.** AGPL-3.0 applies to the Model Files and to any work based on
   them, including any firmware image, and any product, that incorporates
   either file. The sketches enumerated below are licensed under AGPL-3.0
   because each is designed to embed one of the Model Files and is
   distributed together with it.
2. **Separate works.** The remainder of this repository, including the Ohm Lab
   core, the bundled libraries and the other examples, constitutes separate
   works distributed alongside the Model Files on the same medium. Ohm Lab does
   not license any other part of this repository under AGPL-3.0, and nothing
   in this notice extends AGPL-3.0 to those works.
3. **Obligations on distribution.** Any party who conveys a firmware image, a
   product, or any other work that incorporates either Model File must comply
   with AGPL-3.0 in full, including the obligation to make the Corresponding
   Source of the entire work available under the terms of that licence and,
   where the work interacts with users over a network, the obligation under
   section 13 to offer the Corresponding Source to those users. Building and
   flashing a sketch for personal or internal use does not by itself give rise
   to those obligations.
4. **STMicroelectronics software terms.** Firmware built for the Neuro N6
   executes the Model Files by means of software components supplied by
   STMicroelectronics under licence SLA0044, including the ST Edge AI runtime
   and the post-processing libraries contained in this repository. SLA0044
   provides that the ST software may not be used, reproduced or redistributed
   in any manner that would subject it to open source terms. STMicroelectronics
   has further stated, in the README accompanying its distribution of the
   Ultralytics software, that combining software provided under AGPL-3.0 with
   ST software delivered under SLA0044 produces a combined software whose
   distribution or non-internal use is, in most cases, forbidden by the ST
   software terms. Accordingly, it is Ohm Lab's understanding that a firmware
   image containing either Model File together with those ST components may be
   built and used for personal or internal purposes, but may not be
   distributed by any party while AGPL-3.0 governs the Model File.
5. **Commercial licensing.** Ultralytics Inc. offers an Enterprise License
   under which YOLOv8 models may be incorporated into commercial products
   free of the obligations of AGPL-3.0. A party holding such a licence is not
   subject to the open source terms referred to in paragraph 4 in respect of
   the Model Files. Ohm Lab is not a party to, and does not resell, that
   licence.
6. **Provenance and Corresponding Source.** Ohm Lab obtained the Model Files
   from the STMicroelectronics distribution of the Ultralytics software at
   https://github.com/stm32-hotspot/ultralytics, in the directory
   `examples/YOLOv8-STEdgeAI/stedgeai_models/`, and has not modified them.
   As at commit `be7d885c94ed`, the copies in this repository are
   byte-for-byte identical to the files
   `examples/YOLOv8-STEdgeAI/stedgeai_models/segmentation/yolov8n_256_quant_pc_ii_seg_coco-st.tflite` and
   `examples/YOLOv8-STEdgeAI/stedgeai_models/pose_estimation/yolov8n_256_quant_pc_uf_pose_coco-st.tflite` in that
   distribution (SHA-256 `3316733125a5fb85d66ed33993627b5d...` and
   `e2666d331d5e8f55809f83dd32549960...`). The scripts and
   instructions by which those files were produced from the Ultralytics
   checkpoints are published in the same directory, and the training code
   and original checkpoints at https://github.com/ultralytics/ultralytics.
   Those locations constitute the Corresponding Source for the Model Files
   for the purposes of AGPL-3.0. The Model Files are Copyright (c)
   Ultralytics Inc. Ultralytics and YOLO are trademarks of Ultralytics Inc.,
   used here solely to identify the origin of the Model Files.
7. **No warranty; no legal advice.** The Model Files are provided without
   warranty of any kind, as set out in AGPL-3.0. This notice records Ohm Lab's
   understanding of the applicable terms for the convenience of users. It does
   not constitute legal advice. Each user is responsible for its own compliance
   and should obtain independent legal advice before distributing any work
   that incorporates the Model Files.

Sketches licensed under AGPL-3.0, each of which carries this notice in its
header:

- `libraries/NeuroN6Examples/examples/02.Camera/CameraOptions`
- `libraries/NeuroN6Examples/examples/02.Camera/CameraResolution`
- `libraries/NeuroN6Examples/examples/03.NeuralNetwork/CascadedModels`
- `libraries/NeuroN6Examples/examples/03.NeuralNetwork/ObjectSegmentation`
- `libraries/NeuroN6Examples/examples/03.NeuralNetwork/ObjectSegmentationDisplay`
- `libraries/NeuroN6Examples/examples/03.NeuralNetwork/PoseEstimation`
- `libraries/MayaW476/examples/WiFiAccessPoint`
- `libraries/MayaW476/examples/WiFiVideoStream`
- `libraries/N6Events/examples/EventRecording`
- `libraries/VD66GY/examples/GlobalShutterThermal`
- `libraries/VD66GY/examples/GlobalShutterVision`
- `extras/bench/WiFiStreamingStub`
