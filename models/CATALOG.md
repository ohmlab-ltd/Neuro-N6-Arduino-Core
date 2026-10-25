# Model catalogue

What each model costs on the board. Licences are in [README.md](README.md).

Figures are from ST Edge AI v4.0.1 with this core's options.

| Column | Meaning |
|---|---|
| Weights | Size in flash, in the 26112 KB pool at `0x90300000` |
| Working memory | Activation memory, from the 1.75 MB NPU RAM pool. Any excess goes to a 14 MB pool in PSRAM |
| NPU passes | Epochs, with the number run on the CPU in brackets |
| Inference | Time per inference with the flash clocked at 200 MHz. `extras/bench/ModelBench` measures it |

## Bundled models

| Model file | Task | Input | Weights | Working memory | NPU passes | Inference | Post-processor | Example |
|---|---|---|---|---|---|---|---|---|
| `nia_1_int8.onnx` | Objects, COCO-80 | 256x256 | 3591 KB | 896 KB | 61 (0) | 15 ms | `PostProcess_NIA(conf, iou)` | `ObjectDetection`, `ObjectDetectionDisplay`, `GlobalShutterDetection` |
| `yolov8n_256_quant_pc_ii_seg_coco-st.tflite` | Objects with masks, COCO-80 | 256x256 | 3393 KB | 891 KB | 140 (1) | 40 ms | `PostProcess_YOLOv8_ISEG(conf, iou)` | `ObjectSegmentation`, `ObjectSegmentationDisplay` |
| `yolov8n_256_quant_pc_uf_pose_coco-st.tflite` | People with 17 keypoints | 256x256 | 3223 KB | 588 KB | 131 (2) | | `PostProcess_YOLOv8_MPE(conf, iou)` | `PoseEstimation` |
| `033_palm_detection_full_quant_pc_uf_od.tflite` | Hands, as boxes | 192x192 | 1096 KB | 1729 KB | 136 (4) | | `PostProcess_PalmDetector(conf, iou)` | `HandLandmarks`, `GlobalShutterHandLandmarks` |
| `033_hand_landmark_full_quant_pc_uf_handl.tflite` | 21 landmarks of one hand | 224x224 | 3108 KB | 1054 KB | 55 (4) | | `PostProcess_HandLandmarks(min_score)` | `HandLandmarks`, `GlobalShutterHandLandmarks` |
| `mobilenetv2_a035_128_int8.tflite` | Image class, ImageNet-1000 | 128x128 | 1656 KB | 240 KB | 55 (2) | | `PostProcess_Classifier(conf, labels, n)` | `ImageClassification`, `GlobalShutterClassification` |
| `centerface-hwc.tflite` | Faces, as boxes | 128x128 | 1983 KB | 480 KB | 72 (4) | | `FaceID` library | `FaceRecognition`, `GlobalShutterFaceDetection` |
| `face4-int8.onnx` | Face embedding | 112x112 | 1463 KB | 588 KB | 54 (5) | | `FaceID` library | `FaceRecognition` |
| `aed_yamnet_int8.onnx` | Ten sounds | 64x96 log-mel | 3159 KB | 144 KB | 31 (2) | | `N6AudioAI` library | `AudioEventDetection` |
| `pixelkit_shapes_int8.onnx` | Circles and squares | 256x256 | 898 KB | 448 KB | 122 (0) | 7 ms | `PostProcess_PixelKitYOLOX(conf, iou, labels, n)` | `PixelKitDetector`, `PixelKitDetectorDisplay`, `GlobalShutterPixelKit` |

## Models from ST's zoo

From [stm32ai-modelzoo](https://github.com/STMicroelectronics/stm32ai-modelzoo).
Put the file beside the sketch, name it in the pragma and use the
post-processor for its family:

```cpp
#pragma neuron6 model="st_yoloxn_d033_w025_192_qdq_int8.onnx" name=det
NEURON6_DECLARE_MODEL(det);
static const char *const kPerson[] = { "person" };
...
Vision.begin(&NN_Instance_det,
             PostProcess_STYoloX_Ex(0.5f, 0.5f, kPerson, 1, VND_META_SLOT_MODEL_0),
             cfg);
```

| Model file | Task | Input | Weights | Working memory | NPU passes | Post-processor | Licence |
|---|---|---|---|---|---|---|---|
| `st_yoloxn_d033_w025_192_qdq_int8.onnx` | People, one class | 192x192 | 878 KB | 333 KB | 119 (2) | `PostProcess_STYoloX` | ST SLA0044 |
| `yolov2t_192_qdq_int8.onnx` | People, one class | 192x192 | 10790 KB | 252 KB | 12 (0) | `PostProcess_TinyYOLOv2` | ST SLA0044 |
| `ssd_mobilenetv2_pt_coco_300_qdq_int8.onnx` | Objects, COCO-80 | 300x300 | 20034 KB | 1714 KB + 2109 KB PSRAM | 70 (0) | `PostProcess_SSD` | Apache-2.0 |
| `blazeface_front_128_int8.tflite` | Faces | 128x128 | 106 KB | 512 KB | 82 (4) | `PostProcess_BlazeFace` | Apache-2.0 |
| `st_movenet_lightning_a100_heatmaps_192_int8.tflite` | One person, 17 keypoints | 192x192 | 2576 KB | 1080 KB | 78 (10) | `PostProcess_MoveNet` | ST SLA0044 |
| `st_deeplabv3_mnv2_a050_s16_asppv2_256_qdq_int8.onnx` | Person pixels | 256x256 | 882 KB | 1102 KB | 63 (2) | `PostProcess_DeepLabV3` | ST SLA0044 |

`PostProcess_YOLOv8Detect` decodes the Ultralytics detection head: YOLOv8,
YOLO11, YOLOv5u, and YOLO26 as ST's zoo exports it. Those weights are
AGPL-3.0 unless a commercial Ultralytics licence is held. An export with the
NMS-free head, a `[1, 300, 6]` output, is not supported.

## Expected outputs

Shapes, data type and quantisation are read from the compiled model.

| Post-processor | Expects |
|---|---|
| `PostProcess_YOLOv8Detect` | One output, `[4 + classes][boxes]` or `[boxes][4 + classes]`, int8 or float. Boxes normalised or in input pixels |
| `PostProcess_STYoloX` | Three square grids at strides 8, 16 and 32, `5 + classes` values a cell, ST's default anchor |
| `PostProcess_PixelKitYOLOX` | A square input that is a multiple of 32. One output, `[rows][5 + classes]`, a row per cell of the stride 8, 16 and 32 grids, int8 or float. The class name count must match the model |
| `PostProcess_TinyYOLOv2` | One square grid, five anchors, ST's default anchor sizes |
| `PostProcess_SSD` | Scores `[3000][1 + classes]` with the background first, and offsets `[3000][4]` |
| `PostProcess_BlazeFace` | The front-camera model's four outputs: boxes and scores at strides 8 and 16. The six face keypoints are not decoded |
| `PostProcess_MoveNet` | The heatmap export, `[48][48][17]` for a 192-pixel input |
| `PostProcess_DeepLabV3` | Per-pixel class scores, `[classes][H][W]` or `[H][W][classes]`, class 0 the background, at most 512x512 |
| `PostProcess_Classifier` | One output of per-class scores, probabilities or raw |

A model that does not match publishes no results. `PostProcess_ZooStatus()`
returns the reason.

## Post-processing memory

The zoo post-processors share a 1 MB scratch area in PSRAM. Segmentation adds
a 256 KB label map and BlazeFace a 28 KB anchor table. A sketch that uses none
of them links none of it.
