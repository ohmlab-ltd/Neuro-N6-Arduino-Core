# Neuro N6 tutorial

How to use the Neuro N6 Arduino core. Video and results go to the
[Neuro Studio](https://github.com/ohmlab-ltd/NeuroStudio/releases/latest)
desktop application.

- [Sketch structure](#sketch-structure)
- [Camera](#camera)
- [Models](#models)
- [Vision](#vision)
- [Post-processors](#post-processors)
- [Cascaded models](#cascaded-models)
- [Classical vision](#classical-vision)
- [Display and touch](#display-and-touch)
- [Video transport](#video-transport)
- [WiFi](#wifi)
- [Ethernet](#ethernet)
- [BLE](#ble)
- [Sensors and Serial](#sensors-and-serial)
- [Audio](#audio)
- [Live tuning](#live-tuning)
- [Asset store](#asset-store)
- [Low power](#low-power)
- [SD card](#sd-card)
- [Events](#events)
- [Tasks](#tasks)
- [Tools menu](#tools-menu)
- [Serial console](#serial-console)
- [Uploading](#uploading)
- [Reference](#reference)

## Sketch structure

```cpp
#include <OV5640_Arduino.h>
#include <PostProcess.h>

#pragma neuron6 model="yolov8n_256_quant_pc_ii_seg_coco-st.tflite" name=yolov8_iseg

OV5640 camera;                 // Neuro Vision carrier
// OV5640 camera(CARRIER_TFT); // TFT carrier

NEURON6_DECLARE_MODEL(yolov8_iseg);

void setup() {
  Vision.begin(&NN_Instance_yolov8_iseg, PostProcess_YOLOv8_ISEG(0.4f, 0.5f));
}

void loop() {
  Vision.run();
}
```

| Line | Purpose |
|---|---|
| `#include <OV5640_Arduino.h>` | Camera driver |
| `#include <PostProcess.h>` | Post-processors, LCD overlay, metadata |
| `#pragma neuron6 model=... name=...` | Compiles the model and flashes its weights |
| `OV5640 camera;` | Selects the camera and the carrier |
| `NEURON6_DECLARE_MODEL(name)` | Creates `NN_Instance_<name>` |

`Arduino.h` provides the core API and the `Vision` object. A sketch with no
camera omits the sensor object and the model lines.

To require a minimum core version:

```cpp
#ifndef NEURON6_CORE_VERSION_AT_LEAST
#error "This sketch needs the Neuro N6 core 0.1.3 or later"
#elif !NEURON6_CORE_VERSION_AT_LEAST(0, 1, 3)
#error "This sketch needs the Neuro N6 core 0.1.3 or later"
#endif
```

The `#ifndef` catches a build for another board. `NEURON6_CORE_VERSION_STRING`
is the version as text.

## Camera

### Sensor object

Declare one sensor object at file scope. It registers with `Vision` and sets
the carrier's CSI lane mapping.

| Declaration | Carrier |
|---|---|
| `OV5640 camera;` | Neuro Vision carrier, `CARRIER_NEUROVISION` |
| `OV5640 camera(CARRIER_TFT);` | TFT carrier, with the LCD |
| `VD66GY camera;` | Neuro Vision carrier, `VD66GY_CARRIER_NEUROVISION` |
| `VD66GY camera(VD66GY_CARRIER_THERMAL);` | Thermal carrier, with the FLIR Lepton |

- A carrier argument that does not match the hardware gives no frames.
- If the sensor does not answer, `Vision.begin()` returns and prints its name
  and expected chip ID. The VD66GY starts after `Vision.begin()` returns and
  prints a `[VD66GY]` line if it does not start.

### Streaming with no model

```cpp
#include <OV5640_Arduino.h>
OV5640 camera;

void setup() {
  VisionConfig cfg;
  Vision.begin(cfg);
}
void loop() {
  delay(1000);
}
```

The USB stream runs at about 30 fps up to 720p, 26 fps at 1080p and 5 fps at
QSXGA. Example: `CameraStream`.

### Resolutions

OV5640, RGB565 output:

| `cfg.camera` | Pixels | Field of view | Max fps | LCD |
|---|---|---|---|---|
| `CAMERA_VGA` | 640x480 | full | 60 | letterboxed |
| `CAMERA_WVGA` (default) | 800x480 | wide crop | 60 | fills the panel |
| `CAMERA_XGA` | 1024x768 | full | 30 | off |
| `CAMERA_720P` | 1280x720 | cropped | 60 | off |
| `CAMERA_1080P` | 1920x1080 | centre crop | 30 | off |
| `CAMERA_QSXGA` | 2592x1944 | full | 15 | off |

VD66GY, raw Bayer output:

| `cfg.camera` | Output | Max fps |
|---|---|---|
| `CAMERA_QVGA` | 320x240 | 88 |
| `CAMERA_VGA`, `CAMERA_WVGA` (default) | 640x480 | 88 |
| `CAMERA_XGA` | 1024x768 | 60 |
| `CAMERA_720P` | 1120x720 | 60 |
| `CAMERA_VD66_FULL`, `CAMERA_1080P`, `CAMERA_QSXGA` | 1120x1364 | 45 |

On the OV5640, `CAMERA_QVGA` and `CAMERA_VD66_FULL` select WVGA.

### Frame rate

`cfg.fps` takes `FPS_15`, `FPS_30`, `FPS_45`, `FPS_60`, `FPS_90`, `FPS_120`,
`FPS_240` or `FPS_DEFAULT`, the mode's maximum. A higher request is clamped,
with a `[CAM]` line on the serial port.

### VD66GY ISP

| `cfg.isp_enabled` | Effect |
|---|---|
| `isp_tuning_disabled` (default) | Fixed ISP settings |
| `VD66_isp_tuning_auto` | Automatic white balance on the board |
| `VD66_isp_tuning_manual` | White balance, black level, sharpness, gamma and colour matrix set from Neuro Studio |

It has no effect on an OV5640.

### Exposure, gain and white balance

```cpp
camera.setAutoExposure(false);
camera.setAutoGain(false);
camera.setAutoWhiteBalance(false);
```

- Disabling an automatic loop holds its current value. `setExposureUs()` and
  `setGain()` set a value and select manual control.
- Exposure and gain share one brightness loop: with one locked, the other
  compensates.
- `controlCaps()` reports the controls a sensor has. A call the sensor lacks
  returns `-1`.
- The VD66GY has exposure and gain but no white balance control: see
  [its README](libraries/VD66GY/README.md#exposure-and-gain).

### Adding a sensor

```cpp
class MySensor : public ImageSensor {
public:
  MySensor() { registerAsActive(1u); }       // 1 = physical CSI lanes, 2 = inverted
  int init(CameraResolutionN6 res, CameraFpsN6 fps, CameraMirrorFlipN6 flip,
           uint16_t &out_w, uint16_t &out_h) override {
    out_w = 640; out_h = 480; return 0;       // 0 = OK, negative = not detected
  }
  uint32_t chipId() override { return 0x1234u; }
  void getFormat(SensorFormat &out) override {
    out.pixel_format = N6_PIXFMT_YUV422_YUYV;
    out.color_space  = N6_COLORSPACE_BT601;
  }
  void streamOn() override {}                 // start streaming here, not in init()
  void streamOff() override {}
  const char *name() override { return "MySensor"; }
};
MySensor camera;
```

`getFormat()` selects the DCMIPP configuration: YUV422, RGB565 or RAW8. See
[`CustomSensor`](libraries/NeuroN6Examples/examples/02.Camera/CustomSensor/CustomSensor.ino) and
[`OV5640_Sensor.cpp`](libraries/OV5640/src/OV5640_Sensor.cpp).

## Models

### `#pragma neuron6`

```cpp
#pragma neuron6 model="my_model.tflite" name=my_net
NEURON6_DECLARE_MODEL(my_net);
```

The build compiles the model with ST Edge AI Core for the board's memory map
and caches the result. A model's first build takes 1 to 2 minutes.

| Option | Meaning | Default |
|---|---|---|
| `model="file"` | `.tflite` or `.onnx`, searched in the sketch folder, `<sketch>/models/`, then the core's `models/` | |
| `name=id` | The name in `NN_Instance_<name>` | The file stem, sanitised |
| `address=0x9XXXXXXX` | Flash address of the weights | Packed in the weights pool |

A sketch holds up to eight models. To drop one, remove its `#pragma` line and
its `NEURON6_DECLARE_MODEL` line.

### Weights

- The upload writes each model's weights to external flash before the
  application, unless the board holds identical bytes. *Tools > Model
  weights* selects *Upload only when changed* or *Upload every time*.
- The weights pool is 25.5 MB, `0x90300000` to `0x91C80000`. Models pack in
  pragma order on 64 KB boundaries.

### Input requirements

- A uint8, channel-last input. The camera pipeline writes interleaved RGB888.
- Asymmetric input quantisation over [0,1]: scale `1/255`, uint8 zero point 0.
  A symmetric input (scale `1/127`, zero point 0) reads a byte `b` as
  `(b-128)/127`.
- Compiled with ST Edge AI 4.0, the version of the NPU runtime.
- A TFLite model converted from an NCHW source needs its input converted with
  `tools/tflite_input_to_hwc.py`. The generate report's input line must read
  `[b:1,h:H,w:W,c:3]`.

Bundled models: [models/README.md](models/README.md); their costs:
[models/CATALOG.md](models/CATALOG.md). Each model keeps its own licence,
which binds a firmware image that embeds it. The YOLOv8 models in this
tutorial are AGPL-3.0, and so is a sketch that embeds one.

## Vision

### `begin()` and `run()`

| Call | Effect |
|---|---|
| `Vision.begin(model, pp)` | Starts the camera, the NPU and the transport with one model and its post-processor |
| `Vision.begin(model, pp, cfg)` | The same, with a `VisionConfig` |
| `Vision.begin(cfg)`, `Vision.begin()` | No bound model |
| `Vision.run()` | One capture, inference and post-process with the bound model |
| `Vision.runWith(model, pp, mode)` | The same with a given model |
| `Vision.runRoi(roi, model, pp, mode)` | The same on a region of the frame |
| `Vision.beginRawOnly(model)`, `Vision.runRaw()` | No post-processor. `runRaw()` returns the output buffers |

`run()`, `runWith()` and `runRoi()` block the calling task until the result is
published. Streaming continues meanwhile.

### VisionConfig

| Field | Default | Meaning |
|---|---|---|
| `camera` | `CAMERA_WVGA` | Resolution |
| `fps` | `FPS_DEFAULT` | Frame rate. With the panel on it also sets the panel refresh, 30 to 60 Hz |
| `mirror_flip` | `CAMERA_AUTO_FLIP` | `CAMERA_MIRROR_FLIP_NONE`, `CAMERA_FLIP`, `CAMERA_MIRROR`, `CAMERA_MIRROR_FLIP`. Auto selects `CAMERA_FLIP` for the OV5640 on the Neuro Vision carrier, and none otherwise |
| `nn.width`, `nn.height` | `256`, `256` | The model's input size. It is not read from the model |
| `nn.aspect` | `ASPECT_STRETCH` | How the frame maps to the input |
| `nn.crop_x`, `nn.crop_y`, `nn.crop_w`, `nn.crop_h` | `0` | Crop in camera pixels, for `ASPECT_CROP_OFFSET` |
| `nn.max_fps` | `0` | Limit on `run()` inferences per second. 0 is no limit |
| `transport` | `TRANSPORT_USB` | Where the video goes |
| `lcd` | `false` | Also shows the camera and overlay on the panel, on the TFT carrier at WVGA or below |
| `af` | `AF_DISABLED` | Autofocus: `AF_DISABLED`, `AF_SINGLE`, `AF_CONTINUOUS` |
| `strobe` | `STROBE_DISABLED` | Strobe LED: `STROBE_DISABLED`, `STROBE_ENABLED` |
| `band_loc` | `N6_MEM_AUTO` | Memory for the JPEG line buffer: `N6_MEM_AUTO`, `N6_MEM_AXISRAM`, `N6_MEM_PSRAM` |
| `framebuf_loc` | `N6_MEM_AUTO` | Memory for the LCD camera framebuffer, same values |
| `isp_enabled` | `isp_tuning_disabled` | VD66GY ISP tuning |

- An `nn` size larger than the model's input runs no model; a smaller one
  misplaces the results. The serial port reports either.
- A model runs at every camera resolution. A camera to model ratio above 8 is
  decimated by 2, 4 or 8 first.

### Aspect

| `nn.aspect` | Mapping |
|---|---|
| `ASPECT_STRETCH` | The whole frame, scaled to the input |
| `ASPECT_CROP_CENTER` | The largest centred rectangle with the input's aspect ratio |
| `ASPECT_CROP_OFFSET` | The rectangle in `nn.crop_*` |
| `ASPECT_LETTERBOX` | The whole frame with its aspect ratio, centred on grey (114) bars, as PixelKit trains |

Results map back to camera coordinates in every mode; in `ASPECT_LETTERBOX`
boxes are clipped to the frame. A crop must lie inside the frame and be at
least the input size; otherwise it is adjusted, with one
`[VISION WARN] crop clamped` line.

### Several models

```cpp
#include <OV5640_Arduino.h>
#include <PostProcess.h>
#pragma neuron6 model="yolov8n_256_quant_pc_ii_seg_coco-st.tflite"  name=yolov8_iseg
#pragma neuron6 model="yolov8n_256_quant_pc_uf_pose_coco-st.tflite" name=yolov8_mpe

OV5640 camera;
NEURON6_DECLARE_MODEL(yolov8_iseg);
NEURON6_DECLARE_MODEL(yolov8_mpe);

IPostProcessor *iseg_pp;
IPostProcessor *mpe_pp;

void setup() {
  iseg_pp = PostProcess_YOLOv8_ISEG_OnSlot(0.4f, 0.5f, VND_META_SLOT_MODEL_0);
  mpe_pp  = PostProcess_YOLOv8_MPE_OnSlot (0.4f, 0.5f, VND_META_SLOT_MODEL_1);
  Vision.begin();
}

void loop() {
  Vision.runWith(&NN_Instance_yolov8_iseg, iseg_pp, OVL_CLEAR);
  Vision.runWith(&NN_Instance_yolov8_mpe,  mpe_pp,  OVL_RELOAD);
}
```

- Each model publishes to its own metadata slot, `VND_META_SLOT_MODEL_0` to
  `VND_META_SLOT_MODEL_3`. The model on `MODEL_0` owns the frame's box list,
  labels, masks and inference time. Other slots add keypoints.
- A slot keeps its last result until its model runs again.
  `vnd_meta_invalidate_slot(slot)` clears it.

The third argument sets how the LCD overlay is handled:

| Mode | Clears first | Commits after | Use |
|---|---|---|---|
| `OVL_RELOAD_CLEAR` | yes | yes | One model |
| `OVL_CLEAR` | yes | no | First of several |
| `OVL_RELOAD_CLEAR_NONE` | no | no | Middle |
| `OVL_RELOAD` | no | yes | Last |

### Custom post-processor

```cpp
static int32_t my_init(void *ctx, NN_Instance_TypeDef *m) { return 0; }
static int32_t my_run (void *ctx, NN_Instance_TypeDef *m) {
  const LL_Buffer_InfoTypeDef *out = LL_ATON_Output_Buffers_Info(m);
  return 0;
}
static void my_draw_lcd     (void *ctx, OVL_Mode_t mode) {}
static void my_emit_metadata(void *ctx)                  {}

static struct MyState g_state;
static IPostProcessor g_my_pp = {
  .init = my_init, .run = my_run,
  .draw_lcd = my_draw_lcd, .emit_metadata = my_emit_metadata,
  .ctx = &g_state,
};

void setup() { Vision.begin(&NN_Instance_my_model, &g_my_pp); }
void loop()  { Vision.run(); }
```

`draw_lcd` and `emit_metadata` may be `NULL`. The interface is in
[`NeuroN6_IPostProcessor.h`](cores/NeuroN6/NeuroN6_IPostProcessor.h). Read a
model's output layout from `mem_shape` and `mem_ndims`, not from `chpos`.

### Raw output

```cpp
void setup() {
  Vision.beginRawOnly(&NN_Instance_my_model);
}

void loop() {
  const LL_Buffer_InfoTypeDef *out = Vision.runRaw();
  if (!out) return;                            // no camera frame arrived
  for (int i = 0; out[i].name != nullptr; i++) {
    const uint8_t *bytes = LL_Buffer_addr_start(&out[i]);
    uint32_t size_bytes  = LL_Buffer_len(&out[i]);
  }
}
```

### Coordinates

Post-processors report coordinates normalised to the model's input, 0 to 1.

- `nn_norm_to_lcd_x(x)`, `nn_norm_to_lcd_y(y)`: LCD pixels.
  `nn_norm_to_cam_norm_x(x)`, `nn_norm_to_cam_norm_y(y)`: normalised to the
  camera frame, for metadata. Both in `NeuroN6.h`.
- `g_lcd_camera_x`, `_y`, `_w`, `_h`: the camera region on the panel.
  `g_nn_crop_x`, `_y`, `_w`, `_h`: the NN crop in camera pixels.
  `g_camera_full_w`, `g_camera_full_h`: the camera frame size.

## Post-processors

Each factory returns an `IPostProcessor*`.

### Bundled models

| Factory | Model | Results | Metadata |
|---|---|---|---|
| `PostProcess_YOLOv8_ISEG(conf, iou)` | YOLOv8 instance segmentation, 256x256 | `pp_output` | Boxes, masks, class names |
| `PostProcess_YOLOv8_MPE(conf, iou)` | YOLOv8 pose, 256x256 | `mpe_pp_output` | Boxes, 17 keypoints |
| `PostProcess_NIA(conf, iou)` | Nia, 256x256 | `nia_pp_output` | Boxes, COCO-80 labels |
| `PostProcess_PalmDetector(conf, iou)` | MediaPipe palm detection, 192x192 | `pd_ui_detections[]`, `pd_ui_nb_detections` | Boxes, 7 keypoints |
| `PostProcess_HandLandmarks(min_score)` | MediaPipe hand landmarks, 224x224 | `handlm_output` | 21 keypoints |

- Each has an `_OnSlot` form that takes a metadata slot as its last argument.
- `PostProcess_NIA` ignores `iou`. `PostProcess_NIA416(conf, iou)` decodes a
  416x416 Nia model into `nia416_pp_output`; none is bundled.
- Class names come from `classes_table`, COCO-80 by default. A sketch may
  define its own.
- Palm detections are best first, with a score, an NN-normalised box and 7
  keypoints: 0 wrist, 1 index MCP, 2 middle MCP, 3 ring MCP, 4 pinky MCP,
  5 thumb CMC, 6 thumb MCP. Types: `app_postprocess_pd_ui.h`.
- Hand landmarks give `valid`, `score`, `handedness` (0 left, 1 right) and 21
  NN-normalised `{x,y,z}` points: 0 wrist, 1 to 4 thumb, 5 to 8 index, 9 to 12
  middle, 13 to 16 ring, 17 to 20 pinky, each from base to tip. `z` is depth
  relative to the wrist. The bundled model's score does not exceed 0.5, so a
  `min_score` above 0.5 rejects every hand. Types: `app_postprocess_handlm_ui.h`.

### Model zoo families

These read the output shapes, data type and quantisation from the compiled
model.

| Factory | Family | Publishes |
|---|---|---|
| `PostProcess_YOLOv8Detect(conf, iou)` | Ultralytics detection head: YOLOv8, YOLO11, YOLOv5u, YOLO26 | Boxes, labels |
| `PostProcess_STYoloX(conf, iou)` | ST `st_yoloxn`, strides 8, 16, 32 | Boxes, labels |
| `PostProcess_PixelKitYOLOX(conf, iou, labels, n)` | YOLOX-Nano and YOLOX-Tiny from PixelKit | Boxes, labels |
| `PostProcess_TinyYOLOv2(conf, iou)` | `yolov2t`, `st_yololcv1` | Boxes, labels |
| `PostProcess_SSD(conf, iou)` | 300-pixel SSD, 3000 priors | Boxes, labels |
| `PostProcess_BlazeFace(conf, iou)` | MediaPipe BlazeFace, front camera | Boxes |
| `PostProcess_MoveNet(conf)` | MoveNet single-person heatmaps | Box, 17 keypoints |
| `PostProcess_DeepLabV3()` | Per-pixel class scores | One box and mask per class |
| `PostProcess_Classifier(conf, labels, n)` | Image classifier | `META_IMAGE_CLASS` |

The `_Ex` forms add the slot and the class names:

| Form | Arguments |
|---|---|
| Detectors | `(conf, iou, labels, n, slot)` |
| `PostProcess_MoveNet_Ex` | `(conf, slot)` |
| `PostProcess_DeepLabV3_Ex` | `(labels, n, slot)` |
| `PostProcess_Classifier_Ex` | `(conf, labels, n, top_k, slot)`, `top_k` 1 to 8 |

```cpp
static const char *const kPerson[] = { "person" };

#pragma neuron6 model="st_yoloxn_d033_w025_192_qdq_int8.onnx" name=det
NEURON6_DECLARE_MODEL(det);

void setup() {
  VisionConfig cfg;
  cfg.nn.width = cfg.nn.height = 192;
  cfg.nn.aspect = ASPECT_CROP_CENTER;
  Vision.begin(&NN_Instance_det,
               PostProcess_STYoloX_Ex(0.5f, 0.5f, kPerson, 1, VND_META_SLOT_MODEL_0),
               cfg);
}
```

- With no names, an 80-class model uses the COCO-80 table and any other uses
  class numbers.
- `PostProcess_ZooResults(pp, &boxes)` returns the boxes.
  `PostProcess_ClassifierTop(pp, &score)` returns the best class, or -1 when
  none is above `conf`.
- A model that does not match its family publishes no results.
  `PostProcess_ZooStatus(pp)` returns the reason as a negative code.

[PixelKit](https://github.com/ohmlab-ltd/pixelkit/releases/latest) trains
detectors and exports each as a sketch folder: the sketch, the model, and a
labels header with the class names, input size and thresholds.
`PixelKitDetector` and `PixelKitDetectorDisplay` have the same form, with a
sample model that finds circles and squares.

## Cascaded models

A detector finds a subject and a second model runs on a crop of it.

### `Vision.runRoi()`

```cpp
bool Vision.runRoi(const n6_roi_t &roi,
                   NN_Instance_TypeDef *model,
                   IPostProcessor *pp,
                   OVL_Mode_t mode = OVL_RELOAD_CLEAR);
```

- Crops `roi` in hardware and scales it to the model's input, read from the
  model. The next `run()` or `runWith()` restores the base crop.
- Returns `false` when the region is empty, the camera is not streaming, or no
  frame arrives within 1 s.
- The crop is an upright window: the region's `angle` does not rotate the
  pixels. A region past the frame edge is moved inside the frame.

### Regions

An `n6_roi_t` holds a centre and a size, camera-normalised, and a clockwise
`angle` in radians.

- `n6_roi_from_nn_box()`, `n6_roi_from_nn_box_kps()`,
  `n6_roi_from_nn_keypoints()` and `n6_roi_build()` build a region from
  NN-normalised results, using the crop in effect at the call: call them
  directly after the stage that produced the coordinates.
- An `n6_roi_spec_t` sets the rotation keypoints and target angle, then shift,
  square and scale, in that order.
- `n6_roi_expand()`, `n6_roi_aabb()` and `n6_roi_unmap_point()` grow, bound and
  map back.

Declarations: [`NeuroN6_ROI.h`](cores/NeuroN6/NeuroN6_ROI.h).

### Tracker

`n6_tracker_init(&tr, smoothing, max_misses)` sets up a tracker that smooths a
region between frames and drops the lock after `max_misses` consecutive
misses; `smoothing` 0 follows the input exactly. Feed it with
`n6_tracker_update()` or `n6_tracker_update_box()` on a hit and
`n6_tracker_miss()` on a miss, which returns `false` when the lock drops. See
[`NeuroN6_Tracker.h`](cores/NeuroN6/NeuroN6_Tracker.h) and
[`HandLandmarks`](libraries/NeuroN6Examples/examples/03.NeuralNetwork/HandLandmarks/HandLandmarks.ino),
a palm detector driving a hand landmark model.

### Face recognition

```cpp
#pragma neuron6 model="centerface-hwc.tflite"  name=face_det
#pragma neuron6 model="face4-int8.onnx"        name=face_emb

Vision.runWith(&NN_Instance_face_det, n6_face_detector(), OVL_RELOAD_CLEAR_NONE);

for (int i = 0; i < n6_face_count(); i++) {
  const n6_face_t *f = n6_face_get(i);
  Vision.runRoi(n6_face_align_roi(f), &NN_Instance_face_emb,
                n6_face_embedder(), OVL_RELOAD_CLEAR_NONE);
  if (n6_face_embedding_valid())
    slot = n6_face_bank_match(&bank, n6_face_embedding(), 0.45f, &sim);
}
```

`n6_face_align_roi()` builds the crop from the two eye landmarks. Embeddings
are L2-normalised and a match score is a cosine. There is no liveness check.
Example: `FaceRecognition`.

## Classical vision

### Frames

```cpp
static uint8_t g_snapshot[DIM * DIM * 3]
  __attribute__((aligned(32), section(".ext_psram")));

if (NN_capture_snapshot(g_snapshot, sizeof(g_snapshot))) {
  /* one RGB888 frame, DIM x DIM */
}
```

`cfg.nn.width`, `cfg.nn.height` and `cfg.nn.aspect` set the frame's size and
framing. The buffer must be 32-byte aligned. With `TRANSPORT_LCD`,
`LCD_GetCameraBuffer()` returns the panel buffer.

### Decoders

| Library | Call | Finds | Example |
|---|---|---|---|
| `Quirc` | `quirc_*` | QR codes | `QrReader` |
| `Barcode1D` | `barcode1d_scan_gray()` | EAN-13, UPC-A, EAN-8 | `BarcodeReader` |
| `N6Imgproc` | `n6_blob_find_rgb888()` | The most prominent patch of a colour | `BlobTracker` |
| `AprilTag` | `n6_apriltag_find_rgb888()` | tag36h11 markers | `AprilTagReader` |

AprilTag images are at most 320x320, `N6_APRILTAG_MAX_DIM`.

### Publishing a result

```cpp
vnd_meta_vision_t vis;
vnd_meta_vision_begin(&vis, VND_META_SLOT_QR, META_QR_FRAME, g_payload, sizeof(g_payload));
vnd_meta_vision_add(&vis, cx, cy, w, h, "hello", extra, extra_len, g_snapshot, DIM, DIM);
vnd_meta_vision_finish(&vis);
```

Boxes are normalised to the analysed window. Call `finish()` every frame: a
frame with no results clears the previous one. A box with no label is drawn as
class 0, "person".

### Image primitives

`<n6_imgproc.h>` works on 8-bit images: `n6img_rgb888_to_gray()`,
`n6img_statistics()`, `n6img_histogram()`, `n6img_otsu()`,
`n6img_threshold()`, `n6img_erode()`, `n6img_dilate()`,
`n6img_find_components()` and `n6img_absdiff()`.

```cpp
n6img_absdiff(g_prev, g_gray, g_scratch, DIM, DIM);
n6img_statistics(g_scratch, DIM, DIM, NULL, &st);
if (st.mean > 4) { /* motion */ }
```

- A `NULL` rectangle is the whole image. A rectangle past the edge is clipped.
- `n6img_find_components()` and `n6_blob_find_rgb888()` keep static working
  arrays: call each from one task.

### AprilTag pose

Pose needs the camera intrinsics and the tag's printed size. Write the
intrinsics to the board, which resets to store them:

```bash
python tools/push_calibration.py --width 640 --height 480 \
    --fx 520.1 --fy 518.4 --cx 322.7 --cy 239.1 --k1 -0.041 --k2 0.0032
```

`--width` and `--height` are the resolution the intrinsics were measured at.

```cpp
#include <n6_apriltag_pose.h>

static n6_camera_cal_t g_cal;
static bool g_have_cal = false;

void setup() {
  n6_camera_cal_t stored;
  g_have_cal = n6_camera_cal_load(&stored);
  if (g_have_cal) n6_camera_cal_scale(&stored, AT_DIM, AT_DIM, &g_cal);
}

n6_apriltag_pose_t p;
if (g_have_cal && n6_apriltag_pose(t, AT_DIM, AT_DIM, &g_cal, 0.080f, &p)) {
  /* tag side 0.080 m. p.tx, ty, tz in metres. p.rx, ry, rz Euler XYZ in degrees */
}
```

With no calibration stored, tags are published with the `pose_valid` byte of
their `META_APRILTAG` record at 0.

## Display and touch

The panel is on the OV5640 TFT carrier.

### Camera and overlay

```cpp
#include <OV5640_Arduino.h>
#include <PostProcess.h>
#pragma neuron6 model="yolov8n_256_quant_pc_ii_seg_coco-st.tflite" name=yolov8_iseg

OV5640 camera(CARRIER_TFT);
NEURON6_DECLARE_MODEL(yolov8_iseg);

void setup() {
  VisionConfig cfg;
  cfg.camera      = CAMERA_WVGA;
  cfg.transport   = TRANSPORT_LCD;
  Vision.begin(&NN_Instance_yolov8_iseg, PostProcess_YOLOv8_ISEG(0.4f, 0.5f), cfg);
}
void loop() { Vision.run(); }
```

- The panel shows modes up to WVGA. VGA is letterboxed.
- `TRANSPORT_LCD` sends no video to a host. `cfg.lcd = true` shows the panel
  as well as a host transport.
- `cfg.fps` sets the panel refresh, 30 to 60 Hz.
- The camera layer is one buffer, so motion can show a tear line. The overlay
  layer is double-buffered.

### Drawing on the overlay

```cpp
#include <PostProcess.h>
#include <ohmlab_lcd_draw.h>

uint16_t *ovl = OVL_GetBackBuffer();
if (ovl == nullptr) return;

ohm_lcd_bind_argb4444(ovl, 800, 480, 800);
ohm_lcd_clear_transparent();
ohm_lcd_set_font(&Font16);

ohm_lcd_set_color(255, 40, 40);
ohm_lcd_rect_thick(60, 180, 260, 110, 3);
ohm_lcd_text(70, 300, "UNKNOWN");

OVL_Commit();
```

- Primitives: `putpixel`, `hline`, `vline`, `rect`, `fillrect`, `rect_thick`,
  `char`, `text`.
- `ohm_lcd_set_color(r, g, b)` and `ohm_lcd_set_color_alpha(a, r, g, b)` take
  0 to 255. The layer is ARGB4444: 16 levels per channel.
- The `_white` variants draw white. `ohm_lcd_reset_color()` restores the
  default.
- In a sketch that runs a model, draw from a post-processor's `draw_lcd`
  callback.

### Touch

```cpp
#include <Wire.h>
#include <GT911.h>

GT911 touch;

void setup() {
  Wire.begin();
  if (!touch.begin()) Serial.println("no touch panel found");
}

void loop() {
  touch.read();
  if (touch.clicked(20, 400, 120, 48)) { /* button pressed */ }
}
```

| Call | Result |
|---|---|
| `read()` | Number of points, up to five. Keeps the last points when the controller has no new sample |
| `x(i)`, `y(i)`, `size(i)`, `id(i)` | One point |
| `pressed(x, y, w, h)` | True while a touch is inside the rectangle |
| `clicked(x, y, w, h)` | True after the `read()` that first reports a touch starting inside the rectangle |
| `controllerWidth()`, `controllerHeight()` | The resolution the controller is configured for |
| `setMapping()` | Axis mapping for another panel or mount. `CalibrateTouch` prints the call |

`begin()` maps touches to panel coordinates on the TFT carrier. It probes
addresses `0x5D` and `0x14`.

## Video transport

| `cfg.transport` | Video goes to | Needs |
|---|---|---|
| `TRANSPORT_USB` (default) | The USB vendor endpoint | |
| `TRANSPORT_WIFI_SOFTAP` | A host on the board's own access point | `<MayaW476.h>` |
| `TRANSPORT_LCD` | The panel | TFT carrier, WVGA or below |
| `TRANSPORT_ETHERNET` | A host over wired Ethernet | `<EthernetN6.h>` |

The USB serial port works with every transport. Video goes over WiFi only on
the board's own access point: `TRANSPORT_WIFI_STA` stays on USB.

## WiFi

The MAYA-W476 module is fitted to OV5640 boards. For a sketch that links
`MayaW476`, `ArduinoBLE` or `MayaBLE`, the upload writes the module's firmware
to flash at `0x91E00000`.

### Video over WiFi

```cpp
#include <MayaW476.h>
#include <OV5640_Arduino.h>
#include <PostProcess.h>
#pragma neuron6 model="yolov8n_256_quant_pc_ii_seg_coco-st.tflite" name=yolov8_iseg

OV5640 camera(CARRIER_TFT);
NEURON6_DECLARE_MODEL(yolov8_iseg);

void setup() {
  WiFi.beginAP("NeuroN6-AP");                  /* with the board's own password */

  VisionConfig cfg;
  cfg.transport = TRANSPORT_WIFI_SOFTAP;

  Vision.begin(&NN_Instance_yolov8_iseg,
               PostProcess_YOLOv8_ISEG(0.4f, 0.5f),
               cfg);
}

void loop() {
  static bool monitor = false;                 /* a Serial Monitor connecting */
  if (Serial && !monitor) Serial.println(WiFi.devicePassword());
  monitor = Serial;
  Vision.run();
}
```

- The host connects to TCP port 5000. The board answers discovery probes on
  UDP port 5000 and to `<hostname>.local` over mDNS, once `Vision.begin()`
  streams on the board's own access point, at `192.168.10.1` by default.
- One host at a time: while a WiFi host is connected the stream goes to it,
  otherwise to USB. The board announces its address over USB about once a
  second.
- A WiFi host may send camera controls, tuning values, ISP tuning and
  thumbnail subscriptions. Writing the asset store and entering the
  bootloader need USB.
- If WiFi cannot start, the stream stays on USB and the serial port says why.
- If the sketch hosts no network, `Vision.begin()` hosts one from the
  `setSoftAP()` settings or else the stored ones.

### Security

`begin()` and `beginAP()` take WPA2 and WPA3 with a passphrase of 8 to 63
characters. Open, WEP, WPA and mixed WPA/WPA2 networks are refused: the call
returns `WL_CONNECT_FAILED` or `WL_AP_FAILED` and the serial port says why.

| `WifiSecurity` | Meaning |
|---|---|
| `WIFI_SECURITY_WPA2` (default) | WPA2-PSK |
| `WIFI_SECURITY_WPA3_SAE` | WPA3 personal |
| `WIFI_SECURITY_WPA2_WPA3` | WPA2 and WPA3 |

### The board's password

- `WiFi.beginAP(ssid)` hosts with the board's own password,
  `WiFi.devicePassword()`: 16 characters from the chip's random number
  generator, in four groups of four.
- The first call stores it and restarts the board once. Restarts, uploads and
  stored WiFi settings leave it unchanged.
- The examples print the network and password each time a Serial Monitor
  connects. While the board hosts an access point, a USB host such as Neuro
  Studio also receives them about once a second. They are never sent over the
  network.

### WiFi API

`<MayaW476.h>` provides Arduino's `WiFi`, `WiFiClient`, `WiFiServer` and
`WiFiUDP`. `WiFiSSLClient`, `WiFi.firmwareVersion()` and `WiFi.ping()` are
absent. The radio is 2.4 GHz only.

```cpp
#include <MayaW476.h>

void setup() {
  Serial.begin(115200);
  if (WiFi.begin("MyHomeNetwork", "wifipassword") != WL_CONNECTED) {
    Serial.println("not joined yet");
  }
  Serial.println(WiFi.localIP());
}
```

| Call | Effect |
|---|---|
| `WiFi.begin(ssid, passphrase[, security])` | Joins a WPA2 or WPA3 network. Waits up to `setTimeout()`, 7 s by default, then keeps trying |
| `WiFi.begin()` | Joins or hosts the stored network. Returns `WL_NO_SSID_AVAIL` when none is stored |
| `WiFi.beginAP(ssid[, passphrase[, channel[, security]]])` | Hosts a network at 192.168.10.1/24, with the board's own password when the passphrase is absent or `nullptr`. Channel 1 to 11, or 0 for automatic |
| `WiFi.devicePassword()` | The board's own password |
| `WiFi.disconnect()` | Leaves or stops hosting |
| `WiFi.end()` | The same, and puts the WiFi radio to sleep. BLE keeps working |
| `WiFi.config(ip[, dns[, gateway[, subnet]]])` | Fixed address before `begin()`, or the access point's address before `beginAP()` |
| `WiFi.setDNS(dns1[, dns2])` | DNS servers |
| `WiFi.setHostname(name)`, `getHostname()` | DHCP and mDNS name. Default `neuron6-xxxxxx`. Set it before `begin()` |
| `WiFi.setTcpPort(port)` | Port of the Neuro Studio connection and discovery, 5000 by default. Set it before `Vision.begin()` |
| `WiFi.hostByName(name, ip)` | Resolves a name |
| `WiFi.getTime()` | Seconds since 1970 from NTP, or 0. `setTimeServer(name)` picks the server, `pool.ntp.org` by default |
| `WiFi.scanNetworks()` | Up to 32 networks, strongest first, or -1 |
| `WiFi.lowPowerMode()`, `noLowPowerMode()` | IEEE power save for a joined station |
| `WiFi.storedMode()`, `storedSSID()` | The stored network: `WL_STORED_NONE`, `WL_STORED_STA` or `WL_STORED_AP` |
| `WiFi.setLinkKey(key)` | Link key, 8 to 64 characters |
| `WiFi.setSoftAP(cfg)` | The access point `Vision.begin()` hosts for `TRANSPORT_WIFI_SOFTAP` when the sketch hosts none. It does not start one |

`localIP()`, `ipv4String()`, `subnetMask()`, `gatewayIP()`, `dnsIP(n)`,
`SSID()`, `RSSI()`, `BSSID(mac)`, `channel()` and `macAddress(mac)` describe
the network in use.

| `WiFi.status()` | Meaning |
|---|---|
| `WL_IDLE_STATUS` | Not started, or joining |
| `WL_CONNECTED` | Joined, with an address |
| `WL_NO_SSID_AVAIL` | No network of that name answered |
| `WL_CONNECT_FAILED` | The passphrase was refused or the join failed |
| `WL_CONNECTION_LOST` | The network was lost. The board is rejoining |
| `WL_DISCONNECTED` | After `disconnect()` or `end()` |
| `WL_AP_LISTENING`, `WL_AP_CONNECTED` | Hosting, with no client or with one |
| `WL_AP_FAILED` | The access point did not start |
| `WL_NO_MODULE` | No module, or its firmware did not start |

The stack holds 8 TCP connections, 4 listening sockets and 6 UDP sockets; the
host server uses one TCP and two UDP of them. A UDP datagram holds up to 1472
bytes.

### Stored settings

Neuro Studio stores a network on the board over USB: its mode, name,
passphrase, hostname and an optional link key. From a script:

```bash
python tools/bench/n6bench.py wifiset --sta NAME
```

- The passphrase is read from `N6_WIFI_KEY`. `--ap NAME` stores an access
  point, with the board's own password given `--board-password`, and `--clear`
  removes the settings. The board restarts to store them.
- Settings without WPA2 or WPA3 are refused when `begin()` reads them.

### Link key

With a link key set, a host must answer a challenge with an HMAC-SHA256 under
the key before it receives anything. The key does not encrypt the stream.

## Ethernet

`<EthernetN6.h>` streams the video over wired Ethernet with
`cfg.transport = TRANSPORT_ETHERNET`, through a LAN8742A PHY on the header.

```cpp
#include <EthernetN6.h>
#include <OV5640_Arduino.h>

OV5640 camera;

void setup() {
  EthernetN6.setStaticIp(IPAddress(192, 168, 20, 1), 5000);
  VisionConfig cfg;
  cfg.transport = TRANSPORT_ETHERNET;
  Vision.begin(cfg);
}

void loop() { delay(1000); }
```

- The address is static, with mask 255.255.255.0 and no DHCP. The default is
  192.168.20.1, TCP port 5000. `setStaticIp()` takes effect only before
  `Vision.begin()`. Give the computer an address on the same network, such as
  192.168.20.2.
- The MAC address is locally administered and made from the chip's unique
  ID, the same on every boot.
- If the link does not come up, the video stays on USB and the serial port
  says why.

| Pin | Signal |
|---|---|
| `D1` | MDC |
| `D9` | MDIO |
| `D6` | Reference clock |
| `D10` | CRS_DV |
| `A1` | TX_EN |
| `A2` | TXD0 |
| `A0` | TXD1 |
| `A4` | RXD0 |
| `D11` | RXD1 |
| `D5` | PHY reset |

The driver also holds `A5`. `libraries/EthernetN6/extras/eth_stream_test.py`
checks the stream from a computer. Example: `EthernetVideoStream`.

## BLE

`<ArduinoBLE.h>` provides the ArduinoBLE API on the MAYA-W476 module.

- `BLE.begin()` powers the module and loads its firmware. WiFi is not needed.
- A background task services the radio. `BLE.poll()` is optional.
- Peripheral and central roles, and pairing with LE Secure Connections.
- As a central the board sees legacy advertising only.
- `BLE.end()` empties the GATT database. Add the services again before the
  next `BLE.begin()`.
- `MayaBLE` advertises a named, non-connectable beacon.

Examples: `BlePeripheral`, `BleBeacon`.

## Sensors and Serial

### Wire

| Object | Bus | Pins |
|---|---|---|
| `Wire` | I2C1, shared with the camera and on-board sensors | Module connector |
| `Wire1` | I2C2 | Header `SDA` and `SCL`, 4.7k pull-ups |

- `Wire` is a master with 7-bit addresses and a 32-byte buffer.
  `Wire.begin()` and `Wire.setClock()` have no effect on the on-board bus.
- Every I2C1 user locks the bus per transaction. `n6_i2c_claim()` and
  `n6_i2c_release()`, on the same task, hold it longer;
  `n6_i2c_trylock()` returns 1 when locked and 0 when busy, released by
  `n6_i2c_unlock()`.

```cpp
n6_i2c_claim();        /* other I2C users block until release */
thermal.powerOn();
delay(2500);
n6_i2c_release();
```

### Serial

`Serial` is a `Stream` on the USB CDC port with a 512-byte receive buffer.
`Serial.begin(baud)` is accepted and ignored.

### On-board sensors

```cpp
#include <Wire.h>
#include <LSM6DSL_Arduino.h>

LSM6DSL imu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!imu.begin()) Serial.println("no IMU");
}

void loop() {
  imuVec3 a = imu.readAccel();   // g
  Serial.println(a.x, 3);
  delay(200);
}
```

| Object | Sensor | Library README | Example |
|---|---|---|---|
| `MMC5603NJ mag;` | Magnetometer | [MMC5603NJ](libraries/MMC5603NJ/README.md) | `ReadMagneticField` |
| `LSM6DSL imu;` | IMU | [LSM6DSL](libraries/LSM6DSL/README.md) | `ReadAcceleration` |
| `VL53L1CB tof;` | Time of flight | [VL53L1CB](libraries/VL53L1CB/README.md) | `ReadDistance` |

`sendNS()` publishes a reading to Neuro Studio, with or without a camera. It
needs `<PostProcess.h>`.

## Audio

### Microphone

```cpp
#include <Microphone.h>

Microphone mic;

void setup() { mic.begin(); }
void loop()  { }
```

`begin()` starts capture and streaming to Neuro Studio: 16-bit mono PCM at
32552 Hz, through a ring of about 0.5 s. `mic.peak()` returns the level.
`mic.read()` returns samples and works only with streaming off. Example:
`MicrophoneLevel`.

### Spectrum

```cpp
uint32_t got = mic.read(chunk, N6_FFT_HOP);      // 1024 samples
n6_frame_push(&frame, chunk, got);               // 4096-sample window
n6_fft_magnitude(n6_frame_data(&frame), mag);    // 2048 bins, full scale = 1.0
n6_spectrum_update(&spec, mag);                  // bars[] and peaks[], 0..1
```

Examples: `AudioSpectrum`, `AudioWaterfall`.

### Audio events

`N6AudioAI` classifies sound with the bundled `aed_yamnet_int8.onnx`.
`n6_aed_top()` returns a class index and `n6_aed_label(i)` its name. Example:
`AudioEventDetection`.

### Speaker

```cpp
#include <Speaker.h>

Speaker speaker;

void setup() {
  speaker.begin();
  speaker.tone(440, 250);      // 440 Hz for 250 ms
}
void loop() { }
```

`tone()`, `write()` for 16-bit mono PCM, and `setVolume()`, 1.0 by default,
at about 48828 Hz. `SpeakerClip` plays an embedded clip; its
`clip_to_header.py` converts an audio file to a header, 500 KB by default.
Calls and pins: the [Speaker README](libraries/Speaker/README.md).

## Live tuning

```cpp
static int32_t TARGET_HUE = 25;
static int32_t HUE_TOL    = 18;

void setup() {
  n6_tune_hue("target hue", &TARGET_HUE);        // 0..359
  n6_tune_int("hue tol",    &HUE_TOL, 0, 180);
}
```

Neuro Studio shows a control for each registered variable and writes the
variable when it moves. `n6_tune_float()` and `n6_tune_bool()` exist too.

- Up to 16 parameters, registered from `setup()` with a static or global
  variable.
- Values are clamped to the declared range and are not stored on the board.

## Asset store

```cpp
#include <n6_asset.h>

uint8_t buf[64]; uint32_t len, ver;
if (n6_asset_read(buf, sizeof(buf), &len, &ver)) {
  /* stored value */
} else {
  /* none stored */
}

n6_asset_stage(payload, n, 0);   /* version 0 = newer than the stored one */
n6_asset_apply();                /* resets the board. Does not return */
```

- The store is 4 KB in flash, shared with the stored WiFi settings and the
  board's password. The bootloader writes it after the reset.
- `n6_asset_present(&ver, &len)` reports the size. `n6_asset_read()` refuses
  a buffer that is too small.
- Reads are CRC-checked.

Example: `SaveText`.

## Low power

```cpp
#include <NeuroN6_PowerSave.h>

void setup() {
  PowerSave.begin();
  PowerSave.setClockProfile(PS_CLOCK_LOW);
  PowerSave.npuOff();
  PowerSave.sensorsSleep();      /* IMU, magnetometer, microphone */
  PowerSave.usbOff();
  PowerSave.idleWfiEnable();     /* call last */
}
```

`idleWfiEnable()` makes the idle task wait in `__WFI()`. Sleep modes, clock
profiles, a one-shot camera capture and an external watchdog are in
[the library README](libraries/NeuroN6-Powersave/README.md). Examples:
`SleepModes`, `Standby`.

## SD card

The card is on the header SPI through SdFat. Install SdFat from
*Tools > Manage Libraries*, or with `arduino-cli lib install SdFat`.

```cpp
#include <SPI.h>
#include <SdFat.h>

SPI.begin();
SPI.setKernelSource(RCC_SPI2CLKSOURCE_HSI);
```

`setKernelSource(RCC_SPI2CLKSOURCE_HSI)` is required before the card starts:
it gives the 400 kHz or lower clock the card needs.
`SPI.forceDataMode(SPI_MODE3)` fixes the SPI mode for a card that needs it.

| Signal | Pin |
|---|---|
| MOSI | `MOSI` |
| MISO | `MISO` |
| SCK | `SCK` |
| CS | `A5` |

## Events

`N6Events` keeps recent JPEG frames in a PSRAM ring. When the sketch fires an
event, it writes the frames before and after to a sink and sends a
`META_EVENT` record to the host.

```cpp
#include <N6Events.h>
#include <N6EventsSD.h>

static uint8_t snap[256 * 256 * 3] __attribute__((aligned(32), section(".ext_psram")));
static n6_motion_gate_t gate;
SdFat sd;

void setup() {
  VisionConfig cfg;
  cfg.camera = CAMERA_VGA;  cfg.fps = FPS_15;  cfg.nn.max_fps = 5;
  Vision.begin(&NN_Instance_yolov8_mpe, pp, cfg);

  n6_events_cfg_t ec = N6_EVENTS_CFG_DEFAULT;
  n6_events_begin(&ec);
  n6_motion_gate_begin(&gate, 64, 48, NULL);
  SPI.begin();
  SPI.setKernelSource(RCC_SPI2CLKSOURCE_HSI);
  sd.begin(SdSpiConfig(A5, DEDICATED_SPI, SD_SCK_MHZ(20), &SPI));
  n6_events_add_sink(n6_event_sink_sd(&sd, "/events"));
}

void loop() {
  if (NN_capture_snapshot(snap, sizeof snap) > 0 &&
      n6_motion_gate_update(&gate, snap, g_nn_out_w, g_nn_out_h)) {
    Vision.run();
    if (n6_events_last_boxes(60) > 0)
      n6_events_fire(N6_EVT_SRC_MOTION | N6_EVT_SRC_NN, 1000);
  }
  n6_events_service();
}
```

- Call `n6_events_begin()` and `n6_motion_gate_begin()` after `Vision.begin()`.
- Call `n6_events_service()` every loop. It writes the clip and blocks while
  it does.
- The SD sink writes one folder per event: a JPEG per frame, `meta.bin` and
  `event.txt`.

Examples: `EventRecording`, `EventRecordingLowPower`.

## Tasks

The core runs FreeRTOS: 1 kHz tick, 96 KB heap.

| Task | Priority | Stack | Job |
|---|---|---|---|
| `ArduinoLoop` | `tskIDLE_PRIORITY+2` | 16 KB | `setup()` and `loop()` |
| `n6_capture` | `tskIDLE_PRIORITY+3` | 4 KB | Camera frame, JPEG encode, metadata |
| `n6_usb` | `tskIDLE_PRIORITY+3` | 8 KB | USB, `Serial` input, frame send |
| `maya_w4_conn` | `tskIDLE_PRIORITY+2` | 6 KB | WiFi join, host, scan, rejoin |
| `maya_w4_vid` | `tskIDLE_PRIORITY+3` | 6 KB | WiFi host server and discovery |

- The two WiFi tasks exist once a sketch starts WiFi.
- A sketch can add tasks with `xTaskCreate()`, including `FreeRTOS.h` and
  `task.h` inside `extern "C"`.
- `millis()` and `micros()` run from the internal oscillator and can be out by
  a few tenths of a percent. Audio is clocked from the crystal.

## Tools menu

| Menu | Choices, default first |
|---|---|
| *Optimize* | Faster (`-O2`), Fastest (`-O3`), Smallest (`-Os`), Fast (`-O1`), Debug (`-O0`) |
| *Debug console* | Off, On. See [Serial console](#serial-console) |
| *Model weights* | Upload only when changed, Upload every time. Covers model weights and the WiFi firmware |
| *Build output* | Minimal, Verbose |

## Serial console

*Tools > Debug console*:

| Setting | Effect |
|---|---|
| Off (default) | No periodic output. Every received byte goes to the sketch |
| On | About once a second: `[CAMDBG]`, `[RATE]`, `[LBDBG]`, `[LBGEO]`, `[CSILIVE]`, `[LCDDBG]`, `[MICDBG]`, `[VENCDUMP]`. Single-key commands, each on a line of its own |

- With either setting the core prints the `[BOOT]` reset cause and setup
  warnings when a host opens the port. With On, a camera sketch also prints a
  `[DIAG]` register dump then.
- Lines published with `vnd_meta_publish_debug_text()` before a host attaches
  are kept, up to 2 KB, and replayed to Neuro Studio's Terminal when it
  connects.
- A red LED at power-up means the firmware reached its error handler. The
  serial port then carries an `[ERRDUMP]` block.

## Uploading

| Mode | USB ID |
|---|---|
| Application | `1209:8008` |
| Bootloader | `1209:8009` |

The board enters its bootloader on any of:

- An upload from the Arduino IDE.
- The port opened at 1200 bps and DTR dropped.
- The `REBOOT_DFU` command on the USB vendor endpoint.
- A double-tap of reset.
- `b` on a line of its own, with the debug console On.

| Tool, in `tools/neuro-n6-uploader/` | Use |
|---|---|
| `neuron6_deploy.py` | Run by the IDE. Writes changed weights and wireless firmware, then the application, and checks each by CRC |
| `neuro-n6-uploader.py` | GUI. Writes one image and checks it by CRC32 |
| `neuro-n6-uploader-cli.py` | CLI. Writes one image, with no check |

```bash
python neuro-n6-uploader-cli.py --port COMx --bin MySketch.ino_Trusted.bin --address 0x90100000
```

- The image is the `_Trusted.bin` in the build folder, with the header the
  bootloader checks.
- `--no-reboot` skips the bootloader entry. `build_exe.bat` builds the GUI as
  a standalone program.

## Reference

| Subject | Path |
|---|---|
| Examples | `libraries/*/examples/` |
| Bench sketches | `extras/bench/` |
| C API | `cores/NeuroN6/NeuroN6_app.h`, `cores/NeuroN6/NeuroN6.h` |
| Post-processors, overlay, metadata | `libraries/PostProcess/src/` |
| Host protocol | `cores/NeuroN6/n6_host_contract.h` |
| Image primitives | `libraries/N6Imgproc/src/n6_imgproc.h` |
| AprilTag and pose | `libraries/AprilTag/src/` |
| Asset store | `libraries/N6Assets/src/` |
| Low power | `libraries/NeuroN6-Powersave/` |
