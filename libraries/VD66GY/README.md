# VD66GY

Driver for the ST VD66GY, a 1.5 MP global-shutter sensor with raw Bayer
output over 2-lane MIPI CSI-2, with ST's ISP middleware.

```cpp
#include <VD66GY_Arduino.h>

VD66GY camera;
// VD66GY camera(VD66GY_CARRIER_THERMAL);   // thermal carrier, inverted CSI lanes
```

Modes and ISP settings: [TUTORIAL.md](../../TUTORIAL.md#camera).

## Exposure and gain

```cpp
camera.setExposureUs(1000);          // manual exposure, microseconds
camera.setGain(8.0f);                // analogue up to 8, then digital
camera.setAutoExposure(true);        // the sensor's own loop again

uint32_t us = camera.exposureUs();   // what the sensor applies
float    g  = camera.gain();
```

- Exposure and gain share the sensor's loop. Disabling it holds both. A manual
  exposure or gain stops it, and the other keeps the loop's last value until
  set. `setAutoExposure(true)` drops both manual values.
- Exposure is rounded up to whole lines, at least 21 lines (about 0.16 ms at
  VGA), and at most the frame period less the sensor's margin.
- Calls made before the camera starts apply when it starts.
- `VD66GY::setAeTargetPercent()`, before `Vision.begin()`, sets the loop's
  brightness target, 0 to 100.

## Examples

Each example but `GlobalShutterThermal` declares the Neuro Vision carrier,
with the Thermal carrier's declaration in a comment.

| Example | Shows |
|---|---|
| `GlobalShutterStream` | Video with no model, and the camera settings |
| `GlobalShutterExposure` | A short manual exposure lit by the strobe LED |
| `GlobalShutterDetection` | Nia object detection |
| `GlobalShutterPixelKit` | A PixelKit detector |
| `GlobalShutterClassification` | ImageNet classification |
| `GlobalShutterFaceDetection` | CenterFace face detection |
| `GlobalShutterHandLandmarks` | A palm detector, then 21 points on the hand |
| `GlobalShutterQrReader` | QR codes |
| `GlobalShutterBarcodeReader` | EAN-13, UPC-A and EAN-8 barcodes |
| `GlobalShutterAprilTag` | AprilTag markers, and their pose with a calibration |
| `GlobalShutterMotion` | The largest moving region and its speed |
| `GlobalShutterVision` | YOLOv8 segmentation with the motion, magnetic field and distance sensors |
| `GlobalShutterThermal` | YOLOv8 segmentation with the FLIR Lepton, on the Thermal carrier only |

## Licence

- `src/` and `extras/st-driver/isp/flat/isp_conf.h` are MIT.
- `extras/st-driver/` (stm32-mw-camera v1.5.1 and ISP library 2.0.1, from
  [stm32-mw-camera](https://github.com/STMicroelectronics/stm32-mw-camera) and
  [x-cube-isp](https://github.com/STMicroelectronics/x-cube-isp)) and
  `reference/` are ST's, under SLA0044 (`LICENSE.md`), except the sensor driver
  in `extras/st-driver/sensors/vd6g/`, which is BSD-3-Clause (its
  `LICENSE.md`).
- The build compiles the ISP copy in `extras/st-driver/isp/flat/`. The
  vendored files are listed in `THIRD-PARTY-NOTICES.md`.
