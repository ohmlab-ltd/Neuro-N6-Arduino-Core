# Neuro N6 Arduino Core

Arduino core for the Ohm Lab [Neuro N6](https://neuron6.com), an STM32N657
board with a Neural-ART NPU.

Version 0.1.3. Guide: [TUTORIAL.md](TUTORIAL.md). Changes:
[CHANGELOG.md](CHANGELOG.md).

## Board

| | |
|---|---|
| MCU | STM32N657, Cortex-M55 at 800 MHz, Neural-ART NPU |
| Camera | OV5640 5 MP, or VD66GY 1.5 MP global shutter |
| Display | 800x480 RGB LCD on the TFT carrier |
| Memory | 64 MB PSRAM (the core uses the first 32 MB), 32 MB NOR flash |
| USB | USB-C high speed: CDC serial and a vendor video stream |
| Wireless | u-blox MAYA-W476, WiFi and BLE, on OV5640 boards |
| Form factor | Adafruit Feather |

## Install

1. In the Arduino IDE, add this URL under *File > Preferences > Additional
   boards manager URLs*:

   ```
   https://raw.githubusercontent.com/ohmlab-ltd/Neuro-N6-Arduino-Core/main/package_ohmlab_index.json
   ```

2. Install **Neuro N6** in *Tools > Board > Boards Manager*.
3. Select **Neuro N6** under *Tools > Board* and the board's port under
   *Tools > Port*.

- The compiler (xpack GCC 13.3.1) and Python 3.12 install with the core.
- Linux: the install adds a udev rule. If it could not, run
  `sudo ./post_install.sh` in `~/.arduino15/packages/Ohmlab/hardware/stm32n6/0.1.3`.
- [ST Edge AI Core](https://www.st.com/en/development-tools/stedgeai-core.html)
  4.0 is needed for `#pragma neuron6`. The build finds it in ST's default
  folder, or in `NEURON6_STEDGEAI`, set to the `stedgeai` executable.
- Optional: [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
  writes the image header when `SIGNING_TOOL=auto`. Otherwise the build writes
  it. The image is not signed.
- Building from a clone: [CONTRIBUTING.md](CONTRIBUTING.md).

## Quick start

```cpp
#include <OV5640_Arduino.h>

OV5640 camera;                  // OV5640 camera(CARRIER_TFT); on the TFT carrier

void setup() {
  VisionConfig cfg;
  cfg.camera    = CAMERA_VGA;
  cfg.transport = TRANSPORT_USB;
  Vision.begin(cfg);
}

void loop() {
  delay(1000);
}
```

This streams JPEG video over USB to
[Neuro Studio](https://github.com/ohmlab-ltd/NeuroStudio/releases/latest).
The camera object names the carrier: see
[Sensor object](TUTORIAL.md#sensor-object). More sketches: *File > Examples*.

## Neural models

```cpp
#pragma neuron6 model="my_model.tflite" name=my_net
NEURON6_DECLARE_MODEL(my_net);
```

Put the quantised `.tflite` or `.onnx` beside the sketch. The build compiles
it for the NPU with ST Edge AI, and the upload writes its weights to external
flash when they differ. Up to eight models per sketch.

## Arduino API

| Area | Notes |
|---|---|
| Digital I/O | `D0`, `D1`, `D5`, `D6`, `D9` to `D13`, `A0` to `A5`, `SDA`, `SCL`, `MOSI`, `MISO`, `SCK`, the RGB LED and `BTN_USER` |
| Analog in | `analogRead` on `A0` to `A5`. Full scale 1.8 V (`VDDA_1V8`); keep inputs below it |
| PWM and tone | `analogWrite` on `D5`, `A4` and the RGB LED, 1 kHz by default. `tone` on any pin |
| Interrupts | `attachInterrupt(digitalPinToInterrupt(pin), fn, mode)`. Pins sharing a GPIO number share one line, used by one pin at a time: `D9`/`A2`, `D12`/`A3`, `D13`/`A4`/`SCL`, `D11`/`A5`/`SDA`, `A1`/`MISO` |
| Timing | `millis`, `micros`, `delay`, `delayMicroseconds`, `pulseIn`, `shiftIn`, `shiftOut` |
| Serial | USB CDC `Serial`. No `Serial1`: `D0` and `D1` are UART4, used by `A7683E` |
| SPI | `<SPI.h>` on the header pins |
| Wire | `Wire` is the on-board sensor bus. `Wire1` is the header `SDA` and `SCL`, with 4.7k pull-ups |

Ethernet takes over several header pins: see [Ethernet](TUTORIAL.md#ethernet).

## Libraries

| Area | Libraries |
|---|---|
| Camera and vision | `OV5640`, `VD66GY`, `PostProcess`, `N6Imgproc`, `Quirc`, `Barcode1D`, `AprilTag`, `FaceID`, `N6Events`, `N6EventsSD` |
| Sensors | `LSM6DSL`, `MMC5603NJ`, `VL53L1CB`, `NeuroVisionThermal`, `GT911` |
| Audio | `Microphone`, `Speaker`, `N6Audio`, `N6AudioAI` |
| Connectivity | `MayaW476` (WiFi), `ArduinoBLE`, `MayaBLE`, `EthernetN6`, `A7683E`, `A7683EVideo`, `N6lwIP` |
| System | `SPI`, `Wire`, `N6Assets`, `NeuroN6-Powersave`, `NeuroN6Examples` |

## Upload

Upload from the IDE: the board reboots into its bootloader, then the upload
writes changed model weights and the application and starts it. If the board
does not enter the bootloader, double-tap reset.

| Mode | USB ID |
|---|---|
| Application | `1209:8008` |
| Bootloader | `1209:8009` |

The core needs bootloader v2.6.1 or newer; the upload prints the version on
the board. To update it, download `Neuro-N6-FSBL-v2.6.2-Trusted.bin` from the
[latest release](https://github.com/ohmlab-ltd/Neuro-N6-Arduino-Core/releases/latest)
and, in the core's folder, run:

```
python tools/neuro-n6-uploader/neuron6_deploy.py --bin Neuro-N6-FSBL-v2.6.2-Trusted.bin --address 0x90000000
```

## Help

Open a GitHub issue with the details in
[CONTRIBUTING.md](CONTRIBUTING.md#reporting-a-problem). Report a vulnerability
as [SECURITY.md](SECURITY.md) describes.

## Licence

- Ohm Lab code is [MIT](LICENSE). Example sketches that say so are in the
  public domain.
- Firmware built with the core contains ST code under ST's SLA0044 licence,
  which allows it to run only on ST devices: the Edge AI runtime among others,
  and the parts of `cores/NeuroN6/NeuroN6.c` and `cores/NeuroN6/imageEncode.c`
  that derive from ST's code.
- The Nia model, `models/nia_1_int8.onnx`, is under the
  [Ohm Lab Model Licence](models/LICENSE-OhmLab-Model.txt) and may be run only
  on Ohm Lab hardware.
- Two YOLOv8 model files and the sketches that embed them are AGPL-3.0: see
  [models/README.md](models/README.md).
- Third-party components keep their own licences:
  [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).
