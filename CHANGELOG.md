# Changelog

## 0.1.3

First public release.

- Arduino core for the Neuro N6, installed through Boards Manager with its
  compiler and Python runtime.
- `Vision`: camera capture, neural network inference and post-processing in
  one call, with the OV5640 and VD66GY cameras, and video over USB, WiFi,
  Ethernet or the TFT carrier's display.
- Models named in `#pragma neuron6` are compiled with ST Edge AI at build
  time and flashed beside the sketch. Post-processors cover the bundled
  models, ST model zoo families and PixelKit detectors.
- Libraries for the on-board sensors, audio, touch, WiFi and BLE,
  low power, SD card recording and classical vision. Updates go over USB.
- WiFi takes WPA2 and WPA3 only. Video over WiFi runs on the board's own
  access point, with a random password the board stores.
- Examples, from `Blink` to cascaded models.
