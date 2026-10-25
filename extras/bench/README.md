# Bench sketches

Diagnostic and self-test sketches. `tools/build_all_examples.py`
builds them with the examples.

| Sketch | Purpose |
|---|---|
| `AprilTagDoctor` | Reports where tag detection stops |
| `AprilTagPoseSelfTest` | Checks the pose solver against known corners |
| `AprilTagSynthetic` | Runs the detector on a synthetic image |
| `AudioLoopback` | Plays tones and measures their pitch with the microphone |
| `BenchAdc` | The on-board ADC: channels, pin map, wiring check |
| `BleBench` | BLE peripheral, pairing, central, and WiFi beside it |
| `EventsRingSelfTest` | Unit tests for the event ring |
| `I2cBusProbe` | Probes the sensor I2C bus |
| `ImgprocSelfTest` | Unit tests for the image primitives |
| `ModelBench` | Inference time per frame |
| `SdCardDoctor` | Reports why a card does not respond |
| `SdCardLogStability` | Long logging run |
| `SdCardWriteBenchmark` | Write throughput against SPI clock |
| `WiFiApi` | The WiFi API |
| `WiFiStreamingStub` | Camera stream over WiFi |
| `WiFiTwoWay` | The host link over WiFi |

## Host scripts

In `tools/bench/`.

| Script | Checks | Sketch |
|---|---|---|
| `n6bench.py` | The host link over USB and WiFi | `WiFiTwoWay` |
| `n6wifiapi.py` | The WiFi API, in phases `ap`, `sta`, `stored` and `link`. Windows only | `WiFiApi` |
| `n6bleapi.py` | BLE. Windows only | `BleBench` |
| `build_n6radio.py` | Builds `n6radio.cs`, the Windows radio helper `n6bleapi.py` uses | |
| `n6fakeboard.py` | Answers the host protocol on a PC | |
| `hosttest/` | The asset store record and the link key hash, on a PC | |

```
python tools/bench/n6bench.py console --seconds 25
python tools/bench/n6bench.py probe
python tools/bench/n6bench.py usbtest --json usbtest.json
python tools/bench/n6bench.py wifitest --host 192.168.10.1 --json wifitest.json
python tools/bench/n6wifiapi.py --port COMn --phases pass,ap,sta,stored,link --json wifiapi.json
python tools/bench/n6bleapi.py --port COMn --json ble.json
python tools/bench/n6fakeboard.py --port 5055
```

Each test prints PASS or FAIL per check and exits non-zero on a failure.

`WiFiTwoWay` hosts the access point `NeuroN6-Bench` with the board's own
password, printed at start-up. A `wifi_secrets.h` beside the sketch sets
another name or password.

`wifitest` options:

| Option | Effect |
|---|---|
| `--usb` | Also checks the hand-over between the USB and WiFi hosts |
| `--try-dfu` | Checks that `REBOOT_DFU` is refused over WiFi |
| `--probe-addr ADDR` | Broadcast address for the discovery probe |

`n6wifiapi.py` uses the PC's WiFi as the other end. It generates the
passphrases and removes the PC's WiFi profiles afterwards. `N6_STA_SSID` and
`N6_STA_KEY` name a 2.4 GHz network for the `sta` phase.
