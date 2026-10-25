# NeuroVisionThermal

Driver for the FLIR Lepton 3.1R thermal camera: control over I2C and 160x120
video over SPI.

## Wiring

| Signal | Neuro N6 pin |
|---|---|
| I2C SDA | PE6, I2C1 |
| I2C SCL | PH9, I2C1 |
| RESET_L | PD9 |
| PWR_DWN_L | PD3 |
| SPI SCK | PA5, SPI1 |
| SPI MISO | PB4, SPI1 |
| SPI MOSI | PA7, SPI1 |
| SPI CS | PF4 |

- The Lepton answers at I2C address `0x2A`. `powerOn()` drives `RESET_L` and
  `PWR_DWN_L` high.
- It boots in about 1.5 s after `powerOn()`. Steady I2C traffic to any device
  on the bus in that time can stop the boot until the next `powerOn()`.

## Control

```cpp
#include <NeuroVisionThermal.h>

NeuroVisionThermal thermal;

void setup() {
  thermal.powerOn();
  if (!thermal.begin()) { /* boot timed out */ }

  uint16_t fpaK100;
  thermal.getFpaTemperatureK100(fpaK100);   // kelvin x 100
}
```

| Call | Result |
|---|---|
| `begin(bootTimeoutMs)` | Waits for the Lepton to boot, 3000 ms by default. Polls from 2 s after `powerOn()` |
| `isPresent()`, `isBooted()`, `waitReady(ms)` | State |
| `readRegister(reg, value)`, `writeRegister(reg, value)` | A 16-bit register |
| `ping()`, `getSerialNumber()`, `getUptimeMs()`, `getFpaTemperatureK100()`, `getAuxTemperatureK100()`, `getAgcEnabled()`, `runFfc()` | Commands |

## Video

```cpp
static uint16_t frame[NVT_VOSPI_PIXELS];   // 160 x 120

thermal.videoBegin();
if (thermal.readFrame(frame, 2000)) {
  // frame[y * 160 + x], in centikelvin with TLinear, the 3.1R default
}
```

A new frame arrives about every 115 ms. `readFrame()` returns `false` on
timeout. `setFlip180(true)` rotates frames by 180 degrees.

## Streaming to Neuro Studio

In a sketch with no camera:

```cpp
void setup() {
  thermal.powerOn();
  thermal.begin();
  thermal.streamBegin();
}
void loop() {
  if (thermal.readFrame(frame, 2000)) thermal.streamFrame(frame);
  else                                thermal.videoResync();
}
```

In a sketch that calls `Vision.begin()`, `serviceBegin()` powers the Lepton
and runs capture and streaming on its own task. Frames go out as
`META_THERMAL_FRAME` records, which need `<PostProcess.h>`.

Examples: `ThermalCamera`, `GlobalShutterThermal`.
