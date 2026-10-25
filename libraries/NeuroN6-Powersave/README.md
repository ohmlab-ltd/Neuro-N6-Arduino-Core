# NeuroN6-Powersave

Low-power duty cycling for the Neuro N6: sleep modes, clock profiles,
peripheral gating, sensor power-down, a one-shot VD66GY capture, GPIO phase
markers and an external watchdog.

```cpp
#include <NeuroN6_PowerSave.h>
#include <PowerSaveCamera.h>
#include <VD66GY_Arduino.h>

VD66GY camera(VD66GY_CARRIER_THERMAL);

void setup() {
  PowerSave.begin();
  PowerSave.sensorsSleep();
  PowerSaveCam.begin(CAMERA_VGA);
  PowerSaveCam.sensorSleep();
  PowerSave.setClockProfile(PS_CLOCK_LOW);
}

void loop() {
  PowerSaveCam.sensorWake();
  PowerSaveCam.capture();                // RGB888 in PowerSaveCam.buffer()
  PowerSaveCam.sensorSleep();
  PowerSave.sleepStop(10000);
}
```

Examples: `SleepModes` and `Standby`.

## Sleep modes

| Call | Mode | Wakes on | Keeps | Notes |
|---|---|---|---|---|
| `sleepLight(ms)` | Sleep | Any interrupt | Everything | USB stays up |
| `sleepStop(ms)` | Stop | RTC wakeup timer | SRAM and registers | `sleepLight()` instead while a USB host is attached |
| `standby(ms)` | Standby | IWDG reset | Backup domain | Reboots through the bootloader. Does not return |

- `sleepStop()` feeds the core's 8 s watchdog in chunks, 6000 ms by default
  (`setStopChunkMs()`, 500 to 6500 ms). It re-applies the clock profile on the
  final wake and advances `millis()` by the time slept. It returns `false`
  when another interrupt ended the sleep early, or when a USB host is
  attached. Call it from `loop()` only.
- `standby()` sleeps at most 100000 ms. Each wake is a boot through the
  bootloader, reported as `wake from standby`; with a bootloader request
  pending it resets into the bootloader instead.
- Both time the sleep on the LSI, +/-5 %.
- `idleWfiEnable()` makes the idle task wait in `__WFI()`. Call it last in
  `setup()`.

## Clock profiles

| Profile | CPU | PLL1 | PLL2 and PLL3 | UART4 |
|---|---|---|---|---|
| `PS_CLOCK_FULL` | 800 MHz | on | on, off after `npuOff()` | yes |
| `PS_CLOCK_CRUISE` | 400 MHz | on | off | yes |
| `PS_CLOCK_LOW` | 100 MHz | on | off | yes |
| `PS_CLOCK_ECO` | 64 MHz, HSI | off | off | no |

- Every profile keeps PLL4, AXI at 400 MHz, HCLK and APB at 200 MHz, so XSPI,
  SDMMC, I2C, SPI and UART rates do not change.
- `setCpuDivider(d)` sets the CPU to 800/d MHz, d from 1 to 256.
- `npuOff()` keeps PLL2 and PLL3 off in every profile. Do not call it with a
  model loaded.

## Peripherals and sensors

| Call | Effect |
|---|---|
| `sensorsSleep()` | `imuSleep()`, `magSleep()` and `micSleep()` |
| `thermalPower(on)` | Lepton power through PWR_DWN_L (PD3) and RESET_L (PD9). The Lepton takes about 1.5 s to boot |
| `usbOff()` | Disconnects USB and turns off the PHY supply until a reboot |
| `periphOff(mask)`, `periphOn(mask)` | Clock gating: `PS_PERIPH_DCMIPP`, `PS_PERIPH_CSI`, `PS_PERIPH_I2C1`, `PS_PERIPH_MDF1`, `PS_PERIPH_CAMERA` |

## One-shot camera

`PowerSaveCam` runs the VD66GY and DCMIPP with no USB video and no extra tasks.
It cannot be used with `Vision.begin()`. Declare
`VD66GY camera(VD66GY_CARRIER_THERMAL);` at file scope.

| Call | Effect |
|---|---|
| `begin(res, outW, outH, fps)` | Starts the sensor and DCMIPP. `outW` and `outH` default to the mode's size |
| `capture()` | One RGB888 frame into `buffer()` |
| `setJpegMode(true)` | Before `begin()`: `captureJpeg()` returns the length of a JPEG at `jpeg()`, in place of `capture()` |
| `sensorSleep()`, `sensorWake()` | VD66GY software standby (its lowest state) and streaming |
| `busSleep()`, `busWake()` | DCMIPP and CSI kernel clocks |
| `setPipe1Assist(false)` | Before `begin()`: no PIPE1 scratch band |

## Phase markers

| Marker | Pin | MCU pin | Meaning |
|---|---|---|---|
| `PS_MARKER_AWAKE` | D5 | PE9 | High when out of Stop |
| `PS_MARKER_CAMERA` | D6 | PF5 | VD66GY capture |
| `PS_MARKER_THERMAL` | D9 | PD12 | Lepton capture |
| `PS_MARKER_AUX` | D10 | PF10 | Set by the sketch |

`markersBegin()` starts them, `markerConfig(marker, pin)` moves one,
`markerSet()` and `markerPulse()` drive them. They hold their state in Stop.

## External watchdog

For a window watchdog such as the TPS3430 on the MikroE Watchdog Click. The
falling edge on WDI must land inside the window.

| Click pin | Signal | Connect to |
|---|---|---|
| PWM | WDI | D13, with 10 k ohm to GND |
| CS | SET1 | D12, with 2.2 k ohm to GND |
| RST | SET0 | Pulled high on the Click |
| INT | WDO | Board reset |

```cpp
PowerSave.externalWatchdogArmPin(12);              // SET1 low: disabled
PowerSave.externalWatchdogBegin(13, 920, 1360, 1000);
...
PowerSave.externalWatchdogArm();                    // SET1 high: armed
```

With CWD unconnected and SET0 = SET1 = 1, the window is 920 to 1360 ms: kick
every 1000 ms. A capacitor on CWD widens it; see the TPS3430 data sheet. The
strap is sampled only as VDD rises, so change it with the board powered down.

## Notes

- `PowerSave.begin()` turns off the run LED. Call it first in `setup()`.
- UART4 has no clock in `PS_CLOCK_ECO`. Use `PS_CLOCK_LOW` with the A7683E.
