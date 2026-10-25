/**
 * @file    VD66GY_Sensor.h
 * @brief   ST VD66GY 1.5 MP global-shutter RAW Bayer sensor as an ImageSensor.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef VD66GY_SENSOR_H
#define VD66GY_SENSOR_H

#ifdef __cplusplus

#include "NeuroN6_ImageSensor.h"
#include <stdint.h>

/* Carrier. The value is the DCMIPP CSI DataLaneMapping. */
enum VD66GYCarrier {
  VD66GY_CARRIER_NEUROVISION = 1,   /* Neuro Vision carrier, physical data lanes */
  VD66GY_CARRIER_THERMAL     = 2,   /* Thermal carrier, inverted data lanes, no LCD */
};

class VD66GY : public ImageSensor {
public:
  /* Registers as the active sensor. Makes no HAL calls. */
  explicit VD66GY(VD66GYCarrier carrier = VD66GY_CARRIER_NEUROVISION);

  int         init(CameraResolutionN6 res, CameraFpsN6 fps, CameraMirrorFlipN6 flip,
                   uint16_t &out_w, uint16_t &out_h) override;
  /* 88, 60 or 45 fps by mode. */
  int         maxFps(CameraResolutionN6 res) override;
  uint32_t    chipId() override { return 0x5603u; }   /* VD6G_MODEL_ID, register 0x0000 */
  void        getFormat(SensorFormat &out) override;
  void        streamOn()  override;
  void        streamOff() override;
  const char *name() override { return "VD66GY"; }

  /* The link runs at 804 Mbps per lane in every mode. */
  uint32_t    csiLaneRateMbps() override;

  /* VD6G_Init and VD6G_Start run in bringUp(), not in init(). */
  bool        defersStreaming() override { return true; }

  /* Called once by the core, from the capture task. A repeat call does nothing. */
  void        bringUp() override;

  /* Auto-exposure target brightness, 0 to 100 with about 40 as visual mid, default 25.
   * Applied at bring-up. */
  static void setAeTargetPercent(int percent);

  /* Strobe LED on the sensor's GPIO0, high during each exposure. Must be set before
   * bring-up: a later call returns -1. */
  int         setStrobe(int mode) override;

  /* Holds GPIO0 high instead of pulsing it, and implies STROBE_ENABLED. Must be set before
   * bring-up. */
  static void setStrobeAlwaysOn(bool on);

  /* Exposure and gain share one loop on the sensor. Disabling either holds both. A manual
   * exposure or gain stops the loop, and the other keeps the loop's value until it is set.
   * Enabling the loop drops both manual values. A call before bring-up is applied once
   * the sensor streams. Each returns -1 on an I2C error. */
  int         setAutoExposure(bool enable) override;
  int         setAutoGain(bool enable) override;

  /* Rounded up to whole lines of about 7.7 us, or 9.5 us in the 1120-wide modes, then
   * clamped to 21 lines and to the frame length less the sensor's margin. */
  int         setExposureUs(uint32_t us) override;

  /* Analogue gain up to 8, then digital gain, about 64 in all. */
  int         setGain(float gain) override;

  uint32_t    controlCaps() override { return N6_CAM_CAP_EXPOSURE | N6_CAM_CAP_GAIN; }

  /* Exposure in microseconds and total gain of the frame being output, 0 before
   * bring-up. */
  uint32_t    exposureUs();
  float       gain();

private:
  static void        strobeWarnIfLate(const char *what);

  static int         s_aeTarget;            /* percent */
  static int         s_strobeMode;          /* STROBE_* */
  static bool        s_strobeAlwaysOn;
  static bool        s_strobeLatched;       /* set at bring-up */
  CameraResolutionN6 _res = CAMERA_VGA;    /* requested mode */
  int                _fps = 30;            /* requested frame rate */
  CameraMirrorFlipN6 _flip = CAMERA_AUTO_FLIP;   /* requested orientation */
  bool               _broughtUp = false;
  int                _bayer    = -1;       /* VD6G_BayerType_t */
};

#endif /* __cplusplus */
#endif /* VD66GY_SENSOR_H */
