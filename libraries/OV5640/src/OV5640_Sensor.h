/**
 ******************************************************************************
 * @file    OV5640_Sensor.h
 * @brief   OV5640 ImageSensor class.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef OV5640_SENSOR_H
#define OV5640_SENSOR_H

#ifdef __cplusplus

#include "NeuroN6_ImageSensor.h"
#include <stdint.h>

/* Carrier. The value is the CSI data-lane mapping CSI_Init programs. */
enum OV5640Carrier {
  CARRIER_NEUROVISION = 1,   /* Neuro Vision carrier, physical data lanes */
  CARRIER_TFT         = 2,   /* TFT carrier, inverted data lanes */
};

class OV5640 : public ImageSensor {
public:
  /* Registers as the active sensor. */
  explicit OV5640(OV5640Carrier carrier = CARRIER_NEUROVISION);

  int         init(CameraResolutionN6 res, CameraFpsN6 fps, CameraMirrorFlipN6 flip,
                   uint16_t &out_w, uint16_t &out_h) override;
  uint32_t    chipId() override { return 0x5640u; }
  void        getFormat(SensorFormat &out) override;
  void        streamOn()  override;
  void        streamOff() override;
  const char *name() override { return "OV5640"; }

  /* mode is a CameraAutoFocus. The first call uploads the AF firmware and
   * returns a negative value when it does not reach S_IDLE. */
  int autoFocus(int mode) override;

  /* mode is a CameraStrobe. */
  int setStrobe(int mode) override;

  /* Valid after Vision.begin(), which programs the mode and frame rate. */
  int setAutoExposure(bool enable) override;
  int setAutoGain(bool enable) override;
  int setAutoWhiteBalance(bool enable) override;
  int setExposureUs(uint32_t us) override;
  int setGain(float gain) override;

  uint32_t controlCaps() override {
    return N6_CAM_CAP_EXPOSURE | N6_CAM_CAP_GAIN | N6_CAM_CAP_AWB | N6_CAM_CAP_AF;
  }

  uint32_t csiLaneRateMbps() override;

  /* True for CARRIER_TFT. */
  bool supportsLCD() override;

  /* Both carriers fit the MayaW476 WiFi module. */
  bool boardHasWiFi() override { return true; }

private:
  OV5640Carrier      _carrier;
  bool               _af_loaded = false;        /* AF firmware uploaded */
  CameraResolutionN6 _res       = CAMERA_WVGA;   /* mode of the last init() */
  int                _fps       = 0;             /* frame rate of the last init() */
};

#endif /* __cplusplus */
#endif /* OV5640_SENSOR_H */
