/**
 * @file    PowerSaveCamera.h
 * @brief   One-shot VD66GY frame capture with sensor sleep and wake.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef POWERSAVE_CAMERA_H
#define POWERSAVE_CAMERA_H

#ifdef __cplusplus

#include <Arduino.h>
#include "NeuroN6_ImageSensor.h"
#include <stdint.h>

class PowerSaveCameraClass {
public:
  /* Starts the sensor declared at file scope and leaves it streaming. outW
   * and outH scale the frame in the DCMIPP, 0 keeping the mode's size. */
  bool begin(CameraResolutionN6 res = CAMERA_VGA,
             uint16_t outW = 0, uint16_t outH = 0,
             CameraFpsN6 fps = FPS_DEFAULT);

  /* Captures one RGB888 frame into buffer(). The sensor must be streaming. */
  bool capture(uint32_t timeoutMs = 1000);

  /* Puts the sensor in software standby. */
  void sensorSleep();

  /* Returns the sensor to streaming. Calls busWake(). */
  bool sensorWake(uint32_t timeoutMs = 500);

  /* busSleep() turns off the CSI interface, the D-PHY regulator and the
   * DCMIPP and CSI clocks. Configuration registers are retained. */
  void busSleep();
  void busWake();

  /* The frame, valid after capture() returns true. buffer() can be in PSRAM,
   * which must then stay awake. */
  uint8_t *buffer() const { return _buf; }
  uint32_t width()  const { return _outW; }
  uint32_t height() const { return _outH; }
  uint32_t frameBytes() const { return (uint32_t)_outW * _outH * 3u; }

  /* Size the sensor streams. */
  uint32_t cameraWidth()  const { return _camW; }
  uint32_t cameraHeight() const { return _camH; }

  /* PIPE1 assist, on by default: PIPE1 runs into a scratch band of
   * w x 2 x 32 bytes. Call before begin(). */
  void setPipe1Assist(bool on) { _assist = on; }

  /* Hardware JPEG mode, set before begin(). It excludes capture(). */
  void setJpegMode(bool on) { _jpegMode = on; }
  /* Returns the JFIF length in bytes, 0 on failure. The data is at jpeg(). */
  uint32_t captureJpeg(uint32_t timeoutMs = 1000);
  const uint8_t *jpeg() const { return _jpeg; }
  uint32_t jpegSize() const { return _jpegLen; }

  bool isStreaming() const { return _streaming; }

private:
  bool vd6gWrite8(uint16_t reg, uint8_t v);
  bool vd6gRead8(uint16_t reg, uint8_t &v);
  bool vd6gPoll8(uint16_t reg, uint8_t want, uint32_t timeoutMs);
  void assistStart();
  void assistStop();

  uint8_t  *_buf = nullptr;
  uint8_t  *_bufRaw = nullptr;    /* unaligned malloc pointer */
  bool      _bufPsram = false;    /* _buf is in PSRAM, which is noncacheable */
  uint8_t  *_band = nullptr;      /* PIPE1 assist scratch band */
  uint8_t  *_bandRaw = nullptr;
  bool      _jpegMode  = false;   /* PIPE1 to YUV422 to VENC JPEG */
  bool      _encoderUp = false;
  uint8_t  *_jpeg = nullptr;      /* VENC output */
  uint32_t  _jpegMax = 0;
  uint32_t  _jpegLen = 0;
  uint16_t  _camW = 0, _camH = 0;
  uint16_t  _outW = 0, _outH = 0;
  bool      _assist    = true;
  bool      _begun     = false;
  bool      _streaming = false;
};

extern PowerSaveCameraClass PowerSaveCam;

#endif /* __cplusplus */
#endif /* POWERSAVE_CAMERA_H */
