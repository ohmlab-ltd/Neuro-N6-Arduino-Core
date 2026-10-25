/**
 ******************************************************************************
 * @file    NeuroN6_ImageSensor.h
 * @brief   Camera sensor interface.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef NEURON6_IMAGE_SENSOR_H
#define NEURON6_IMAGE_SENSOR_H

#ifdef __cplusplus

#include <stdint.h>

/* Output resolution. A sensor maps a value it lacks to its nearest mode. */
enum CameraResolutionN6 {
  CAMERA_WVGA  = 0,   /* 800x480 */
  CAMERA_VGA   = 1,   /* 640x480 */
  CAMERA_XGA   = 2,   /* 1024x768 */
  CAMERA_720P  = 3,   /* 1280x720 */
  CAMERA_1080P = 4,   /* 1920x1080 */
  CAMERA_QSXGA = 5,   /* 2592x1944 */
  CAMERA_QVGA      = 6,   /* 320x240, VD66GY */
  CAMERA_VD66_FULL = 7,   /* 1120x1364, VD66GY */
};

/* Frames per second. A request above maxFps(res) is clamped. */
enum CameraFpsN6 {
  FPS_DEFAULT = 0,    /* the mode's maximum */
  FPS_15      = 15,
  FPS_30      = 30,
  FPS_45      = 45,
  FPS_60      = 60,
  FPS_90      = 90,
  FPS_120     = 120,
  FPS_240     = 240,
};

/* Default maximum frame rate per resolution. */
constexpr int n6_camera_max_fps(CameraResolutionN6 res)
{
  return (res == CAMERA_QSXGA)                        ? 15 :
         (res == CAMERA_XGA || res == CAMERA_1080P)   ? 30 :
                                                        60;
}

/* Image orientation. */
enum CameraMirrorFlipN6 {
  CAMERA_AUTO_FLIP        = -1,   /* the sensor's default for its carrier */
  CAMERA_MIRROR_FLIP_NONE = 0x00,
  CAMERA_FLIP             = 0x01,
  CAMERA_MIRROR           = 0x02,
  CAMERA_MIRROR_FLIP      = 0x03,
};

/* Autofocus mode. */
enum CameraAutoFocus {
  AF_DISABLED   = 0,
  AF_SINGLE     = 1,   /* one shot, then hold */
  AF_CONTINUOUS = 2,
};

/* Strobe LED on the sensor's STROBE pin, synchronised to the exposure. */
enum CameraStrobe {
  STROBE_DISABLED = 0,
  STROBE_ENABLED  = 1,
};

/* Sensor output format. */
enum N6PixelFormat {
  N6_PIXFMT_YUV422_YUYV = 0,
  N6_PIXFMT_RGB565      = 1,
  N6_PIXFMT_RAW8        = 2,   /* Bayer or mono, demosaiced by the DCMIPP */
};
enum N6ColorSpace {
  N6_COLORSPACE_NONE  = 0,     /* RGB */
  N6_COLORSPACE_BT601 = 1,     /* YUV */
};
struct SensorFormat {
  N6PixelFormat pixel_format;
  N6ColorSpace  color_space;
  /* RAW8 only: 0 RGGB, 1 GRBG, 2 GBRG, 3 BGGR. Mono sensors use 0. */
  uint8_t       bayer_order = 0;
};

class ImageSensor {
public:
  virtual ~ImageSensor() {}

  /* Starts the sensor and writes the frame size to out_w and out_h. fps is
   * already clamped to maxFps(res). Returns 0, or a negative value when the
   * sensor does not answer. */
  virtual int init(CameraResolutionN6 res, CameraFpsN6 fps, CameraMirrorFlipN6 flip,
                   uint16_t &out_w, uint16_t &out_h) = 0;

  /* Maximum frame rate for res. */
  virtual int maxFps(CameraResolutionN6 res) { return n6_camera_max_fps(res); }

  /* Expected chip ID. */
  virtual uint32_t chipId() = 0;

  virtual void getFormat(SensorFormat &out) = 0;

  virtual void streamOn()  = 0;
  virtual void streamOff() = 0;

  /* Name for diagnostics. */
  virtual const char *name() = 0;

  /* controlCaps() bits. */
  enum {
    N6_CAM_CAP_EXPOSURE = 0x01u,   /* setAutoExposure, setExposureUs */
    N6_CAM_CAP_GAIN     = 0x02u,   /* setAutoGain, setGain */
    N6_CAM_CAP_AWB      = 0x04u,   /* setAutoWhiteBalance */
    N6_CAM_CAP_AF       = 0x08u,   /* autoFocus */
  };

  /* Optional controls. Each returns 0, or -1 when the sensor lacks it. */

  /* mode is a CameraAutoFocus. */
  virtual int autoFocus(int mode) { (void)mode; return -1; }

  /* mode is a CameraStrobe. */
  virtual int setStrobe(int mode) { (void)mode; return -1; }

  /* Disabling an automatic loop holds its current value. */
  virtual int setAutoExposure(bool enable) { (void)enable; return -1; }
  virtual int setAutoGain(bool enable)     { (void)enable; return -1; }
  virtual int setAutoWhiteBalance(bool enable) { (void)enable; return -1; }

  /* Exposure time in microseconds, at most one frame period. Selects manual
   * exposure. */
  virtual int setExposureUs(uint32_t us) { (void)us; return -1; }

  /* Analogue gain as a multiplier, 1.0 for none. Selects manual gain. */
  virtual int setGain(float gain) { (void)gain; return -1; }

  /* Bitmask of N6_CAM_CAP_*. */
  virtual uint32_t controlCaps() { return 0u; }

  /* CSI-2 bit rate per lane in Mbps for the current mode, or 0 for the
   * default. */
  virtual uint32_t csiLaneRateMbps() { return 0; }

  /* True when the sensor starts streaming in bringUp() instead of init(). */
  virtual bool defersStreaming() { return false; }

  /* Deferred start. Called once from the capture task. */
  virtual void bringUp() {}

  /* True when the carrier has the 800x480 panel. */
  virtual bool supportsLCD() { return false; }

  /* True when the board has the MAYA-W476 module. */
  virtual bool boardHasWiFi() { return false; }

protected:
  /* Makes this the active sensor. Called from the constructor.
   * csi_lane_mapping: 1 physical, 2 inverted. */
  void registerAsActive(uint32_t csi_lane_mapping);
};

/* The active sensor. Vision.setSensor() overrides it. */
extern ImageSensor *g_n6_active_sensor;

/* CSI data-lane mapping, read by CSI_Init. */
extern "C" uint32_t _neuron6_csi_lane_mapping;

inline void ImageSensor::registerAsActive(uint32_t csi_lane_mapping)
{
  _neuron6_csi_lane_mapping = csi_lane_mapping;
  g_n6_active_sensor        = this;
}

#endif /* __cplusplus */
#endif /* NEURON6_IMAGE_SENSOR_H */
