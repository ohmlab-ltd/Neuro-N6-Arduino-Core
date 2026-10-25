/**
 ******************************************************************************
 * @file    NeuroN6_Vision.h
 * @brief   Vision, the C++ wrapper over the C API in NeuroN6_app.h.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef NEURON6_VISION_H
#define NEURON6_VISION_H

#ifdef __cplusplus

#include "NeuroN6_app.h"
#include "NeuroN6_IPostProcessor.h"
#include "NeuroN6_FrameQueue.h"
#include "NeuroN6_ImageSensor.h"
#include "NeuroN6_ROI.h"

/* How the camera frame maps into the model input. */
enum AspectMode {
  ASPECT_STRETCH      = 0,   /* whole frame, scaled non-uniformly */
  ASPECT_CROP_CENTER  = 1,   /* largest centred crop of the input's aspect */
  ASPECT_CROP_OFFSET  = 2,   /* crop_x, crop_y, crop_w, crop_h */
  ASPECT_LETTERBOX    = 3,   /* whole frame, aspect kept, centred on grey 114 */
};

/* NN pipe configuration. The crop affects the model input alone: LCD and USB
 * show the full frame. */
struct NnPipeConfig {
  /* Model input size in pixels. */
  uint16_t   width;
  uint16_t   height;
  AspectMode aspect;
  /* Camera pixels. Used only with ASPECT_CROP_OFFSET. */
  uint16_t   crop_x;
  uint16_t   crop_y;
  uint16_t   crop_w;
  uint16_t   crop_h;

  /* Limit on Vision.run() inference passes per second, 0 for none. Video is
   * unaffected. */
  uint16_t   max_fps;

  NnPipeConfig()
    : width  (256),
      height (256),
      aspect (ASPECT_STRETCH),
      crop_x (0),
      crop_y (0),
      crop_w (0),
      crop_h (0),
      max_fps(0)
  {}
};

/* Values for cfg.isp_enabled. */
enum IspTuningMode {
  isp_tuning_disabled    = 0,   /* static baseline */
  VD66_isp_tuning_manual = 1,   /* VD66GY: the host drives the ISP over USB */
  VD66_isp_tuning_auto   = 2,   /* VD66GY: ST middleware auto white balance */
};

struct VisionConfig {
  CameraResolutionN6 camera;
  /* A rate above the mode's maximum is clamped. */
  CameraFpsN6        fps;
  CameraMirrorFlipN6 mirror_flip;
  NnPipeConfig       nn;
  n6_transport_t     transport;

  /* Runs the panel alongside a host transport. Needs a carrier with the panel
   * and a camera mode of WVGA or smaller. */
  bool               lcd;

  CameraAutoFocus    af;

  CameraStrobe       strobe;

  /* RAW8 sensors only. */
  IspTuningMode      isp_enabled;

  /* Memory for the encoder line-buffer band and the LCD camera framebuffer.
   * band_loc applies when video goes to a host, framebuf_loc when the panel
   * runs. */
  n6_mem_loc_t       band_loc;
  n6_mem_loc_t       framebuf_loc;

  VisionConfig()
    : camera       (CAMERA_WVGA),
      fps          (FPS_DEFAULT),
      mirror_flip  (CAMERA_AUTO_FLIP),
      nn           (),
      transport    (TRANSPORT_USB),
      lcd          (false),
      af           (AF_DISABLED),
      strobe       (STROBE_DISABLED),
      isp_enabled  (isp_tuning_disabled),
      band_loc     (N6_MEM_AUTO),
      framebuf_loc (N6_MEM_AUTO)
  {}
};

class Vision_t
{
public:
  /* Constructed before HAL init: no hardware access until begin(). */
  Vision_t() = default;

  /* Starts the camera, the NPU and the video pipes. model comes from
   * NEURON6_DECLARE_MODEL. */
  void begin(NN_Instance_TypeDef *model, IPostProcessor *pp);

  void begin(NN_Instance_TypeDef *model, IPostProcessor *pp,
             const VisionConfig &cfg);

  /* No bound model: every pass goes through runWith(). */
  void begin();
  void begin(const VisionConfig &cfg);

  /* No post-processor: runRaw() returns the NPU output buffers. */
  void beginRawOnly(NN_Instance_TypeDef *model);

  /* One pass with the bound model and post-processor. */
  void run();

  /* One pass with the given model and post-processor. Runs nothing when no
   * camera frame arrives within 1 s or cfg.nn is larger than the model's
   * input. */
  void runWith(NN_Instance_TypeDef *model, IPostProcessor *pp,
               OVL_Mode_t mode = OVL_RELOAD_CLEAR);

  /* One pass on a region of the frame, cropped in hardware and scaled to the
   * model's input. roi is camera-normalised and its rotation is not applied.
   * Returns false when the ROI is empty, the camera is not streaming or no
   * frame arrives within 1 s. The next run() or runWith() restores the crop. */
  bool runRoi(const n6_roi_t &roi, NN_Instance_TypeDef *model,
              IPostProcessor *pp, OVL_Mode_t mode = OVL_RELOAD_CLEAR);

  /* One pass with the bound model. Returns the ATON output-buffer array, ended
   * by an entry with a NULL name, or NULL when no model is bound, cfg.nn is
   * larger than the model's input or no frame arrives within 1 s. */
  const LL_Buffer_InfoTypeDef *runRaw();

  /* Overrides the sensor that registered itself. */
  void setSensor(ImageSensor &sensor) { g_n6_active_sensor = &sensor; }

  NN_Instance_TypeDef *getBoundModel() const { return boundModel; }

private:
  void applyConfigureAndStart(NN_Instance_TypeDef *model,
                              IPostProcessor      *pp,
                              const VisionConfig  &cfg);

  /* Post-processors whose init() has run. */
  static constexpr int MAX_REGISTERED_PP = 32;
  IPostProcessor *registered_pp[MAX_REGISTERED_PP] = {};
  int registered_pp_count = 0;

  bool isInitialised(IPostProcessor *pp) const;
  void markInitialised(IPostProcessor *pp);
  void ensureInitialised(NN_Instance_TypeDef *model, IPostProcessor *pp);

  NN_Instance_TypeDef *boundModel = nullptr;
  IPostProcessor      *boundPP    = nullptr;

  /* True once the sensor's init() succeeded. */
  bool                 camera_ok  = false;

  /* False when the LCD is off. draw_lcd is then skipped. */
  bool                 lcd_active = true;

  /* cfg.nn.max_fps as a minimum interval in ms, 0 for none. */
  uint32_t             nn_min_interval_ms = 0;
  uint32_t             nn_last_run_ms     = 0;

  /* NN pipe programming from cfg.nn, restored after a runRoi(). */
  uint16_t             base_nn_w      = 0;
  uint16_t             base_nn_h      = 0;
  uint32_t             base_crop_x    = 0;
  uint32_t             base_crop_y    = 0;
  uint32_t             base_crop_w    = 0;
  uint32_t             base_crop_h    = 0;
  bool                 base_letterbox = false;
  bool                 roi_crop_active = false;

  void restoreBaseCrop();
};

extern Vision_t Vision;

#endif /* __cplusplus */
#endif /* NEURON6_VISION_H */
