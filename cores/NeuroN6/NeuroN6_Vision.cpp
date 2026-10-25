/**
 ******************************************************************************
 * @file    NeuroN6_Vision.cpp
 * @brief   Vision class implementation.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "NeuroN6_Vision.h"
#include "NeuroN6.h"
#include "NeuroN6_FrameQueue.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

Vision_t Vision;

/* Null when no sensor is declared. */
ImageSensor *g_n6_active_sensor = nullptr;

/* Called from the capture task. */
extern "C" void n6_sensor_bringup(void)
{
  if (g_n6_active_sensor != nullptr) {
    g_n6_active_sensor->bringUp();
  }
}

/* Bitmask of ImageSensor::N6_CAM_CAP_*, announced as META_CAM_CAPS. */
extern "C" uint32_t n6_sensor_control_caps(void)
{
  return (g_n6_active_sensor != nullptr) ? g_n6_active_sensor->controlCaps() : 0u;
}

/* Applies a host 'CAM0' packet. apply_mask selects the fields. The auto flags
 * go first, so a value sent with its auto flag cleared is not overwritten. */
extern "C" int n6_sensor_apply_cam0(uint8_t flags, uint32_t exposure_us,
                                    uint16_t gain_x256, uint8_t apply_mask)
{
  ImageSensor *s = g_n6_active_sensor;
  if (s == nullptr) return -1;

  if (apply_mask & N6_CAM0_APPLY_AUTO_EXPOSURE) {
    (void)s->setAutoExposure((flags & N6_CAM0_FLAG_AUTO_EXPOSURE) != 0u);
  }
  if (apply_mask & N6_CAM0_APPLY_AUTO_GAIN) {
    (void)s->setAutoGain((flags & N6_CAM0_FLAG_AUTO_GAIN) != 0u);
  }
  if (apply_mask & N6_CAM0_APPLY_AUTO_AWB) {
    (void)s->setAutoWhiteBalance((flags & N6_CAM0_FLAG_AUTO_AWB) != 0u);
  }
  if (apply_mask & N6_CAM0_APPLY_EXPOSURE_US) {
    (void)s->setExposureUs(exposure_us);
  }
  if (apply_mask & N6_CAM0_APPLY_GAIN) {
    (void)s->setGain((float)gain_x256 / 256.0f);
  }
  return 0;
}

namespace {

/* Clamps the crop to what DCMIPP PIPE2 can do: inside the camera frame, no
 * smaller than the NN input since the pipe only downsizes, and even-aligned. */
void clamp_crop_to_valid(uint16_t nn_w, uint16_t nn_h,
                         uint16_t cam_w, uint16_t cam_h,
                         uint32_t req_x, uint32_t req_y,
                         uint32_t req_w, uint32_t req_h,
                         uint32_t &crop_x, uint32_t &crop_y,
                         uint32_t &crop_w, uint32_t &crop_h)
{
  uint32_t cx = req_x;
  uint32_t cy = req_y;
  uint32_t cw = req_w;
  uint32_t ch = req_h;

  bool shrunk_to_cam   = false;
  bool slid_origin     = false;
  bool enlarged_to_nn  = false;

  if (cx >= (uint32_t)cam_w) { cx = (uint32_t)cam_w - 1u; slid_origin = true; }
  if (cy >= (uint32_t)cam_h) { cy = (uint32_t)cam_h - 1u; slid_origin = true; }

  if (cw > (uint32_t)cam_w) { cw = cam_w; shrunk_to_cam = true; }
  if (ch > (uint32_t)cam_h) { ch = cam_h; shrunk_to_cam = true; }

  if (cx + cw > (uint32_t)cam_w) {
    cx = (uint32_t)cam_w - cw;   /* cw <= cam_w here */
    slid_origin = true;
  }
  if (cy + ch > (uint32_t)cam_h) {
    cy = (uint32_t)cam_h - ch;
    slid_origin = true;
  }

  if (cw < (uint32_t)nn_w) {
    cw = nn_w;
    enlarged_to_nn = true;
    if (cx + cw > (uint32_t)cam_w) {
      cx = ((uint32_t)cam_w > cw) ? ((uint32_t)cam_w - cw) : 0u;
      slid_origin = true;
    }
    if (cw > (uint32_t)cam_w) cw = cam_w;
  }
  if (ch < (uint32_t)nn_h) {
    ch = nn_h;
    enlarged_to_nn = true;
    if (cy + ch > (uint32_t)cam_h) {
      cy = ((uint32_t)cam_h > ch) ? ((uint32_t)cam_h - ch) : 0u;
      slid_origin = true;
    }
    if (ch > (uint32_t)cam_h) ch = cam_h;
  }

  crop_x = cx & ~1u;
  crop_y = cy & ~1u;
  crop_w = cw & ~1u;
  crop_h = ch & ~1u;

  if (shrunk_to_cam || slid_origin || enlarged_to_nn) {
    char reasons[64];
    int  rl = 0;
    rl += snprintf(reasons + rl, sizeof(reasons) - (size_t)rl, "%s",
                   enlarged_to_nn ? "enlarged-to-NN " : "");
    rl += snprintf(reasons + rl, sizeof(reasons) - (size_t)rl, "%s",
                   shrunk_to_cam  ? "shrunk-to-camera " : "");
    rl += snprintf(reasons + rl, sizeof(reasons) - (size_t)rl, "%s",
                   slid_origin    ? "origin-slid " : "");

    char msg[400];
    snprintf(msg, sizeof(msg),
      "[VISION WARN] crop clamped [%s]: requested (%lu,%lu,%lux%lu) for "
      "NN %ux%u camera %ux%u -> applied (%lu,%lu,%lux%lu). A crop must fit "
      "inside the camera frame and be no smaller than the NN input, since the "
      "pipe only scales down. Adjust cfg.nn.crop_* to silence this warning.\r\n",
      reasons,
      (unsigned long)req_x, (unsigned long)req_y,
      (unsigned long)req_w, (unsigned long)req_h,
      (unsigned)nn_w,        (unsigned)nn_h,
      (unsigned)cam_w,       (unsigned)cam_h,
      (unsigned long)crop_x, (unsigned long)crop_y,
      (unsigned long)crop_w, (unsigned long)crop_h);
    NeuroN6_Notice(msg);
  }
}

/* Crop rectangle in camera pixels for the aspect mode. Zeros mean no crop. */
void resolve_crop(const VisionConfig &cfg,
                  uint16_t cam_w, uint16_t cam_h,
                  uint32_t &crop_x, uint32_t &crop_y,
                  uint32_t &crop_w, uint32_t &crop_h)
{
  const uint16_t nn_w = (cfg.nn.width  != 0) ? cfg.nn.width  : 256;
  const uint16_t nn_h = (cfg.nn.height != 0) ? cfg.nn.height : 256;

  switch (cfg.nn.aspect) {
    case ASPECT_CROP_CENTER: {
      uint32_t kw, kh;
      if ((uint32_t)cam_w * (uint32_t)nn_h >
          (uint32_t)cam_h * (uint32_t)nn_w) {
        /* limited by camera height */
        kh = cam_h;
        kw = ((uint32_t)cam_h * (uint32_t)nn_w) / (uint32_t)nn_h;
      } else {
        /* limited by camera width */
        kw = cam_w;
        kh = ((uint32_t)cam_w * (uint32_t)nn_h) / (uint32_t)nn_w;
      }
      if (kw == 0u || kh == 0u) {
        crop_x = crop_y = crop_w = crop_h = 0;
        return;
      }
      uint32_t kx = ((uint32_t)cam_w - kw) / 2u;
      uint32_t ky = ((uint32_t)cam_h - kh) / 2u;
      clamp_crop_to_valid(nn_w, nn_h, cam_w, cam_h,
                          kx, ky, kw, kh,
                          crop_x, crop_y, crop_w, crop_h);
      break;
    }
    case ASPECT_CROP_OFFSET: {
      const uint32_t cx = cfg.nn.crop_x;
      const uint32_t cy = cfg.nn.crop_y;
      const uint32_t cw = cfg.nn.crop_w;
      const uint32_t ch = cfg.nn.crop_h;
      /* A zero size means unset. */
      if (cw == 0u || ch == 0u) {
        crop_x = crop_y = crop_w = crop_h = 0;
        return;
      }
      clamp_crop_to_valid(nn_w, nn_h, cam_w, cam_h,
                          cx, cy, cw, ch,
                          crop_x, crop_y, crop_w, crop_h);
      break;
    }
    case ASPECT_STRETCH:
    case ASPECT_LETTERBOX:
    default:
      crop_x = crop_y = crop_w = crop_h = 0;
      break;
  }
}

} /* anonymous namespace */

/* The order is required: each stage programs hardware the next one reads. */
void Vision_t::applyConfigureAndStart(NN_Instance_TypeDef *model,
                                      IPostProcessor      *pp,
                                      const VisionConfig  &cfg)
{
  /* Only the first begin() takes effect. */
  static bool s_vision_begun = false;
  if (s_vision_begun) {
    NeuroN6_Notice("[VISION] begin() called again and ignored. The first "
                     "configuration stays until the board resets.\r\n");
    return;
  }
  s_vision_begun = true;

  /* Set before any pipe, encoder or task setup reads it. */
  g_n6_usb_enabled = (cfg.transport != TRANSPORT_LCD) ? 1u : 0u;

  uint16_t cam_w = 800, cam_h = 480;
  camera_ok = false;
  ImageSensor *sensor = g_n6_active_sensor;
  if (sensor == nullptr) {
    NeuroN6_Notice("[VISION] no camera sensor declared - add a sensor object "
                     "at file scope, e.g. `OV5640 camera;` (with "
                     "#include <OV5640_Arduino.h>). Skipping camera bring-up.\r\n");
  } else {
    int max_fps  = sensor->maxFps(cfg.camera);
    int want_fps = (cfg.fps == FPS_DEFAULT) ? max_fps : (int)cfg.fps;
    if (want_fps > max_fps) {
      char fmsg[128];
      snprintf(fmsg, sizeof(fmsg),
        "[CAM] %d fps is not supported at this resolution; using %d.\r\n",
        want_fps, max_fps);
      NeuroN6_Notice(fmsg);
      want_fps = max_fps;
    }
    /* Must precede the LCD init: it sets the LTDC pixel-clock divider. */
    n6_lcd_set_fps(want_fps);
    int rc = sensor->init(cfg.camera, (CameraFpsN6)want_fps, cfg.mirror_flip, cam_w, cam_h);
    if (rc != 0) {
      char msg[208];
      snprintf(msg, sizeof(msg),
        "[VISION] camera '%s' did not start (rc=%d, expected chip 0x%04lX). "
        "Check that the sensor's carrier argument matches the board. No model "
        "runs; the serial port stays up.\r\n",
        sensor->name(), rc, (unsigned long)sensor->chipId());
      NeuroN6_Notice(msg);
    } else {
      camera_ok = true;
      SensorFormat sf;
      sensor->getFormat(sf);
      n6_apply_sensor_format((int)sf.pixel_format, (int)sf.color_space, (int)sf.bayer_order);

      /* Autofocus first: its firmware load resets the AF MCU, which would
       * disturb the strobe setup. */
      if (cfg.af     != AF_DISABLED)     (void)sensor->autoFocus((int)cfg.af);
      if (cfg.strobe != STROBE_DISABLED) (void)sensor->setStrobe((int)cfg.strobe);
    }
  }

  g_isp_tuning_mode = (uint8_t)cfg.isp_enabled;

  const bool lcd_transport = (cfg.transport == TRANSPORT_LCD);
  bool lcd_on = lcd_transport || cfg.lcd;
  if (lcd_on && (sensor == nullptr || !sensor->supportsLCD())) {
    if (lcd_transport) {
      NeuroN6_Notice("[VISION WARN] TRANSPORT_LCD selected, but this carrier has no LCD panel. "
                       "The OV5640 TFT carrier has one: OV5640 camera(CARRIER_TFT). No video "
                       "output; the model and the serial port still run. Use TRANSPORT_USB for "
                       "host video.\r\n");
    } else {
      NeuroN6_Notice("[VISION WARN] cfg.lcd is set, but this carrier has no LCD panel "
                       "(only the OV5640 TFT carrier does). The video still goes to the "
                       "selected transport.\r\n");
    }
    lcd_on = false;
  }
  if (lcd_on && camera_ok && (cam_w > 800u || cam_h > 480u)) {
    char msg[200];
    snprintf(msg, sizeof(msg),
      "[VISION WARN] %s: %ux%u exceeds the 800x480 panel (PIPE0 has no downscaler), so the "
      "panel stays off. %s\r\n",
      lcd_transport ? "TRANSPORT_LCD" : "cfg.lcd", (unsigned)cam_w, (unsigned)cam_h,
      lcd_transport ? "Use WVGA, or TRANSPORT_USB for full-res host video."
                    : "The video still goes to the selected transport.");
    NeuroN6_Notice(msg);
    lcd_on = false;
  }

  /* A rate of 0 keeps the boot-time CSI-2 PHY bit rate. */
  if (camera_ok && sensor) {
    uint32_t mbps = sensor->csiLaneRateMbps();
    if (sensor->defersStreaming()) {
      g_n6_deferred_cfg.mbps = mbps;   /* applied in bringUp() */
    } else if (mbps != 0u) {
      n6_csi_set_phy_bitrate(mbps);
    }
  }

  /* Stream on after the CSI-2 PHY bit rate is set, never inside init(): the
   * D-PHY then locks the HS clock on the first edge. */
  if (camera_ok && sensor && !sensor->defersStreaming()) {
    sensor->streamOn();
  }

  const uint16_t nn_w = (cfg.nn.width  != 0) ? cfg.nn.width  : 256;
  const uint16_t nn_h = (cfg.nn.height != 0) ? cfg.nn.height : 256;

  uint32_t crop_x, crop_y, crop_w, crop_h;
  resolve_crop(cfg, cam_w, cam_h, crop_x, crop_y, crop_w, crop_h);
  const bool letterbox = (cfg.nn.aspect == ASPECT_LETTERBOX);

  if (camera_ok) {
    if (n6_alloc_camera_buffers(cam_w, cam_h, lcd_on ? 1 : 0,
                                cfg.band_loc, cfg.framebuf_loc) != 0) {
      NeuroN6_Notice("[VISION] not enough memory for the camera buffers at "
                       "this resolution. Lower cfg.camera or turn the LCD off. "
                       "Camera pipes skipped.\r\n");
      camera_ok = false;
    }
  }

  /* Without a camera the CSI is uninitialised, so the pipes are left alone. */
  if (camera_ok) {
    if (sensor->defersStreaming()) {
      /* bringUp() configures the pipes from these values. */
      g_n6_deferred_cfg.cam_w  = cam_w;  g_n6_deferred_cfg.cam_h = cam_h;
      g_n6_deferred_cfg.nn_w   = nn_w;   g_n6_deferred_cfg.nn_h  = nn_h;
      g_n6_deferred_cfg.crop_x = crop_x; g_n6_deferred_cfg.crop_y = crop_y;
      g_n6_deferred_cfg.crop_w = crop_w; g_n6_deferred_cfg.crop_h = crop_h;
      g_n6_deferred_cfg.letterbox = letterbox ? 1u : 0u;
      g_n6_deferred_cfg.lcd    = lcd_on ? 1u : 0u;
      g_n6_deferred_cfg.valid  = 1u;
      g_n6_sensor_streaming    = 0u;
      /* DCMIPP_USB_Init() runs later, so the host frame size is set here. */
      usb_frame_width  = (uint32_t)cam_w;
      usb_frame_height = (uint32_t)cam_h;
    } else {
      DCMIPP_USB_Init(cam_w, cam_h);
      if (letterbox) {
        DCMIPP_NN_InitLetterbox((uint32_t)nn_w, (uint32_t)nn_h);
      } else {
        DCMIPP_NN_InitWithCrop((uint32_t)nn_w, (uint32_t)nn_h,
                               crop_x, crop_y, crop_w, crop_h);
      }
      if (lcd_on) {
        DCMIPP_LCD_Init(cam_w, cam_h);
      }
      if (g_n6_usb_enabled) {
        n6_linebuf_prime_pipe();
      }
    }
  }

  NPU_Init();

  if (camera_ok && lcd_on) {
    LCD_Init();
    LCD_feed_start(FEED_CONTINUOUS);
  }

  base_nn_w       = nn_w;
  base_nn_h       = nn_h;
  base_crop_x     = crop_x;
  base_crop_y     = crop_y;
  base_crop_w     = crop_w;
  base_crop_h     = crop_h;
  base_letterbox  = letterbox;
  roi_crop_active = false;

  boundModel = model;
  boundPP    = pp;
  lcd_active = lcd_on;
  nn_min_interval_ms = (cfg.nn.max_fps != 0u) ? (1000u / (uint32_t)cfg.nn.max_fps) : 0u;
  nn_last_run_ms     = 0u;
  if (pp && pp->init) {
    (void)pp->init(pp->ctx, model);
    markInitialised(pp);
  }

  /* Without a camera the USB task posts metadata-only frames by itself. */
  if (camera_ok) {
    NeuroN6_VisionTasks_Start();
  } else {
    NeuroN6_StreamUsbOnlyStart();
  }

  g_n6_active_transport = cfg.transport;
  if (cfg.transport == TRANSPORT_WIFI_SOFTAP || cfg.transport == TRANSPORT_WIFI_STA) {
    if (sensor == nullptr || !sensor->boardHasWiFi()) {
      NeuroN6_Notice("[VISION WARN] WiFi transport requested, but this board has no WiFi "
                       "module. Falling back to TRANSPORT_USB.\r\n");
      g_n6_active_transport = TRANSPORT_USB;
    } else {
      int rc = MayaW476_Start((int)cfg.transport);
      if (rc != 0) {
        char msg[192];
        if (rc == MAYAW476_NOT_LINKED) {
          snprintf(msg, sizeof(msg),
            "[VISION WARN] cfg.transport=%d needs the MayaW476 library: add "
            "#include <MayaW476.h> to the sketch. Falling back to "
            "TRANSPORT_USB.\r\n", (int)cfg.transport);
        } else {
          snprintf(msg, sizeof(msg),
            "[VISION WARN] cfg.transport=%d: WiFi did not start (MayaW476 "
            "returned %d, see its lines above). Falling back to "
            "TRANSPORT_USB.\r\n", (int)cfg.transport, rc);
        }
        NeuroN6_Notice(msg);
        g_n6_active_transport = TRANSPORT_USB;
      }
    }
  } else if (cfg.transport == TRANSPORT_ETHERNET) {
    int rc = EthernetN6_Start((int)cfg.transport);
    if (rc != 0) {
      char msg[192];
      if (rc == ETHERNETN6_NOT_LINKED) {
        snprintf(msg, sizeof(msg),
          "[VISION WARN] cfg.transport=%d needs the EthernetN6 library: add "
          "#include <EthernetN6.h> to the sketch. Falling back to "
          "TRANSPORT_USB.\r\n", (int)cfg.transport);
      } else {
        snprintf(msg, sizeof(msg),
          "[VISION WARN] cfg.transport=%d: Ethernet did not start (EthernetN6 "
          "returned %d). Falling back to TRANSPORT_USB.\r\n", (int)cfg.transport, rc);
      }
      NeuroN6_Notice(msg);
      g_n6_active_transport = TRANSPORT_USB;
    }
  }
}

void Vision_t::begin(NN_Instance_TypeDef *model, IPostProcessor *pp)
{
  VisionConfig cfg;
  applyConfigureAndStart(model, pp, cfg);
}

void Vision_t::begin(NN_Instance_TypeDef *model, IPostProcessor *pp,
                     const VisionConfig &cfg)
{
  applyConfigureAndStart(model, pp, cfg);
}

void Vision_t::begin()
{
  VisionConfig cfg;
  applyConfigureAndStart(/*model=*/nullptr, /*pp=*/nullptr, cfg);
}

void Vision_t::begin(const VisionConfig &cfg)
{
  applyConfigureAndStart(/*model=*/nullptr, /*pp=*/nullptr, cfg);
}

void Vision_t::beginRawOnly(NN_Instance_TypeDef *model)
{
  VisionConfig cfg;
  applyConfigureAndStart(model, /*pp=*/nullptr, cfg);
}

void Vision_t::run()
{
  if (!boundModel || !boundPP) return;

  if (nn_min_interval_ms != 0u) {
    const uint32_t now = HAL_GetTick();
    if ((now - nn_last_run_ms) < nn_min_interval_ms) {
      vTaskDelay(pdMS_TO_TICKS(2));
      return;
    }
    nn_last_run_ms = now;
  }

  runWith(boundModel, boundPP, OVL_RELOAD_CLEAR);
}

void Vision_t::restoreBaseCrop()
{
  if (!roi_crop_active) return;
  roi_crop_active = false;
  if (!camera_ok || base_nn_w == 0u || base_nn_h == 0u) return;
  if (base_letterbox) {
    DCMIPP_NN_InitLetterbox(base_nn_w, base_nn_h);
  } else {
    DCMIPP_NN_InitWithCrop(base_nn_w, base_nn_h,
                           base_crop_x, base_crop_y, base_crop_w, base_crop_h);
  }
}

bool Vision_t::runRoi(const n6_roi_t &roi, NN_Instance_TypeDef *model,
                      IPostProcessor *pp, OVL_Mode_t mode)
{
  if (!model || !camera_ok) return false;
  if (!g_n6_sensor_streaming) return false;

  /* The model's input size: mem_shape is {1, H, W, C}. */
  uint16_t nn_w = base_nn_w, nn_h = base_nn_h;
  {
    const LL_Buffer_InfoTypeDef *in = LL_ATON_Input_Buffers_Info(model);
    if (in && in[0].name != NULL && in[0].mem_shape && in[0].mem_ndims >= 4) {
      const uint32_t h = in[0].mem_shape[1];
      const uint32_t w = in[0].mem_shape[2];
      if (w > 0u && h > 0u && w <= 4096u && h <= 4096u) {
        nn_w = (uint16_t)w;
        nn_h = (uint16_t)h;
      }
    }
  }
  if (nn_w == 0u || nn_h == 0u) return false;

  const uint16_t cam_w = g_camera_full_w;
  const uint16_t cam_h = g_camera_full_h;
  if (cam_w == 0u || cam_h == 0u) return false;

  /* The DCMIPP cropper is axis-aligned: the ROI's own size is used as an
   * upright window, not its rotated bounding box. */
  const float aabb_w = roi.w;
  const float aabb_h = roi.h;
  if (!(aabb_w > 0.0f) || !(aabb_h > 0.0f)) return false;

  /* The rules of clamp_crop_to_valid(), without its warning. */
  float fw = aabb_w * (float)cam_w;
  float fh = aabb_h * (float)cam_h;
  if (!(fw >= 1.0f) || !(fh >= 1.0f)) return false;

  uint32_t cw = (uint32_t)(fw + 0.5f);
  uint32_t ch = (uint32_t)(fh + 0.5f);

  if (cw < nn_w)  cw = nn_w;
  if (ch < nn_h)  ch = nn_h;
  if (cw > cam_w) cw = cam_w;
  if (ch > cam_h) ch = cam_h;

  float fcx = roi.cx * (float)cam_w;
  float fcy = roi.cy * (float)cam_h;
  int32_t sx = (int32_t)(fcx - 0.5f * (float)cw + 0.5f);
  int32_t sy = (int32_t)(fcy - 0.5f * (float)ch + 0.5f);
  if (sx < 0) sx = 0;
  if (sy < 0) sy = 0;
  if ((uint32_t)sx + cw > cam_w) sx = (int32_t)(cam_w - cw);
  if ((uint32_t)sy + ch > cam_h) sy = (int32_t)(cam_h - ch);

  uint32_t cx = (uint32_t)sx & ~1u;
  uint32_t cy = (uint32_t)sy & ~1u;
  cw &= ~1u;
  ch &= ~1u;
  if (cw == 0u || ch == 0u) return false;

  /* Republishes g_nn_crop_*, so the outputs map back to camera coordinates. */
  DCMIPP_NN_InitWithCrop(nn_w, nn_h, cx, cy, cw, ch);
  roi_crop_active = true;

  ensureInitialised(model, pp);

  if (!NN_feed_start(FEED_SNAPSHOT, model)) return false;
  if (!NPU_Run_Inference(model)) return false;   /* no frame */

  if (pp && pp->run)                       (void)pp->run(pp->ctx, model);
  if (lcd_active && g_lcd_overlay_on && pp && pp->draw_lcd) pp->draw_lcd(pp->ctx, mode);
  if (pp && pp->emit_metadata)             pp->emit_metadata(pp->ctx);

  return true;
}

void Vision_t::runWith(NN_Instance_TypeDef *model, IPostProcessor *pp,
                       OVL_Mode_t mode)
{
  if (!model) return;

  restoreBaseCrop();
  if (!camera_ok) return;

  /* A deferred sensor's NN pipe is unconfigured until bringUp() has run. */
  if (!g_n6_sensor_streaming) return;

  ensureInitialised(model, pp);

  if (!NN_feed_start(FEED_SNAPSHOT, model)) return;   /* cfg.nn larger than the model's input */
  if (!NPU_Run_Inference(model)) return;   /* no frame */

  if (pp && pp->run)                       (void)pp->run(pp->ctx, model);
  /* With the LCD off the LTDC is uninitialised and a layer reload faults. */
  if (lcd_active && g_lcd_overlay_on && pp && pp->draw_lcd) pp->draw_lcd(pp->ctx, mode);
  if (pp && pp->emit_metadata)             pp->emit_metadata(pp->ctx);
}

bool Vision_t::isInitialised(IPostProcessor *pp) const
{
  if (!pp) return true;
  for (int i = 0; i < registered_pp_count; i++) {
    if (registered_pp[i] == pp) return true;
  }
  return false;
}

void Vision_t::markInitialised(IPostProcessor *pp)
{
  if (!pp) return;
  if (isInitialised(pp)) return;
  if (registered_pp_count >= MAX_REGISTERED_PP) {
    NeuroN6_Notice("[VISION] more than 32 post-processors: init() runs again on "
                   "each pass for the rest.\r\n");
    return;
  }
  registered_pp[registered_pp_count++] = pp;
}

void Vision_t::ensureInitialised(NN_Instance_TypeDef *model, IPostProcessor *pp)
{
  if (!pp || !pp->init) return;
  if (isInitialised(pp)) return;
  (void)pp->init(pp->ctx, model);
  markInitialised(pp);
}

const LL_Buffer_InfoTypeDef *Vision_t::runRaw()
{
  if (!boundModel) return nullptr;
  if (!camera_ok)  return nullptr;
  if (!g_n6_sensor_streaming) return nullptr;

  if (!NN_feed_start(FEED_SNAPSHOT, boundModel)) return nullptr;
  if (!NPU_Run_Inference(boundModel)) return nullptr;   /* no frame */

  return LL_ATON_Output_Buffers_Info(boundModel);
}

extern "C" bool n6_stream_pause(void)
{
  if (g_n6_active_sensor == nullptr) return false;
  if (g_n6_active_sensor->defersStreaming()) return false;   /* streamOff() is a no-op there */
  if (!g_n6_sensor_streaming) return true;                    /* already parked */

  /* The gate closes first. A capture in flight then has 150 ms to finish. */
  g_n6_sensor_streaming = 0u;
  vTaskDelay(pdMS_TO_TICKS(150));
  g_n6_active_sensor->streamOff();
  return true;
}

extern "C" bool n6_stream_resume(void)
{
  if (g_n6_active_sensor == nullptr) return false;
  if (g_n6_active_sensor->defersStreaming()) return false;
  if (g_n6_sensor_streaming) return true;                     /* already live */

  g_n6_active_sensor->streamOn();

  /* The re-sync is armed before the gate reopens. */
  g_lb_dcmipp_continuous  = 0u;
  g_lb_venc_reset_pending = 1u;
  g_n6_sensor_streaming   = 1u;
  return true;
}

