/**
 ******************************************************************************
 * @file    OV5640_Sensor.cpp
 * @brief   OV5640 ImageSensor implementation.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "OV5640_Sensor.h"
#include "OV5640_Arduino.h"
#include "ov5640_af_firmware.h"
#include "stm32n6xx_hal.h"

/* Autofocus and strobe registers */
#define OV5640_REG_SYS_RESET00   0x3000u   /* bit5 = AF MCU reset */
#define OV5640_AF_FW_BASE        0x8000u   /* AF firmware load address */
#define OV5640_REG_CMD_MAIN      0x3022u
#define OV5640_REG_CMD_ACK       0x3023u
#define OV5640_REG_CMD_PARA0     0x3024u
#define OV5640_REG_CMD_PARA1     0x3025u
#define OV5640_REG_CMD_PARA2     0x3026u
#define OV5640_REG_CMD_PARA3     0x3027u
#define OV5640_REG_CMD_PARA4     0x3028u
#define OV5640_REG_FW_STATUS     0x3029u
#define OV5640_AF_CMD_SINGLE     0x03u
#define OV5640_AF_CMD_CONTINUOUS 0x04u
#define OV5640_AF_STATUS_IDLE    0x70u     /* S_IDLE: firmware ready */
#define OV5640_REG_STROBE_CTRL    0x3b00u   /* bit7 ON, bit6 polarity, bits[1:0] mode */
#define OV5640_REG_PAD_OUT_EN00   0x3016u   /* bit1 = STROBE pad output enable */
#define OV5640_REG_CLOCK_ENABLE00 0x3004u   /* bit3 = STROBE peripheral clock */
#define OV5640_REG_PAD_SELECT00   0x301cu   /* bit1 = STROBE pad data path, 0 = normal */
#define OV5640_REG_FREX_MODE      0x3b07u   /* bit1 = rolling-strobe enable */

OV5640::OV5640(OV5640Carrier carrier)
  : _carrier(carrier)
{
  /* Runs at static init: no I2C or HAL here. */
  registerAsActive((uint32_t)carrier);
}

int OV5640::init(CameraResolutionN6 res, CameraFpsN6 fps, CameraMirrorFlipN6 flip,
                 uint16_t &out_w, uint16_t &out_h)
{
  _res = res;
  CameraResolution ov_res;
  switch (res) {
    case CAMERA_VGA:   ov_res = VGA;     out_w = 640;  out_h = 480;  break;
    case CAMERA_XGA:   ov_res = XGA;     out_w = 1024; out_h = 768;  break;
    case CAMERA_720P:  ov_res = HD720;   out_w = 1280; out_h = 720;  break;
    case CAMERA_1080P: ov_res = FHD1080; out_w = 1920; out_h = 1080; break;
    case CAMERA_QSXGA: ov_res = QSXGA;   out_w = 2592; out_h = 1944; break;
    case CAMERA_WVGA:
    default:           ov_res = WVGA;    out_w = 800;  out_h = 480;  break;
  }

  CameraMirrorFlip ov_flip;
  if (flip == CAMERA_AUTO_FLIP) {
    ov_flip = (_carrier == CARRIER_NEUROVISION) ? FLIP : MIRROR_FLIP_NONE;
  } else {
    ov_flip = (CameraMirrorFlip)(int)flip;
  }

  int rc = (int)ov5640_init(ov_res, ov_flip);
  if (rc != 0) {
    return rc;
  }

  /* A frame rate below native scales VTS by native/target. The pixel clock
   * and the CSI lane rate do not change. */
  const int native = n6_camera_max_fps(res);
  const int target = (fps == FPS_DEFAULT) ? native : (int)fps;
  _fps = target;
  if (target > 0 && target < native) {
    /* VTS, 0x380E high and 0x380F low: 16-bit total vertical size in lines. */
    uint16_t vts_base = (uint16_t)(((uint16_t)ov5640_read(0x380E) << 8) |
                                    (uint16_t)ov5640_read(0x380F));
    uint32_t vts_new  = (uint32_t)vts_base * (uint32_t)native / (uint32_t)target;
    if (vts_new > 0xFFFFu) { vts_new = 0xFFFFu; }
    ov5640_write(0x380E, (uint8_t)(vts_new >> 8));
    ov5640_write(0x380F, (uint8_t)(vts_new & 0xFFu));
  }

  /* ov5640_init() leaves the sensor streaming. It stops here so Vision.begin()
   * can set the CSI-2 PHY before streamOn(). */
  ov5640_stream_off();
  return rc;
}

void OV5640::getFormat(SensorFormat &out)
{
  out.pixel_format = N6_PIXFMT_RGB565;
  out.color_space  = N6_COLORSPACE_NONE;
}

void OV5640::streamOn(void)  { ov5640_stream_on();  }
void OV5640::streamOff(void) { ov5640_stream_off(); }

bool OV5640::supportsLCD(void) { return _carrier == CARRIER_TFT; }

int OV5640::autoFocus(int mode)
{
  if (mode == AF_DISABLED) {
    return 0;
  }

  /* The AF firmware is loaded once, with the AF MCU held in reset. */
  if (!_af_loaded) {
    ov5640_write(OV5640_REG_SYS_RESET00, 0x20);            /* hold AF MCU in reset */
    for (unsigned i = 0; i < ov5640_af_firmware_len; ++i) {
      ov5640_write((uint16_t)(OV5640_AF_FW_BASE + i), ov5640_af_firmware[i]);
    }
    ov5640_write(OV5640_REG_CMD_MAIN,  0x00);
    ov5640_write(OV5640_REG_CMD_ACK,   0x00);
    ov5640_write(OV5640_REG_CMD_PARA0, 0x00);
    ov5640_write(OV5640_REG_CMD_PARA1, 0x00);
    ov5640_write(OV5640_REG_CMD_PARA2, 0x00);
    ov5640_write(OV5640_REG_CMD_PARA3, 0x00);
    ov5640_write(OV5640_REG_CMD_PARA4, 0x00);
    ov5640_write(OV5640_REG_FW_STATUS, 0x7f);
    ov5640_write(OV5640_REG_SYS_RESET00, 0x00);            /* release */

    /* Waits up to 1.25 s for S_IDLE. */
    bool ready = false;
    for (int i = 0; i < 250; ++i) {
      if (ov5640_read(OV5640_REG_FW_STATUS) == OV5640_AF_STATUS_IDLE) { ready = true; break; }
      HAL_Delay(5);
    }
    if (!ready) {
      return -1;   /* no S_IDLE: no VCM or AF lens fitted */
    }
    _af_loaded = true;
  }

  /* ACK is written 0x01 before the command. The MCU clears it once consumed. */
  ov5640_write(OV5640_REG_CMD_ACK, 0x01);
  ov5640_write(OV5640_REG_CMD_MAIN,
               (mode == AF_CONTINUOUS) ? OV5640_AF_CMD_CONTINUOUS
                                       : OV5640_AF_CMD_SINGLE);
  return 0;
}

int OV5640::setStrobe(int mode)
{
  if (mode == STROBE_DISABLED) {
    ov5640_write(OV5640_REG_STROBE_CTRL, 0x00);
    return 0;
  }

  /* Order: strobe clock, pad data path, rolling strobe, pad output, request. */
  ov5640_write(OV5640_REG_CLOCK_ENABLE00,
               (uint8_t)(ov5640_read(OV5640_REG_CLOCK_ENABLE00) | 0x08));
  ov5640_write(OV5640_REG_PAD_SELECT00,
               (uint8_t)(ov5640_read(OV5640_REG_PAD_SELECT00) & (uint8_t)~0x02));
  ov5640_write(OV5640_REG_FREX_MODE,
               (uint8_t)(ov5640_read(OV5640_REG_FREX_MODE) | 0x02));
  ov5640_write(OV5640_REG_PAD_OUT_EN00,
               (uint8_t)(ov5640_read(OV5640_REG_PAD_OUT_EN00) | 0x02));
  ov5640_write(OV5640_REG_STROBE_CTRL, 0x83);   /* ON, LED3 mode. 0xC3 for an inverted LED. */
  return 0;
}

/* Exposure, gain and white balance registers */
#define OV5640_REG_AEC_MANUAL       0x3503u  /* bit0 manual exposure, bit1 manual gain */
#define OV5640_REG_AEC_EXP_19_16    0x3500u  /* exposure, 20 bits, in 1/16 line */
#define OV5640_REG_AEC_EXP_HIGH     0x3501u
#define OV5640_REG_AEC_EXP_LOW      0x3502u
#define OV5640_REG_AEC_GAIN_9_8     0x350au  /* gain, 10 bits, 16 = 1.0x */
#define OV5640_REG_AEC_GAIN_LOW     0x350bu
#define OV5640_REG_AWB_MANUAL       0x3406u  /* 0 = auto, 1 = manual */
#define OV5640_REG_TIMING_VTS_HIGH  0x380eu
#define OV5640_REG_TIMING_VTS_LOW   0x380fu

#define OV5640_AEC_MANUAL_EXPOSURE  0x01u   /* 0x3503 bit0 */
#define OV5640_AEC_MANUAL_GAIN      0x02u   /* 0x3503 bit1 */

int OV5640::setAutoExposure(bool enable)
{
  uint8_t m = ov5640_read(OV5640_REG_AEC_MANUAL);
  if (enable) { m &= (uint8_t)~OV5640_AEC_MANUAL_EXPOSURE; }
  else        { m |= (uint8_t)OV5640_AEC_MANUAL_EXPOSURE;  }
  ov5640_write(OV5640_REG_AEC_MANUAL, m);
  return 0;
}

int OV5640::setAutoGain(bool enable)
{
  uint8_t m = ov5640_read(OV5640_REG_AEC_MANUAL);
  if (enable) { m &= (uint8_t)~OV5640_AEC_MANUAL_GAIN; }
  else        { m |= (uint8_t)OV5640_AEC_MANUAL_GAIN;  }
  ov5640_write(OV5640_REG_AEC_MANUAL, m);
  return 0;
}

int OV5640::setAutoWhiteBalance(bool enable)
{
  ov5640_write(OV5640_REG_AWB_MANUAL, enable ? 0x00u : 0x01u);
  return 0;
}

int OV5640::setExposureUs(uint32_t us)
{
  /* Exposure counts in lines, and a frame is VTS lines. VTS is read back
   * because init() scales it. */
  const uint32_t vts = ((uint32_t)ov5640_read(OV5640_REG_TIMING_VTS_HIGH) << 8) |
                        (uint32_t)ov5640_read(OV5640_REG_TIMING_VTS_LOW);
  if ((vts == 0u) || (_fps <= 0)) { return -1; }

  const uint32_t frame_us = 1000000u / (uint32_t)_fps;
  if (frame_us == 0u) { return -1; }

  /* 64-bit: us * VTS overflows 32 bits. */
  uint32_t lines = (uint32_t)(((uint64_t)us * (uint64_t)vts) / (uint64_t)frame_us);

  /* A row exposes for at most one frame, less the readout overhead. */
  const uint32_t max_lines = (vts > 4u) ? (vts - 4u) : 1u;
  if (lines > max_lines) { lines = max_lines; }
  if (lines < 1u)        { lines = 1u; }

  uint32_t v = lines << 4;                    /* 1/16 line units */
  if (v > 0xFFFFFu) { v = 0xFFFFFu; }         /* 20-bit field */

  (void)setAutoExposure(false);
  ov5640_write(OV5640_REG_AEC_EXP_19_16, (uint8_t)((v >> 16) & 0x0Fu));
  ov5640_write(OV5640_REG_AEC_EXP_HIGH,  (uint8_t)((v >>  8) & 0xFFu));
  ov5640_write(OV5640_REG_AEC_EXP_LOW,   (uint8_t)( v        & 0xFFu));
  return 0;
}

int OV5640::setGain(float gain)
{
  if (gain < 1.0f)   { gain = 1.0f; }
  if (gain > 63.9f)  { gain = 63.9f; }
  const uint32_t v = (uint32_t)((gain * 16.0f) + 0.5f);

  (void)setAutoGain(false);
  ov5640_write(OV5640_REG_AEC_GAIN_9_8, (uint8_t)((v >> 8) & 0x03u));
  ov5640_write(OV5640_REG_AEC_GAIN_LOW, (uint8_t)( v       & 0xFFu));
  return 0;
}

uint32_t OV5640::csiLaneRateMbps()
{
  /* 0 keeps the boot-time BT_500. QSXGA returns 0 because BT_1350 gives no
   * packet sync on this board. */
  switch (_res) {
    case CAMERA_XGA:   return 768u;
    case CAMERA_720P:  return 592u;
    case CAMERA_1080P: return 592u;
    case CAMERA_QSXGA: return 0u;
    case CAMERA_VGA:
    case CAMERA_WVGA:
    default:           return 0u;
  }
}
