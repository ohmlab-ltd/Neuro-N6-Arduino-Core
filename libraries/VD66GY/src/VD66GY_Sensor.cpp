/**
 * @file    VD66GY_Sensor.cpp
 * @brief   VD66GY ImageSensor on the ST vd6g driver.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "VD66GY_Sensor.h"
#include "stm32n6xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#include "../extras/st-driver/sensors/vd6g/vd6g.h"

/* Defined in VD66GY_ISP.cpp. Does nothing unless VD66_ISP_AUTO_ENABLED is defined.
 * vd6g_bayer is a VD6G_BayerType_t. */
extern "C" void vd66_isp_auto_start(int vd6g_bayer);

/* The core's I2C1 handle and bus mutex. */
extern "C" I2C_HandleTypeDef hi2c1;
extern "C" void n6_i2c_lock(void);
extern "C" void n6_i2c_unlock(void);
/* The core's CDC console print. */
extern "C" void NeuroN6_DbgPrint(const char *s);
/* Prints, and prints again to each host that connects later. */
extern "C" void NeuroN6_Notice(const char *msg);
/* Starts PIPE1 into the line-buffer band. */
extern "C" void n6_linebuf_prime_pipe(void);
/* Set once the sensor streams. The core's capture task waits for it. */
extern "C" volatile uint8_t g_n6_sensor_streaming;
/* Runs the DCMIPP pipe configuration that Vision.begin() deferred. */
extern "C" void n6_vision_deferred_pipes(void);
/* Sets the demosaicer's Bayer order: 0 RGGB, 1 GRBG, 2 GBRG, 3 BGGR. */
extern "C" void n6_dcmipp_set_bayer_order(int bayer_order);

#define VD66GY_I2C_ADDR_8BIT   0x20u      /* 7-bit 0x10 */

#define VD66GY_I2C_TIMEOUT_MS  100u

/* Status of the frame being output, 0 before the first frame. */
#define VD66GY_REG_APPLIED_EXPOSURE      0x0064u   /* lines */
#define VD66GY_REG_APPLIED_ANALOG_GAIN   0x0068u   /* code c, gain 32 / (32 - c) */
#define VD66GY_REG_APPLIED_DIGITAL_GAIN  0x006Au   /* 1/256 steps */

#define VD66GY_REG_LINE_LENGTH           0x0300u   /* pixel clocks */
#define VD66GY_PIXEL_CLOCK_HZ            160800000u

/* Exposure control, as ST's vd6g.c uses it. */
#define VD66GY_REG_EXP_MODE              0x044Cu   /* VD6G_ExposureMode_t */
#define VD66GY_REG_MANUAL_ANALOG_GAIN    0x044Du   /* code */
#define VD66GY_REG_MANUAL_EXPOSURE       0x044Eu   /* lines */
#define VD66GY_REG_MANUAL_DIGITAL_GAIN   0x0450u   /* four channels, 2 bytes apart */
#define VD66GY_REG_FRAME_LENGTH          0x0458u   /* lines */
#define VD66GY_REG_EXP_MARGIN            0x0946u   /* lines */
#define VD66GY_REG_MAX_DIGITAL_GAIN      0x095Eu
#define VD66GY_REG_MAX_ANALOG_GAIN       0x0960u   /* low 5 bits */
#define VD66GY_REG_MIN_DIGITAL_GAIN      0x097Cu
#define VD66GY_REG_MIN_ANALOG_GAIN       0x097Eu   /* low 5 bits */

/* Manual exposure range: 21 lines to the frame length less the margin less 7. The
 * margin is at least 68. */
#define VD66GY_EXP_MIN_LINES             21u
#define VD66GY_EXP_END_LINES             7u
#define VD66GY_EXP_MIN_MARGIN            68u

static VD6G_Ctx_t s_vd6g;

/* vd6g driver callbacks: 16-bit register addresses, values least significant byte first.
 * shutdown_pin does nothing: the carrier holds XSHUTDOWN. */
static void vd6g_cb_shutdown(VD6G_Ctx_t *c, int v) { (void)c; (void)v; }
static int vd6g_cb_read8(VD6G_Ctx_t *c, uint16_t a, uint8_t *v) { (void)c;
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, VD66GY_I2C_ADDR_8BIT, a, I2C_MEMADD_SIZE_16BIT, v, 1u, VD66GY_I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  return (st == HAL_OK) ? 0 : -1; }
static int vd6g_cb_read16(VD6G_Ctx_t *c, uint16_t a, uint16_t *v) { (void)c;
  uint8_t d[2];
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, VD66GY_I2C_ADDR_8BIT, a, I2C_MEMADD_SIZE_16BIT, d, 2u, VD66GY_I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  if (st != HAL_OK) return -1;
  *v = (uint16_t)(((uint16_t)d[1] << 8) | d[0]); return 0; }
static int vd6g_cb_read32(VD6G_Ctx_t *c, uint16_t a, uint32_t *v) { (void)c;
  uint8_t d[4];
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, VD66GY_I2C_ADDR_8BIT, a, I2C_MEMADD_SIZE_16BIT, d, 4u, VD66GY_I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  if (st != HAL_OK) return -1;
  *v = ((uint32_t)d[3] << 24) | ((uint32_t)d[2] << 16) | ((uint32_t)d[1] << 8) | d[0]; return 0; }
static int vd6g_cb_write8(VD6G_Ctx_t *c, uint16_t a, uint8_t v) { (void)c;
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, VD66GY_I2C_ADDR_8BIT, a, I2C_MEMADD_SIZE_16BIT, &v, 1u, VD66GY_I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  return (st == HAL_OK) ? 0 : -1; }
static int vd6g_cb_write16(VD6G_Ctx_t *c, uint16_t a, uint16_t v) { (void)c;
  uint8_t d[2] = { (uint8_t)(v & 0xFFu), (uint8_t)(v >> 8) };
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, VD66GY_I2C_ADDR_8BIT, a, I2C_MEMADD_SIZE_16BIT, d, 2u, VD66GY_I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  return (st == HAL_OK) ? 0 : -1; }
static int vd6g_cb_write32(VD6G_Ctx_t *c, uint16_t a, uint32_t v) { (void)c;
  uint8_t d[4] = { (uint8_t)(v & 0xFFu), (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, VD66GY_I2C_ADDR_8BIT, a, I2C_MEMADD_SIZE_16BIT, d, 4u, VD66GY_I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  return (st == HAL_OK) ? 0 : -1; }
static int vd6g_cb_write_array(VD6G_Ctx_t *c, uint16_t a, uint8_t *data, int len) { (void)c;
  n6_i2c_lock();
  HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, VD66GY_I2C_ADDR_8BIT, a, I2C_MEMADD_SIZE_16BIT, data, (uint16_t)len, VD66GY_I2C_TIMEOUT_MS);
  n6_i2c_unlock();
  return (st == HAL_OK) ? 0 : -1; }
static void vd6g_cb_delay(VD6G_Ctx_t *c, uint32_t ms) { (void)c; HAL_Delay(ms); }
static void vd6g_cb_log(VD6G_Ctx_t *c, int lvl, const char *fmt, va_list ap) { (void)c;
  if (lvl > VD6G_LVL_DBG(0)) return;
  char b[192];
  int n = vsnprintf(b, sizeof(b), fmt, ap);
  if (n < 0) return;
  NeuroN6_DbgPrint(b);
}

VD66GY::VD66GY(VD66GYCarrier carrier)
{
  /* Runs at static initialisation, so it makes no HAL calls. */
  registerAsActive((uint32_t)carrier);
}

/* AE target in percent. */
int VD66GY::s_aeTarget = 25;

int  VD66GY::s_strobeMode     = STROBE_DISABLED;
bool VD66GY::s_strobeAlwaysOn = false;
bool VD66GY::s_strobeLatched  = false;

/* VD6G_Init writes the GPIO block once, in bringUp(), and the driver has no runtime GPIO
 * setter. A later strobe request is reported on the console and ignored. */
void VD66GY::strobeWarnIfLate(const char *what)
{
  if (s_strobeLatched) {
    char m[256];
    snprintf(m, sizeof(m),
             "[VD66GY] %s called AFTER sensor bring-up - IGNORED. The strobe "
             "GPIO is programmed once by VD6G_Init; set it before Vision.begin() "
             "(or via cfg.strobe).\r\n", what);
    NeuroN6_DbgPrint(m);
  }
}

int VD66GY::setStrobe(int mode)
{
  if (mode != STROBE_ENABLED && mode != STROBE_DISABLED) {
    NeuroN6_DbgPrint("[VD66GY] setStrobe: unknown mode - ignored.\r\n");
    return -1;
  }
  strobeWarnIfLate("setStrobe()");
  if (s_strobeLatched) {
    return -1;
  }
  /* Applied in bringUp(). */
  s_strobeMode = mode;
  return 0;
}

void VD66GY::setStrobeAlwaysOn(bool on)
{
  strobeWarnIfLate("setStrobeAlwaysOn()");
  if (s_strobeLatched) {
    return;
  }
  s_strobeAlwaysOn = on;
  if (on) {
    /* Vision.begin() calls setStrobe() only when cfg.strobe is not STROBE_DISABLED. */
    s_strobeMode = STROBE_ENABLED;
  }
}

void VD66GY::setAeTargetPercent(int percent)
{
  if (percent < 0)   percent = 0;
  if (percent > 100) percent = 100;
  s_aeTarget = percent;
}

/* Exposure loop state, read and written with the I2C lock held. Integer only: bringUp()
 * applies it on the capture task, where floating point would add FPU context to the
 * task's stack. */
static VD6G_ExposureMode_t s_expMode    = VD6G_EXPOSURE_AUTO;   /* requested */
static VD6G_ExposureMode_t s_expApplied = VD6G_EXPOSURE_AUTO;   /* on the sensor */
static uint32_t            s_expUs      = 0u;      /* manual exposure, 0 until set */
static uint32_t            s_gainQ8     = 0u;      /* manual gain x 256, 0 until set */
static bool                s_expLive    = false;   /* the sensor streams */

/* Line time in ns, or 0 on a read error. */
static uint32_t vd66_line_ns(void)
{
  uint16_t len = 0u;
  if (vd6g_cb_read16(&s_vd6g, VD66GY_REG_LINE_LENGTH, &len) != 0) return 0u;
  return (uint32_t)(((uint64_t)len * 1000000000ull) / VD66GY_PIXEL_CLOCK_HZ);
}

/* Exposure of the frame being output, in microseconds, or 0 on a read error. */
static uint32_t vd66_applied_exposure_us(void)
{
  uint16_t lines = 0u;
  const uint32_t ns = vd66_line_ns();
  if (ns == 0u || vd6g_cb_read16(&s_vd6g, VD66GY_REG_APPLIED_EXPOSURE, &lines) != 0) return 0u;
  return (uint32_t)(((uint64_t)lines * ns) / 1000u);
}

/* Registers are accessed directly: ST's setters add an error-code read and log lines to
 * each I2C error, and these run under the I2C lock, on the USB task too. Each returns -1
 * at the first error. */

/* Clamps lines to the manual range and writes them. */
static int vd66_write_exposure_lines(uint32_t lines)
{
  uint16_t frame = 0u, margin = 0u;
  if (vd6g_cb_read16(&s_vd6g, VD66GY_REG_FRAME_LENGTH, &frame) != 0 ||
      vd6g_cb_read16(&s_vd6g, VD66GY_REG_EXP_MARGIN, &margin) != 0) return -1;
  if ((uint32_t)margin < VD66GY_EXP_MIN_MARGIN ||
      (uint32_t)frame < (uint32_t)margin + VD66GY_EXP_END_LINES + VD66GY_EXP_MIN_LINES) return -1;
  const uint32_t hi = (uint32_t)frame - (uint32_t)margin - VD66GY_EXP_END_LINES;
  if (lines < VD66GY_EXP_MIN_LINES) lines = VD66GY_EXP_MIN_LINES;
  if (lines > hi)                   lines = hi;
  return vd6g_cb_write16(&s_vd6g, VD66GY_REG_MANUAL_EXPOSURE, (uint16_t)lines);
}

/* Rounds up to whole lines. */
static int vd66_write_exposure_us(uint32_t us)
{
  const uint32_t ns = vd66_line_ns();
  if (ns == 0u) return -1;
  return vd66_write_exposure_lines((uint32_t)(((uint64_t)us * 1000u + ns - 1u) / ns));
}

/* Manual gain ranges: analogue codes, and digital gains in 1/256 steps. */
typedef struct {
  uint32_t ag_min, ag_max;
  uint32_t dg_min, dg_max;
} vd66_gain_range_t;

static int vd66_read_gain_range(vd66_gain_range_t *r)
{
  uint8_t  ag_lo = 0u, ag_hi = 0u;
  uint16_t dg_lo = 0u, dg_hi = 0u;
  if (vd6g_cb_read8(&s_vd6g, VD66GY_REG_MIN_ANALOG_GAIN, &ag_lo) != 0 ||
      vd6g_cb_read8(&s_vd6g, VD66GY_REG_MAX_ANALOG_GAIN, &ag_hi) != 0 ||
      vd6g_cb_read16(&s_vd6g, VD66GY_REG_MIN_DIGITAL_GAIN, &dg_lo) != 0 ||
      vd6g_cb_read16(&s_vd6g, VD66GY_REG_MAX_DIGITAL_GAIN, &dg_hi) != 0) return -1;
  r->ag_min = ag_lo & 0x1Fu;
  r->ag_max = ag_hi & 0x1Fu;
  r->dg_min = dg_lo;
  r->dg_max = dg_hi;
  return 0;
}

/* Clamps an analogue code and a digital gain to r and writes them, the digital gain to
 * all four channels. */
static int vd66_write_gain_codes(uint32_t ag, uint32_t dg, const vd66_gain_range_t *r)
{
  if (ag < r->ag_min) ag = r->ag_min;
  if (ag > r->ag_max) ag = r->ag_max;
  if (dg < r->dg_min) dg = r->dg_min;
  if (dg > r->dg_max) dg = r->dg_max;
  if (vd6g_cb_write8(&s_vd6g, VD66GY_REG_MANUAL_ANALOG_GAIN, (uint8_t)ag) != 0) return -1;
  for (uint16_t ch = 0u; ch < 4u; ch++) {
    if (vd6g_cb_write16(&s_vd6g, (uint16_t)(VD66GY_REG_MANUAL_DIGITAL_GAIN + 2u * ch),
                        (uint16_t)dg) != 0) return -1;
  }
  return 0;
}

/* The analogue code c with 32 / (32 - c) at or below q8 / 256, then the digital gain
 * for the rest, in 1/256 steps. q8 is the gain x 256, 256 to 16384. */
static int vd66_write_gain(uint32_t q8)
{
  vd66_gain_range_t r;
  if (vd66_read_gain_range(&r) != 0) return -1;
  if (q8 < 256u) q8 = 256u;
  uint32_t ag = 32u - (8192u + q8 - 1u) / q8;   /* 0 to 31 */
  if (ag < r.ag_min) ag = r.ag_min;
  if (ag > r.ag_max) ag = r.ag_max;
  return vd66_write_gain_codes(ag, (q8 * (32u - ag) + 16u) / 32u, &r);
}

/* Copies the exposure and gains of the frame being output to the manual registers. A
 * status register still at 0, before the first frame, is not copied. */
static int vd66_hold_applied(void)
{
  uint16_t lines = 0u, dg = 0u;
  uint8_t  ag = 0u;
  if (vd6g_cb_read16(&s_vd6g, VD66GY_REG_APPLIED_EXPOSURE, &lines) != 0 ||
      vd6g_cb_read8(&s_vd6g, VD66GY_REG_APPLIED_ANALOG_GAIN, &ag) != 0 ||
      vd6g_cb_read16(&s_vd6g, VD66GY_REG_APPLIED_DIGITAL_GAIN, &dg) != 0) return -1;
  if (lines != 0u && vd66_write_exposure_lines(lines) != 0) return -1;
  if (dg != 0u) {
    vd66_gain_range_t r;
    if (vd66_read_gain_range(&r) != 0 || vd66_write_gain_codes(ag, dg, &r) != 0) return -1;
  }
  return 0;
}

/* Waits up to 200 ms for the status registers to hold the first frame's values. */
static void vd66_wait_first_frame(void)
{
  for (int i = 0; i < 40; i++) {
    uint16_t dg = 0u;
    if (vd6g_cb_read16(&s_vd6g, VD66GY_REG_APPLIED_DIGITAL_GAIN, &dg) == 0 && dg != 0u) return;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* Brings a streaming sensor to the requested loop state. exp and gain write the manual
 * values the caller set. Returns 0, or -1 when a register access fails. */
static int vd66_apply_exposure(bool exp, bool gain)
{
  if (!s_expLive) return 0;
  if (s_expMode == VD6G_EXPOSURE_MANUAL) {
    if (s_expApplied != VD6G_EXPOSURE_MANUAL) {
      if (vd66_hold_applied() != 0) return -1;
      exp  = exp  || (s_expUs != 0u);
      gain = gain || (s_gainQ8 != 0u);
    }
    if (exp  && vd66_write_exposure_us(s_expUs) != 0) return -1;
    if (gain && vd66_write_gain(s_gainQ8) != 0)       return -1;
  }
  if (s_expApplied != s_expMode) {
    if (vd6g_cb_write8(&s_vd6g, VD66GY_REG_EXP_MODE, (uint8_t)s_expMode) != 0) return -1;
    s_expApplied = s_expMode;
  }
  return 0;
}

/* Runs the loop, or holds what it last applied. A manual mode stays manual. Running the
 * loop drops the manual values, so the next manual mode starts from the loop's. */
static int vd66_set_auto(bool enable)
{
  n6_i2c_lock();
  if (enable) {
    s_expMode = VD6G_EXPOSURE_AUTO;
    s_expUs   = 0u;
    s_gainQ8  = 0u;
  } else if (s_expMode == VD6G_EXPOSURE_AUTO) {
    s_expMode = VD6G_EXPOSURE_FREEZE_AEALGO;
  }
  const int rc = vd66_apply_exposure(false, false);
  n6_i2c_unlock();
  return rc;
}

int VD66GY::setAutoExposure(bool enable) { return vd66_set_auto(enable); }
int VD66GY::setAutoGain(bool enable)     { return vd66_set_auto(enable); }

int VD66GY::setExposureUs(uint32_t us)
{
  n6_i2c_lock();
  s_expUs   = (us != 0u) ? us : 1u;
  s_expMode = VD6G_EXPOSURE_MANUAL;
  const int rc = vd66_apply_exposure(true, false);
  n6_i2c_unlock();
  return rc;
}

int VD66GY::setGain(float g)
{
  if (!(g >= 1.0f)) g = 1.0f;
  if (g > 64.0f)    g = 64.0f;
  const uint32_t q8 = (uint32_t)(g * 256.0f + 0.5f);
  n6_i2c_lock();
  s_gainQ8  = q8;
  s_expMode = VD6G_EXPOSURE_MANUAL;
  const int rc = vd66_apply_exposure(false, true);
  n6_i2c_unlock();
  return rc;
}

uint32_t VD66GY::exposureUs()
{
  n6_i2c_lock();
  const uint32_t us = s_expLive ? vd66_applied_exposure_us() : 0u;
  n6_i2c_unlock();
  return us;
}

float VD66GY::gain()
{
  float g = 0.0f;
  n6_i2c_lock();
  uint8_t  ag = 0u;
  uint16_t dg = 0u;
  if (s_expLive &&
      vd6g_cb_read8(&s_vd6g, VD66GY_REG_APPLIED_ANALOG_GAIN, &ag) == 0 &&
      vd6g_cb_read16(&s_vd6g, VD66GY_REG_APPLIED_DIGITAL_GAIN, &dg) == 0 && ag < 32) {
    g = (32.0f / (float)(32 - ag)) * ((float)dg / 256.0f);
  }
  n6_i2c_unlock();
  return g;
}

/* Maps a resolution to the nearest VD66GY mode and writes that mode's size to w and h. */
static VD6G_Res_t vd66_map_res(CameraResolutionN6 res,
                               uint16_t *w = nullptr, uint16_t *h = nullptr)
{
  VD6G_Res_t mode; uint16_t mw, mh;
  switch (res) {
    case CAMERA_QVGA:      mode = VD6G_RES_QVGA_320_240;      mw = 320u;  mh = 240u;  break;
    case CAMERA_XGA:       mode = VD6G_RES_XGA_1024_768;      mw = 1024u; mh = 768u;  break;
    case CAMERA_720P:      mode = VD6G_RES_PORTRAIT_1120_720; mw = 1120u; mh = 720u;  break;
    case CAMERA_1080P:
    case CAMERA_QSXGA:
    case CAMERA_VD66_FULL: mode = VD6G_RES_FULL_1120_1364;    mw = 1120u; mh = 1364u; break;
    case CAMERA_WVGA:
    case CAMERA_VGA:
    default:               mode = VD6G_RES_VGA_640_480;       mw = 640u;  mh = 480u;  break;
  }
  /* The encoder floors the height to a multiple of 8, so 1364 lines code as 1360. */
  if (w) *w = mw;
  if (h) *h = mh;
  return mode;
}

/* Maps an orientation to the driver's mode. Automatic is neither, on both carriers. */
static VD6G_MirrorFlip_t vd66_map_flip(CameraMirrorFlipN6 flip)
{
  switch (flip) {
    case CAMERA_FLIP:        return VD6G_FLIP;
    case CAMERA_MIRROR:      return VD6G_MIRROR;
    case CAMERA_MIRROR_FLIP: return VD6G_MIRROR_FLIP;
    default:                 return VD6G_MIRROR_FLIP_NONE;
  }
}

/* Line length in pixel clocks, 0 for the sensor's default of about 1236. At the default the
 * encoder falls behind on a 1120-wide mode and overflows. */
static int vd66_line_len(uint16_t width)
{
  return (width > 1024u) ? 1528 : 0;
}

/* frame_length = 160.8 MHz / (line_length x fps) must stay above about 1450 rows. The driver
 * does not clamp it, so a rate above the limit does not stream. */
int VD66GY::maxFps(CameraResolutionN6 res)
{
  switch (res) {
    case CAMERA_QVGA:
    case CAMERA_WVGA:
    case CAMERA_VGA:       return 88;    /* frame_length limit */
    case CAMERA_XGA:
    case CAMERA_720P:      return 60;    /* VENC limit */
    case CAMERA_1080P:
    case CAMERA_QSXGA:
    case CAMERA_VD66_FULL: return 45;    /* VENC limit */
    default:               return 60;
  }
}

int VD66GY::init(CameraResolutionN6 res, CameraFpsN6 fps, CameraMirrorFlipN6 flip,
                 uint16_t &out_w, uint16_t &out_h)
{
  _res = res;
  _flip = flip;
  _fps = (fps == FPS_DEFAULT) ? maxFps(res) : (int)fps;

  /* Reports the mode's size only. bringUp() starts the sensor. */
  vd66_map_res(res, &out_w, &out_h);
  return 0;
}

void VD66GY::bringUp()
{
  if (!_broughtUp) {
    _broughtUp = true;

    n6_vision_deferred_pipes();

    s_vd6g.shutdown_pin = vd6g_cb_shutdown;
    s_vd6g.read8        = vd6g_cb_read8;
    s_vd6g.read16       = vd6g_cb_read16;
    s_vd6g.read32       = vd6g_cb_read32;
    s_vd6g.write8       = vd6g_cb_write8;
    s_vd6g.write16      = vd6g_cb_write16;
    s_vd6g.write32      = vd6g_cb_write32;
    s_vd6g.write_array  = vd6g_cb_write_array;
    s_vd6g.delay        = vd6g_cb_delay;
    s_vd6g.log          = vd6g_cb_log;

    VD6G_Config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.ext_clock_freq_in_hz = 12000000;                       /* on-board oscillator */
    cfg.resolution           = vd66_map_res(_res);
    cfg.frame_rate           = _fps;
    cfg.flip_mirror_mode     = vd66_map_flip(_flip);
    {
      uint16_t mode_w = 0u, mode_h = 0u;
      (void)vd66_map_res(_res, &mode_w, &mode_h);
      cfg.line_len           = vd66_line_len(mode_w);
    }
    cfg.patgen               = VD6G_PATGEN_DISABLE;
    cfg.flicker              = VD6G_FLICKER_FREE_50HZ;        /* 50 Hz mains */
    cfg.pixel_depth          = VD6G_COLOR_DEPTH_RAW8;
    cfg.out_itf.datalane_nb                    = 2;
    cfg.out_itf.clock_lane_swap_enable         = 0;
    cfg.out_itf.data_lane0_swap_enable         = 0;
    cfg.out_itf.data_lane1_swap_enable         = 0;
    cfg.out_itf.data_lanes_mapping_swap_enable = 1;
    for (int gi = 0; gi < VD6G_GPIO_NB; gi++) cfg.gpio_ctrl[gi] = VD6G_GPIO_GPIO_IN;

    /* Strobe LED on the sensor's GPIO0. VD6G_GPIO_STROBE pulses with each exposure. */
    if (s_strobeMode == STROBE_ENABLED) {
      cfg.gpio_ctrl[VD6G_GPIO_0] = s_strobeAlwaysOn
                                     ? (VD6G_GPIO_OUT | VD6G_GPIO_HIGH)
                                     : VD6G_GPIO_STROBE;
      NeuroN6_DbgPrint(s_strobeAlwaysOn
                         ? "[VD66GY] strobe: GPIO0 steady HIGH (always-on).\r\n"
                         : "[VD66GY] strobe: GPIO0 hardware STROBE (pulses with exposure).\r\n");
    }
    s_strobeLatched = true;
    cfg.exposure_mode        = VD6G_EXPOSURE_AUTO;

    const int initRet = VD6G_Init(&s_vd6g, &cfg);
    _bayer = (int)s_vd6g.bayer;
    if (initRet == 0) {
      /* VD6G_Init detects the Bayer order: GRBG on a colour VD66GY, changed by a mirror or
       * flip. */
      {
        SensorFormat sf;
        getFormat(sf);
        n6_dcmipp_set_bayer_order((int)sf.bayer_order);
      }
      /* Order: stream on, the ROI write, then the PIPE1 start against a live CSI link. */
      if (VD6G_Start(&s_vd6g) == 0) {
        /* Crop on the sensor: a PIPE1 crop corrupts the RAW8 demosaic path. ROI registers are
         * X start 0x045e, X end 0x0460, Y start 0x0462, Y end 0x0464, rewritten by VD6G_Start. */
        {
          uint16_t cap_w = 0u, cap_h = 0u;
          (void)vd66_map_res(_res, &cap_w, &cap_h);
          uint16_t x0 = 0u, y0 = 0u;
          if (vd6g_cb_read16(&s_vd6g, 0x045eu, &x0) == 0 &&
              vd6g_cb_read16(&s_vd6g, 0x0462u, &y0) == 0) {
            (void)vd6g_cb_write16(&s_vd6g, 0x0460u, (uint16_t)(x0 + cap_w - 1u));
            (void)vd6g_cb_write16(&s_vd6g, 0x0464u, (uint16_t)(y0 + cap_h - 1u));
          }
        }
        (void)VD6G_SetBrightnessLevel(&s_vd6g, s_aeTarget);

        n6_linebuf_prime_pipe();

        g_n6_sensor_streaming = 1u;

        vd66_isp_auto_start(_bayer);

        /* Exposure and gain requested before bring-up. A manual mode copies the status
         * registers, which hold values from the first frame on. */
        vd66_wait_first_frame();
        n6_i2c_lock();
        s_expLive = true;
        (void)vd66_apply_exposure(false, false);
        n6_i2c_unlock();
      } else {
        NeuroN6_Notice("[VD66GY] the sensor did not start streaming.\r\n");
      }
    } else {
      char m[112];
      snprintf(m, sizeof(m), "[VD66GY] the sensor did not start (VD6G_Init %d): check the "
               "camera module and its cable.\r\n", initRet);
      NeuroN6_Notice(m);
    }
  }
}

void VD66GY::getFormat(SensorFormat &out)
{
  uint8_t order;
  switch (_bayer) {
    case VD6G_BAYER_GRBG: order = 1u; break;
    case VD6G_BAYER_GBRG: order = 2u; break;
    case VD6G_BAYER_BGGR: order = 3u; break;
    default:              order = 0u; break;   /* mono or RGGB */
  }
  out.pixel_format = N6_PIXFMT_RAW8;
  out.color_space  = N6_COLORSPACE_NONE;
  out.bayer_order  = order;
}

void VD66GY::streamOn(void)  { /* VD6G_Start runs in bringUp(). */ }
void VD66GY::streamOff(void) { }

uint32_t VD66GY::csiLaneRateMbps(void)
{
  /* The link runs at 804 Mbps per lane. 800 selects DCMIPP_CSI_PHY_BT_800. */
  return 800u;
}
