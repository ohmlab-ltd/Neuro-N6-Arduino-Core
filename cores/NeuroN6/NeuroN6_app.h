/**
 * @file    NeuroN6_app.h
 * @brief   C API: camera, NPU, JPEG and buffer allocation.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#ifndef __NEURON6_APP_H
#define __NEURON6_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined ( __ICCARM__ )
#  define CMSE_NS_CALL  __cmse_nonsecure_call
#  define CMSE_NS_ENTRY __cmse_nonsecure_entry
#else
#  define CMSE_NS_CALL  __attribute((cmse_nonsecure_call))
#  define CMSE_NS_ENTRY __attribute((cmse_nonsecure_entry))
#endif

#include "stm32n6xx_hal.h"
#include "main.h"
#include "ll_aton_rt_user_api.h"
#include <stdbool.h>
#include <stdint.h>

#if defined ( __ICCARM__ )
typedef void (CMSE_NS_CALL *funcptr)(void);
#else
typedef void CMSE_NS_CALL (*funcptr)(void);
#endif

typedef funcptr funcptr_NS;

#define GREEN_LED_Pin        GPIO_PIN_6
#define GREEN_LED_GPIO_Port  GPIOD
#define BLUE_LED_Pin         GPIO_PIN_1
#define BLUE_LED_GPIO_Port   GPIOC
#define RED_LED_Pin          GPIO_PIN_11
#define RED_LED_GPIO_Port    GPIOE

/* Valid after NeuroN6_Setup(). */
extern I2C_HandleTypeDef hi2c1;

/* Recursive mutex around every hi2c1 transaction. Both do nothing before the
 * scheduler starts and inside ISRs. */
void n6_i2c_lock(void);
void n6_i2c_unlock(void);

/* Holds the same mutex from claim() to release(), blocking every other
 * n6_i2c_lock() caller. Both must run on the same task. */
void n6_i2c_claim(void);
void n6_i2c_release(void);

/* Non-blocking. Returns 1 when the bus was taken, to be followed by
 * n6_i2c_unlock(), or 0 when it is busy. Returns int because NeuroN6.c's
 * includes retypedef bool. */
int n6_i2c_trylock(void);

/* Called from main.cpp. */
void NeuroN6_Setup(void);
void NeuroN6_App(void);

/* RCC->RSR, and PWR->CPUCR & PWR_CPUCR_SBF, as this boot found them.
 * NeuroN6_Setup() reads and clears both first. */
uint32_t n6_boot_rsr(void);
uint32_t n6_boot_sbf(void);

/* ST's declaration leaves struct fields to their zero default, which
 * -Wextra reports. */
#ifdef __cplusplus
#define NEURON6_DECLARE_MODEL(name)                                    \
  _Pragma("GCC diagnostic push")                                       \
  _Pragma("GCC diagnostic ignored \"-Wmissing-field-initializers\"")   \
  extern "C" {                                                         \
    LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(name)              \
  }                                                                    \
  _Pragma("GCC diagnostic pop")
#else
#define NEURON6_DECLARE_MODEL(name)                                    \
  _Pragma("GCC diagnostic push")                                       \
  _Pragma("GCC diagnostic ignored \"-Wmissing-field-initializers\"")   \
  LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(name)                \
  _Pragma("GCC diagnostic pop")
#endif

void NPU_Init(void);

/* Waits up to 1 s for the frame NN_feed_start() armed, then runs the model on
 * it. Returns 1, or 0 when no frame arrives. */
int NPU_Run_Inference(NN_Instance_TypeDef *instance);

/* Runs the model on its input buffer as it stands, without a camera frame. The
 * caller cleans the D-cache over that buffer first. */
void NPU_Run_Inference_Direct(NN_Instance_TypeDef *instance);

/* CSI data-lane mapping, set by the sensor object and read by CSI_Init(). */
extern uint32_t _neuron6_csi_lane_mapping;

/* No sensor declared. NeuroN6_Setup() then skips CSI_Init(). */
#define NEURON6_CAMERA_NONE   0xFFFFFFFFu

/* Input format of the DCMIPP pipes and the JPEG encoder, as HAL constants. */
typedef struct {
  uint32_t csi_data_type;       /* DCMIPP_DT_* */
  uint32_t packer_format_usb;   /* DCMIPP_PIXEL_PACKER_FORMAT_* for PIPE1 */
  uint32_t jpeg_enc_format;     /* JPEGENC_* frame type */
  uint8_t  bytes_per_pixel;     /* PIPE1 pitch is width * bytes_per_pixel */
  uint8_t  yuv_swap;            /* 1 enables the PIPE1 YUV byte swap */
  uint8_t  needs_yuv_to_rgb;    /* 1: BT.601 YUV to RGB on NN and LCD pipes */
} n6_sensor_format_t;

extern n6_sensor_format_t g_n6_sensor_format;

/* VisionConfig.isp_enabled, set by Vision.begin(). */
extern volatile uint8_t g_isp_tuning_mode;

/* The DCMIPP HAL handle, for the ST ISP middleware's ISP_Init. */
void *n6_dcmipp_handle(void);

/* Called before each start of pipe 0, 1 or 2: clears its event and overrun
 * flags, whose interrupts HAL_DCMIPP_CSI_PIPE_Start enables without clearing
 * them, and its snapshot bit, which the start only ORs in. */
void n6_dcmipp_prepare_start(uint32_t pipe);

/* Runs the sensor's deferred bringUp(). Called by the capture task. */
void n6_sensor_bringup(void);

/* 'CAM0' capture control: a 12-byte packet on the vendor bulk-OUT endpoint.
 *   offset 0   u32 magic 'CAM0'
 *   offset 4   u8  flags        N6_CAM0_FLAG_*, the auto states wanted
 *   offset 5   u32 exposure_us
 *   offset 9   u16 gain_x256    256 is 1.0x
 *   offset 11  u8  apply_mask   N6_CAM0_APPLY_*, the fields this packet sets */
enum {
  N6_CAM0_FLAG_AUTO_EXPOSURE  = 0x01u,
  N6_CAM0_FLAG_AUTO_GAIN      = 0x02u,
  N6_CAM0_FLAG_AUTO_AWB       = 0x04u,

  N6_CAM0_APPLY_AUTO_EXPOSURE = 0x01u,
  N6_CAM0_APPLY_AUTO_GAIN     = 0x02u,
  N6_CAM0_APPLY_AUTO_AWB      = 0x04u,
  N6_CAM0_APPLY_EXPOSURE_US   = 0x08u,
  N6_CAM0_APPLY_GAIN          = 0x10u,
};

/* Bitmask of ImageSensor::N6_CAM_CAP_*, 0 with no sensor. Announced as
 * META_CAM_CAPS. */
uint32_t n6_sensor_control_caps(void);

/* Applies a 'CAM0' packet. Returns 0, or -1 with no sensor. Controls the
 * sensor lacks are ignored. */
int n6_sensor_apply_cam0(uint8_t flags, uint32_t exposure_us,
                         uint16_t gain_x256, uint8_t apply_mask);

/* Fills g_n6_sensor_format from N6PixelFormat and N6ColorSpace values. Call
 * before the DCMIPP_*_Init functions. */
void n6_apply_sensor_format(int pixel_format, int color_space, int bayer_order);

/* Sets the RAW8 demosaicer Bayer order at run time: 0 RGGB, 1 GRBG, 2 GBRG,
 * 3 BGGR. */
void n6_dcmipp_set_bayer_order(int bayer_order);

/* Sets the CSI-2 PHY receiver bit rate from the per-lane rate in Mbps. 0 keeps
 * the boot default. */
void n6_csi_set_phy_bitrate(uint32_t mbps_per_lane);

/* Placement of the VENC line-buffer band and the LCD framebuffer. */
typedef enum {
  N6_MEM_AUTO    = 0,   /* AXISRAM, or PSRAM when it does not fit */
  N6_MEM_AXISRAM = 1,   /* fails when it does not fit */
  N6_MEM_PSRAM   = 2,
} n6_mem_loc_t;

/* Heap ceiling read by _sbrk, 0 for the default. Lowered to the base of the
 * AXISRAM pool. */
extern volatile uint32_t g_axisram_heap_ceiling;

/* Allocates the camera DMA buffers: band, JPEG and LCD. Call before the DCMIPP
 * pipe inits. Returns -1 when the JPEG or LCD buffers do not fit. */
int n6_alloc_camera_buffers(uint32_t cam_w, uint32_t cam_h, int lcd_enabled,
                            n6_mem_loc_t band_loc, n6_mem_loc_t framebuf_loc);

/* LTDC pixel-clock divider from PLL4 at 400 MHz, read by HAL_LTDC_MspInit. */
extern volatile uint32_t g_ltdc_clk_divider;

/* Sets the panel refresh rate in Hz, clamped to 30..60. Call before the LTDC
 * init. */
void n6_lcd_set_fps(int fps);

/* USB frame size in pixels. */
extern uint32_t usb_frame_width;
extern uint32_t usb_frame_height;

/* DCMIPP PIPE1, the JPEG pipe. */
void DCMIPP_USB_Init(uint32_t width, uint32_t height);

/* DCMIPP PIPE0, the LCD pipe. */
void DCMIPP_LCD_Init(uint32_t width, uint32_t height);

void LCD_Init(void);

typedef enum {
  FEED_SNAPSHOT   = 0,
  FEED_CONTINUOUS = 1,
} Feed_Mode_t;

/* DCMIPP PIPE2, the NN pipe. Scales the full frame to width x height. */
void DCMIPP_NN_Init(uint32_t width, uint32_t height);

/* As DCMIPP_NN_Init(), with a crop in camera pixels applied before the
 * downsize. The crop must fit the frame. A crop_w or crop_h of 0 is no crop. */
void DCMIPP_NN_InitWithCrop(uint32_t width, uint32_t height,
                            uint32_t crop_x, uint32_t crop_y,
                            uint32_t crop_w, uint32_t crop_h);

/* As DCMIPP_NN_Init(), keeping the frame's aspect: the whole frame is scaled by
 * min(width / frame width, height / frame height), rounded to whole pixels and
 * centred, and the rest of the input is grey 114. The picture starts on a
 * 16-pixel boundary, which the DCMIPP's address alignment needs. */
void DCMIPP_NN_InitLetterbox(uint32_t width, uint32_t height);

/* Starts PIPE1 once, line-wrapped into the band. Call once after
 * DCMIPP_USB_Init(). */
void n6_linebuf_prime_pipe(void);

/* 1 when the sensor streams and the capture task may arm. Defaults to 1. */
extern volatile uint8_t g_n6_sensor_streaming;

/* Read by the capture task on its next pass. Not for sketches.
 *   g_lb_dcmipp_continuous   0: the capture reconfigures and starts the DCMIPP
 *   g_lb_venc_reset_pending  1: it also resets and re-initialises the VENC */
extern volatile uint8_t g_lb_dcmipp_continuous;
extern volatile uint8_t g_lb_venc_reset_pending;

/* Stop and restart the sensor's pixel output, leaving the pipeline up. pause()
 * waits 150 ms for a capture in flight. Both return false with no sensor or
 * one that defers streaming. */
bool n6_stream_pause(void);
bool n6_stream_resume(void);
/* 1 runs the JPEG pipe and VENC, 0 keeps both off. Set by Vision.begin(). */
extern volatile uint8_t g_n6_usb_enabled;
/* 0 disables the LCD overlay layer and its draw. Console key 'o' toggles it. */
extern volatile uint8_t g_lcd_overlay_on;

/* Pipe geometry kept by Vision.begin() for a defersStreaming() sensor, whose
 * bringUp() applies it. */
typedef struct {
  uint32_t cam_w, cam_h;
  uint32_t nn_w, nn_h;
  uint32_t crop_x, crop_y, crop_w, crop_h;  /* camera pixels */
  uint8_t  letterbox;            /* 1 uses DCMIPP_NN_InitLetterbox(), no crop */
  uint32_t mbps;                 /* CSI lane rate, 0 to keep the default */
  uint8_t  lcd;                  /* 1 also runs DCMIPP_LCD_Init() */
  uint8_t  valid;                /* 1 when a config is waiting */
} n6_deferred_cfg_t;
extern n6_deferred_cfg_t g_n6_deferred_cfg;

/* Applies g_n6_deferred_cfg once. Called from bringUp(). */
void n6_vision_deferred_pipes(void);

/* Starts the NN pipe's capture into the model's input buffer. Returns 1, or 0
 * when cfg.nn is larger than the model's input. NPU_Run_Inference() releases
 * the pipe. */
int NN_feed_start(Feed_Mode_t mode, NN_Instance_TypeDef *instance);

/* Frame access without a model */

/* Grabs one still from the NN pipe into dst: RGB888, row-major, cfg.nn.width x
 * cfg.nn.height x 3 bytes. dst must be 32-byte aligned and dst_size is checked.
 * Returns 1, or 0 with no usable frame in dst. Waits up to 250 ms for the NN
 * pipe, then up to 250 ms for the frame. */
int NN_capture_snapshot(void *dst, uint32_t dst_size);

/* The live LCD camera layer: RGB565, g_camera_full_w x g_camera_full_h, or
 * NULL when no LCD camera feed is running. The DCMIPP writes it continuously,
 * so a slow read can tear. */
const uint16_t *LCD_GetCameraBuffer(void);

/* The NN input size, cfg.nn, set by DCMIPP_NN_InitWithCrop() or
 * DCMIPP_NN_InitLetterbox() on success. g_nn_pipe_ready is 0 until then. */
extern uint16_t g_nn_out_w;
extern uint16_t g_nn_out_h;
extern uint8_t  g_nn_pipe_ready;

/* Starts the PIPE0 LCD feed. */
void LCD_feed_start(Feed_Mode_t mode);

/* Non-cacheable memory from the PSRAM pool, for DMA buffers. Never freed.
 * Returns 0 when the pool is exhausted. */
void *n6_psram_alloc(uint32_t bytes);

/* One-shot JPEG without the Vision pipeline. Call n6_jpeg_band_setup(),
 * encoder_jpeg_linebuf_ensure_init() and n6_linebuf_prime_pipe() once, in that
 * order, then n6_jpeg_capture_one() per capture. setup returns 0 when the PSRAM
 * pool is exhausted. capture returns the JPEG length, or 0 on failure. */
int      n6_jpeg_band_setup(uint32_t cam_w, uint32_t cam_h);
uint32_t n6_jpeg_capture_one(uint8_t *out_buf);
void     encoder_jpeg_linebuf_ensure_init(int32_t width, int32_t height, uint32_t szOut);

void Error_Handler(void);

/* Clears the FSBL's double-press key, so that a reset in the RTC second the
 * app started in does not enter DFU. Called before a reset that is not a
 * bootloader request. */
void n6_boot_press_clear(void);

/* 1 when the next boot enters the FSBL's DFU mode (a bootloader request is
 * pending). */
int n6_boot_dfu_pending(void);

/* Fault dump over CDC. Never returns. frame is {r0, r1, r2, r3, r12, LR, PC,
 * xPSR}. tag 1..6 is the fault type. */
void NeuroN6_FaultEntry(uint32_t *frame, uint32_t exc_return, uint32_t tag);

#ifdef __cplusplus
}
#endif

#endif /* __NEURON6_APP_H */
