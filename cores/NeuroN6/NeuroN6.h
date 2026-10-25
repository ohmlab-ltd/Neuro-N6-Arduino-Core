/**
  ******************************************************************************
  * @file    NeuroN6.h
  * @brief   Public API for the Neuro N6 board support package.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025-2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  *
  ******************************************************************************
  */

#pragma once
#include <stdbool.h>
#include "NeuroN6_Version.h"
#include "main.h"
#include "NeuroN6_app.h"
#include "NeuroN6_FrameQueue.h"
#include "n6_host_contract.h"

#ifdef __cplusplus
extern "C" {
#endif

void NeuroN6_Setup(void);
void NeuroN6_App(void);
/* Serves USB until the USB task starts. main() calls it before setup(). */
void NeuroN6_UsbBootPumpStart(void);

/* Run LED, purple, on by default. Error paths use red. */
void NeuroN6_SetRunLedEnabled(bool enabled);
bool NeuroN6_GetRunLedEnabled(void);

/* DCMIPP PIPE1, the JPEG pipe. */
void DCMIPP_USB_Init(uint32_t width, uint32_t height);

/* DCMIPP PIPE0, the LCD pipe. */
void DCMIPP_LCD_Init(uint32_t width, uint32_t height);

void LCD_Init(void);

/* USB frame size in pixels. Read-only. */
extern uint32_t usb_frame_width;
extern uint32_t usb_frame_height;

/* OVL_GetBackBuffer() waits up to 50 ms for a committed LTDC reload to retire,
 * and returns NULL without an LCD. */
uint16_t *OVL_GetBackBuffer(void);
void      OVL_Commit(void);

/* Count of draws that waited for a reload. */
uint32_t  OVL_GetStallCount(void);

/* Frees an I2C bus a device holds after a reset mid-transfer: up to nine SCL
 * pulses while SDA is low, each ending in a STOP, then a START and a STOP. Call
 * with the pins' GPIO clocks on and before the I2C peripheral takes them.
 * Returns the pulses sent. */
uint32_t n6_i2c_bus_recover(GPIO_TypeDef *sda_port, uint16_t sda_pin,
                            GPIO_TypeDef *scl_port, uint16_t scl_pin);

/* Sends one TLV as a meta-only vendor frame, or returns false when the
 * transmitter is busy. buf needs three bytes of headroom before the value and
 * must outlive the transfer. */
bool NeuroN6_MetaAnnounce(uint8_t *buf, uint8_t tag, uint16_t len);
/* Writes a string to the USB CDC console, if connected. */
void NeuroN6_DbgPrint(const char *msg);

/* Prints a warning, and again to each host that connects later. Six at most. */
void NeuroN6_Notice(const char *msg);

/* Camera image rectangle on the 800x480 panel, in panel pixels. */
extern int16_t  g_lcd_camera_x;
extern int16_t  g_lcd_camera_y;
extern uint16_t g_lcd_camera_w;
extern uint16_t g_lcd_camera_h;

/* The camera pixels the NN input covers: the crop, the full frame, or for a
 * letterbox a window larger than the frame, starting left of or above it. */
extern int16_t  g_nn_crop_x;
extern int16_t  g_nn_crop_y;
extern uint16_t g_nn_crop_w;
extern uint16_t g_nn_crop_h;

/* Camera frame size in pixels. */
extern uint16_t g_camera_full_w;
extern uint16_t g_camera_full_h;

/* NN-normalised [0..1] to camera-normalised [0..1], through the crop window. */
static inline float nn_norm_to_cam_norm_x(float xn)
{
  return ((float)g_nn_crop_x + xn * (float)g_nn_crop_w) / (float)g_camera_full_w;
}
static inline float nn_norm_to_cam_norm_y(float yn)
{
  return ((float)g_nn_crop_y + yn * (float)g_nn_crop_h) / (float)g_camera_full_h;
}

/* NN-normalised [0..1] to an LCD pixel inside the camera rectangle. */
static inline int nn_norm_to_lcd_x(float xn)
{
  return (int)((float)g_lcd_camera_x + nn_norm_to_cam_norm_x(xn) * (float)g_lcd_camera_w);
}
static inline int nn_norm_to_lcd_y(float yn)
{
  return (int)((float)g_lcd_camera_y + nn_norm_to_cam_norm_y(yn) * (float)g_lcd_camera_h);
}

/* True when a host subscribes to bit, one of the N6_EVIDENCE_* classes. */
bool n6_host_evidence_wanted(uint32_t bit);

#ifdef __cplusplus
}
#endif

#include "NeuroN6_ROI.h"
#include "NeuroN6_Tracker.h"

#include "NeuroN6_Tune.h"

/* Included last: it needs the C declarations above. */
#ifdef __cplusplus
#include "NeuroN6_Vision.h"
#endif
