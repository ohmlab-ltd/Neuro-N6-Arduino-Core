/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* Host stand-in for NeuroN6.h: the NN-crop geometry and the overlay buffer. */
#ifndef NEURON6_H_SHIM
#define NEURON6_H_SHIM

#include <stdint.h>
#include <stdbool.h>

uint16_t *OVL_GetBackBuffer(void);
void      OVL_Commit(void);

extern int16_t  g_lcd_camera_x;
extern int16_t  g_lcd_camera_y;
extern uint16_t g_lcd_camera_w;
extern uint16_t g_lcd_camera_h;
extern int16_t  g_nn_crop_x;
extern int16_t  g_nn_crop_y;
extern uint16_t g_nn_crop_w;
extern uint16_t g_nn_crop_h;
extern uint16_t g_camera_full_w;
extern uint16_t g_camera_full_h;

static inline float nn_norm_to_cam_norm_x(float xn)
{
  return ((float)g_nn_crop_x + xn * (float)g_nn_crop_w) / (float)g_camera_full_w;
}
static inline float nn_norm_to_cam_norm_y(float yn)
{
  return ((float)g_nn_crop_y + yn * (float)g_nn_crop_h) / (float)g_camera_full_h;
}
static inline int nn_norm_to_lcd_x(float xn)
{
  return (int)((float)g_lcd_camera_x + nn_norm_to_cam_norm_x(xn) * (float)g_lcd_camera_w);
}
static inline int nn_norm_to_lcd_y(float yn)
{
  return (int)((float)g_lcd_camera_y + nn_norm_to_cam_norm_y(yn) * (float)g_lcd_camera_h);
}

#endif
