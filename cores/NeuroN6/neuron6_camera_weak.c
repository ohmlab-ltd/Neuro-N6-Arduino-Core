/**
 ******************************************************************************
 * @file    neuron6_camera_weak.c
 * @brief   Default value of _neuron6_csi_lane_mapping: no camera declared.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include <stdint.h>

#include "NeuroN6_app.h"

/* In its own translation unit so the reads in NeuroN6.c are not folded to the
 * default. The sensor object's constructor writes it. */
__attribute__((weak)) uint32_t _neuron6_csi_lane_mapping = NEURON6_CAMERA_NONE;
