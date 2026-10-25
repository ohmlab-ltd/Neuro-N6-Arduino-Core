/**
 ******************************************************************************
 * @file    EdgeViewFrames.h
 * @brief   Hands the core's encoded JPEG frames to this sketch through the
 *          Ethernet transport slot.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef EDGEVIEW_FRAMES_H
#define EDGEVIEW_FRAMES_H

#include <Arduino.h>

class EdgeViewFramesClass {
public:
  /* True when Vision.begin() ran with cfg.transport = TRANSPORT_ETHERNET. */
  bool begin();
  bool active() const;

  /* Waits up to waitMs for a frame. On true, *jpeg points into the core's
   * PSRAM buffer and stays valid until release(). */
  bool grab(const uint8_t **jpeg, uint32_t *len, uint32_t waitMs = 0);

  /* Idempotent. A buffer that is not given back stalls the pipeline after two
   * frames. */
  void release();

  bool holding() const;

  uint32_t framesSeen() const;      /* messages dequeued since boot */
  uint32_t framesSkipped() const;   /* dequeued but empty */
};

extern EdgeViewFramesClass EdgeViewFrames;

#endif /* EDGEVIEW_FRAMES_H */
