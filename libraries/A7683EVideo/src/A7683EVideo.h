/**
 * @file    A7683EVideo.h
 * @brief   Hands the core's JPEG frames to a sketch as the Ethernet transport.
 * @copyright Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef A7683E_VIDEO_H
#define A7683E_VIDEO_H

#include <Arduino.h>

/* Cannot be linked alongside <EthernetN6.h>, which uses the same hooks. */
class A7683EVideoClass {
public:
  /* Optional. Reports whether Vision.begin() ran with TRANSPORT_ETHERNET. */
  bool begin();
  bool active() const;

  /* Takes the next frame, waiting up to waitMs. On true, *jpeg points into the
   * core's buffer and stays valid until release(). */
  bool grab(const uint8_t **jpeg, uint32_t *len, uint32_t waitMs = 0);

  /* Gives the buffer back. Required for every frame: the encoder reopens only
   * on release. */
  void release();

  bool holding() const;

  uint32_t framesSeen() const;      /* messages dequeued since boot */
  uint32_t framesSkipped() const;   /* dequeued but empty */
};

extern A7683EVideoClass A7683EVideo;

#endif /* A7683E_VIDEO_H */
