/**
 * @file    A7683EVideo.cpp
 * @brief   A7683EVideo transport hooks and queue drain.
 * @copyright Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "A7683EVideo.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "NeuroN6_FrameQueue.h"

A7683EVideoClass A7683EVideo;

/* Read by the core producer before it encodes: true keeps the JPEG stable. */
static volatile bool s_busy    = false;
static bool          s_holding = false;
static bool          s_started = false;
static uint32_t      s_seen    = 0;
static uint32_t      s_skipped = 0;

/* Core transport hooks. They need C linkage to override the weak defaults. */
extern "C" {

int EthernetN6_Start(int transport_mode)
{
  (void)transport_mode;
  /* Returning 0 keeps Vision.begin() on TRANSPORT_ETHERNET. */
  s_started = true;
  return 0;
}

bool     EthernetN6_TxBusy(void)       { return s_busy; }
uint32_t EthernetN6_GetIPv4_BE(void)   { return 0u; }   /* no IP */
uint16_t EthernetN6_GetIPv4_Port(void) { return 0u; }

} /* extern "C" */

bool A7683EVideoClass::begin() { return s_started; }

bool A7683EVideoClass::active() const
{
  return s_started && (g_n6_active_transport == TRANSPORT_ETHERNET);
}

bool     A7683EVideoClass::holding() const       { return s_holding; }
uint32_t A7683EVideoClass::framesSeen() const    { return s_seen; }
uint32_t A7683EVideoClass::framesSkipped() const { return s_skipped; }

bool A7683EVideoClass::grab(const uint8_t **jpeg, uint32_t *len, uint32_t waitMs)
{
  if (jpeg == NULL || len == NULL) {
    return false;
  }
  if (s_holding) {
    return false;
  }
  if (g_n6_video_eth_queue == NULL) {
    return false;                  /* Vision.begin() has not run */
  }

  n6_frame_msg_t msg;
  if (xQueueReceive(g_n6_video_eth_queue, &msg, pdMS_TO_TICKS(waitMs)) != pdTRUE) {
    return false;
  }
  s_seen++;

  /* A meta-only message owns no buffer, so it needs no completion. */
  if (msg.jpeg_ptr == NULL || msg.jpeg_len == 0u) {
    s_skipped++;
    return false;
  }

  s_busy    = true;
  s_holding = true;
  *jpeg     = msg.jpeg_ptr;
  *len      = msg.jpeg_len;
  return true;
}

void A7683EVideoClass::release()
{
  if (!s_holding) {
    return;
  }
  s_holding = false;
  s_busy    = false;
  n6_video_tx_complete();          /* reopens the encoder */
}
