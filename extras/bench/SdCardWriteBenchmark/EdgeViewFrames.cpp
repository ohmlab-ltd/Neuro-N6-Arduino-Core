/**
 ******************************************************************************
 * @file    EdgeViewFrames.cpp
 * @brief   Transport hooks and queue drain.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "EdgeViewFrames.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "NeuroN6_FrameQueue.h"

EdgeViewFramesClass EdgeViewFrames;

/* The core reads s_busy through EthernetN6_TxBusy before it encodes. */
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
  /* 0 makes Vision.begin() keep TRANSPORT_ETHERNET instead of USB. */
  s_started = true;
  return 0;
}

bool     EthernetN6_TxBusy(void)      { return s_busy; }
uint32_t EthernetN6_GetIPv4_BE(void)  { return 0u; }   /* no IP to announce */
uint16_t EthernetN6_GetIPv4_Port(void){ return 0u; }

} /* extern "C" */

bool EdgeViewFramesClass::begin()
{
  return s_started;
}

bool EdgeViewFramesClass::active() const
{
  return s_started && (g_n6_active_transport == TRANSPORT_ETHERNET);
}

bool EdgeViewFramesClass::holding() const
{
  return s_holding;
}

uint32_t EdgeViewFramesClass::framesSeen() const    { return s_seen; }
uint32_t EdgeViewFramesClass::framesSkipped() const { return s_skipped; }

bool EdgeViewFramesClass::grab(const uint8_t **jpeg, uint32_t *len, uint32_t waitMs)
{
  if (jpeg == NULL || len == NULL) {
    return false;
  }
  if (s_holding) {
    return false;                  /* release() the previous frame first */
  }
  if (g_n6_video_eth_queue == NULL) {
    return false;                  /* Vision.begin() has not run yet */
  }

  n6_frame_msg_t msg;
  if (xQueueReceive(g_n6_video_eth_queue, &msg, pdMS_TO_TICKS(waitMs)) != pdTRUE) {
    return false;
  }
  s_seen++;

  /* A meta-only message owns no buffer and is dropped without a completion. */
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

void EdgeViewFramesClass::release()
{
  if (!s_holding) {
    return;
  }
  s_holding = false;
  s_busy    = false;
  n6_video_tx_complete();          /* reopens the encoder */
}
