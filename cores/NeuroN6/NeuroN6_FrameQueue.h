/**
 ******************************************************************************
 * @file    NeuroN6_FrameQueue.h
 * @brief   Per-transport JPEG frame queues.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef NEURON6_FRAME_QUEUE_H
#define NEURON6_FRAME_QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"

/* Video transport, from cfg.transport. A WiFi mode without <MayaW476.h> falls
 * back to USB. */
typedef enum {
  TRANSPORT_USB         = 0,   /* USB vendor endpoint, the default */
  TRANSPORT_WIFI_SOFTAP = 1,   /* the board hosts an access point */
  TRANSPORT_WIFI_STA    = 2,   /* refused: WiFi video needs the board's own AP */
  TRANSPORT_LCD         = 3,   /* the TFT carrier's panel, no host video */
  TRANSPORT_ETHERNET    = 4,   /* TCP over Ethernet, needs <EthernetN6.h> */
} n6_transport_t;

/* One queued frame. The JPEG buffer stays valid until the consumer calls
 * n6_video_tx_complete(). */
typedef struct {
  const uint8_t *jpeg_ptr;
  uint32_t       jpeg_len;
  const uint8_t *meta_ptr;
  uint32_t       meta_len;
} n6_frame_msg_t;

/* Weak hook, called from the capture task for every encoded frame. It must not
 * block. The pointers are valid only during the call, and meta may be NULL. */
void n6_frame_tap(const uint8_t *jpeg, uint32_t jpeg_len,
                  const uint8_t *meta, uint32_t meta_len);

/* The USB queue also carries WiFi frames. The producer drops on overflow. */
extern QueueHandle_t g_n6_video_usb_queue;
extern QueueHandle_t g_n6_video_eth_queue;

/* Set by Vision.begin() from cfg.transport. */
extern volatile n6_transport_t g_n6_active_transport;

/* True while the consumer is mid-send. */
bool n6_usb_tx_busy(void);
bool n6_eth_tx_busy(void);

/* The active transport's busy flag. */
bool n6_video_tx_busy(void);

/* A network client driven by n6_usb_task, the only caller. None of the three
 * may block. While a client is attached the stream goes to it, not to USB. */
typedef struct {
  /* Bytes taken, 0 when it can take none, < 0 when the connection is gone. */
  int32_t (*send)(const uint8_t *data, uint32_t len);
  /* Bytes read, 0 when none wait, < 0 when the connection is gone. */
  int32_t (*recv)(uint8_t *data, uint32_t len);
  /* The core has released the client and will not call the other two again. */
  void    (*closed)(void);
} n6_net_link_t;

/* Hands a connected client to n6_usb_task. Returns false when one is already
 * attached or the task is not running. */
bool NeuroN6_NetAttach(const n6_net_link_t *link);

/* Writes the META_BOARD_IPV4 announce frame to out. Returns its length, or 0
 * when cap is too small or the board has no address. */
uint32_t NeuroN6_IpAnnounceFrame(uint8_t *out, uint32_t cap);

/* Releases the frame's buffer. A consumer calls it exactly once per frame it
 * finishes, abandons or drains. */
void n6_video_tx_complete(void);

/* Creates the queues and the n6_capture and n6_usb tasks. Idempotent. */
void NeuroN6_VisionTasks_Start(void);

/* Starts only the n6_usb task, for meta-only messages with jpeg_ptr NULL.
 * Idempotent. NeuroN6_VisionTasks_Start() adds the capture task. */
void NeuroN6_StreamUsbOnlyStart(void);

/* MayaW476 hooks, weak. Start returns 0 on success, a negative code when
 * WiFi did not start, and MAYAW476_NOT_LINKED from the default. */
#define MAYAW476_NOT_LINKED  (-99)
int  MayaW476_Start(int transport_mode);

/* Address and TCP port for the META_BOARD_IPV4 announce, 0 for no address. A
 * memcpy of the address lands dotted-quad MSB-first on the wire. */
uint32_t MayaW476_GetIPv4_BE(void);
uint16_t MayaW476_GetIPv4_Port(void);

/* Writes the META_BOARD_SOFTAP value for the access point the board hosts and
 * returns its length, 0 when it hosts none. */
uint32_t MayaW476_SoftApAnnounce(uint8_t *out, uint32_t cap);

/* EthernetN6 hooks, weak, with the same contract: ETHERNETN6_NOT_LINKED from
 * the default. */
#define ETHERNETN6_NOT_LINKED  (-99)
int      EthernetN6_Start(int transport_mode);
bool     EthernetN6_TxBusy(void);
uint32_t EthernetN6_GetIPv4_BE(void);
uint16_t EthernetN6_GetIPv4_Port(void);

/* Microphone hook, weak. Called by n6_usb_task when the vendor link is idle.
 * Returns true with a META_AUDIO_PCM TLV, valid until the next call. */
bool n6_audio_drain(const uint8_t **out_tlv, uint32_t *out_len);

/* Weak. Dumps microphone state to the CDC console once a second, with the
 * debug console built in. */
void n6_audio_debug_dump(void);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_FRAME_QUEUE_H */
