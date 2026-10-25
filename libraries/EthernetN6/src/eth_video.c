/**
 ******************************************************************************
 * @file    eth_video.c
 * @brief   Sends JPEG frames to one TCP client over wired Ethernet.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "NeuroN6_FrameQueue.h"
#include "eth_if.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "lwip/sockets.h"
#include "lwip/inet.h"

extern void     NeuroN6_DbgPrint(const char *msg);
extern void     NeuroN6_Notice(const char *msg);     /* kept and replayed to each host */
extern void     n6_video_tx_complete(void);       /* releases the frame's buffer */
extern uint16_t EthernetN6_GetIPv4_Port(void);

#define ETH_DELAY_MS(ms)   vTaskDelay(pdMS_TO_TICKS(ms))

/* 20-byte frame header, identical to the core's vnd_frame_hdr_t. The metadata
 * TLVs follow it, then the JPEG. */
typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t frame_id;
  uint32_t meta_len;
  uint32_t payload_len;
  uint32_t flags;
} eth_frame_hdr_t;

#define ETH_VND_MAGIC          0x30564E44u   /* "DNV0" LE, matches VND_MAGIC */
#define ETH_TCP_BACKLOG        1             /* one client */
#define HDR_META_COALESCE_CAP  1024U         /* cap for header and metadata in one send */

static TaskHandle_t   s_eth_video_handle = NULL;
static uint16_t       s_tcp_port         = 5000;
static volatile bool  s_tx_active        = false;
static uint32_t       s_frame_id         = 0U;

/* While true, the core producer does not overwrite the frame being sent. */
bool eth_video_tx_busy(void)
{
  return s_tx_active;
}

static int open_listen_socket(void)
{
  int s = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s < 0) return -1;

  int yes = 1;
  (void)lwip_setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  struct sockaddr_in addr;
  (void)memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port        = htons(s_tcp_port);

  if (lwip_bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    lwip_close(s);
    return -1;
  }
  if (lwip_listen(s, ETH_TCP_BACKLOG) < 0) {
    lwip_close(s);
    return -1;
  }
  return s;
}

/* Non-blocking send pump */
typedef enum {
  SP_IDLE,
  SP_HEADER_META,
  SP_HEADER,
  SP_META,
  SP_JPEG,
} send_phase_t;

static int pump_phase(int fd, const uint8_t *src, uint32_t total, uint32_t *offset)
{
  int sent = lwip_send(fd, src + *offset, total - *offset, 0);
  if (sent > 0) {
    *offset += (uint32_t)sent;
    return (*offset >= total) ? 1 : 0;
  }
  if (sent < 0 && (errno == EWOULDBLOCK || errno == EAGAIN)) {
    return 0;  /* TCP buffer full */
  }
  return -1;
}

/* Returns on disconnect or send error. */
static void run_send_loop(int client_fd)
{
  int yes = 1;
  (void)lwip_setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
  int nonblock = 1;
  (void)lwip_ioctl(client_fd, FIONBIO, &nonblock);

  uint8_t         hdr_meta[HDR_META_COALESCE_CAP];
  uint32_t        hdr_meta_total = 0U;
  eth_frame_hdr_t hdr;
  n6_frame_msg_t  msg = {0};

  send_phase_t phase  = SP_IDLE;
  uint32_t     offset = 0U;

  for (;;) {
    if (phase == SP_IDLE) {
      if (xQueueReceive(g_n6_video_eth_queue, &msg, portMAX_DELAY) != pdTRUE) {
        continue;
      }
      if (msg.jpeg_ptr == NULL || msg.jpeg_len == 0U) {
        continue;
      }

      s_tx_active = true;

      hdr.magic       = ETH_VND_MAGIC;
      hdr.frame_id    = s_frame_id;
      hdr.meta_len    = msg.meta_len;
      hdr.payload_len = msg.jpeg_len;
      hdr.flags       = 1U;

      hdr_meta_total = (uint32_t)sizeof(hdr) + msg.meta_len;
      const bool coalesce =
          (msg.meta_len == 0U) ||
          (msg.meta_ptr != NULL && hdr_meta_total <= HDR_META_COALESCE_CAP);

      if (coalesce) {
        memcpy(hdr_meta, &hdr, sizeof(hdr));
        if (msg.meta_len > 0U) {
          memcpy(hdr_meta + sizeof(hdr), msg.meta_ptr, msg.meta_len);
        }
        phase  = SP_HEADER_META;
        offset = 0U;
      } else {
        phase  = SP_HEADER;
        offset = 0U;
      }
    }

    int r = 0;
    switch (phase) {
      case SP_HEADER_META:
        r = pump_phase(client_fd, hdr_meta, hdr_meta_total, &offset);
        if (r == 1) { phase = SP_JPEG; offset = 0U; }
        break;

      case SP_HEADER:
        r = pump_phase(client_fd, (const uint8_t *)&hdr, (uint32_t)sizeof(hdr), &offset);
        if (r == 1) {
          if (msg.meta_len > 0U && msg.meta_ptr != NULL) { phase = SP_META; offset = 0U; }
          else                                           { phase = SP_JPEG; offset = 0U; }
        }
        break;

      case SP_META:
        r = pump_phase(client_fd, msg.meta_ptr, msg.meta_len, &offset);
        if (r == 1) { phase = SP_JPEG; offset = 0U; }
        break;

      case SP_JPEG:
        r = pump_phase(client_fd, msg.jpeg_ptr, msg.jpeg_len, &offset);
        if (r == 1) {
          s_tx_active = false;
          n6_video_tx_complete();
          s_frame_id++;
          phase = SP_IDLE;
        }
        break;

      default:
        phase = SP_IDLE;
        break;
    }

    if (r < 0) {
      /* The client is gone. The in-flight frame's buffer is still released. */
      s_tx_active = false;
      if (phase != SP_IDLE) { n6_video_tx_complete(); }
      return;
    }
    if (r == 0 && phase != SP_IDLE) {
      ETH_DELAY_MS(1U);   /* lets tcpip_thread and eth_rx run */
    }
  }
}

static void eth_video_task(void *arg)
{
  (void)arg;

  /* Waits for the link as long as it takes, saying so once after 60 s. */
  uint32_t waited_ms = 0U;
  while (!eth_link_ready()) {
    ETH_DELAY_MS(100U);
    waited_ms += 100U;
    if (waited_ms == 60000U) {
      NeuroN6_Notice("[EthernetN6] no Ethernet link after 60 s. The video starts "
                     "when a cable is connected.\r\n");
    }
  }

  ETH_DELAY_MS(200U);   /* settle */

  {
    char dbg[96];
    (void)snprintf(dbg, sizeof(dbg),
                   "[EthernetN6] link up, TCP video server listening on :%u\r\n",
                   (unsigned)s_tcp_port);
    NeuroN6_DbgPrint(dbg);
  }

  for (;;) {
    int listen_fd = open_listen_socket();
    if (listen_fd < 0) { ETH_DELAY_MS(2000U); continue; }

    struct sockaddr_in client_addr;
    (void)memset(&client_addr, 0, sizeof(client_addr));
    socklen_t client_addr_len = (socklen_t)sizeof(client_addr);
    int client_fd = lwip_accept(listen_fd, (struct sockaddr *)&client_addr, &client_addr_len);
    if (client_fd < 0) {
      lwip_close(listen_fd);
      ETH_DELAY_MS(1000U);
      continue;
    }

    NeuroN6_DbgPrint("[EthernetN6] Neuro Studio client connected.\r\n");

    /* Stale frames are dropped, each one released. */
    {
      n6_frame_msg_t drain;
      while (xQueueReceive(g_n6_video_eth_queue, &drain, 0U) == pdTRUE) {
        n6_video_tx_complete();
      }
    }

    run_send_loop(client_fd);

    s_tx_active = false;
    lwip_close(client_fd);
    lwip_close(listen_fd);
    NeuroN6_DbgPrint("[EthernetN6] client disconnected, re-listening.\r\n");
    ETH_DELAY_MS(100U);
  }
}

/* Overrides the weak eth_video_start() in ethernetif.c. Called by
 * eth_netif_start() once the netif is up. */
void eth_video_start(void)
{
  if (s_eth_video_handle != NULL) {
    return;
  }

  s_tcp_port = EthernetN6_GetIPv4_Port();
  if (s_tcp_port == 0U) s_tcp_port = 5000U;

  /* 1536 words is 6 kB of stack. */
  BaseType_t ok = xTaskCreate(eth_video_task,
                              "eth_vid",
                              1536U,
                              NULL,
                              tskIDLE_PRIORITY + 3,
                              &s_eth_video_handle);
  if (ok != pdPASS) {
    s_eth_video_handle = NULL;
    NeuroN6_Notice("[EthernetN6] video consumer task create FAILED.\r\n");
  }
}
