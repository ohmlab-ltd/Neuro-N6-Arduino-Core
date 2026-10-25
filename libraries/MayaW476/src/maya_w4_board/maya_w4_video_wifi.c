/**
 ******************************************************************************
 * @file    maya_w4_video_wifi.c
 * @brief   WiFi host link: a TCP server and a UDP discovery responder, on the
 *          board's own access point only.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "FreeRTOS.h"
#include "task.h"

#include "NeuroN6_FrameQueue.h"
#include "n6_host_contract.h"
#include "maya_w4_creds.h"
#include "maya_w4_sha256.h"
#include "stm32n6xx_hal.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "lwip/sockets.h"
#include "lwip/inet.h"

extern void NeuroN6_DbgPrint(const char *msg);

#define MAYA_DELAY_MS(ms)   vTaskDelay(pdMS_TO_TICKS(ms))

/* 2, not 1: a half-open connection holds a place until lwIP's 20 s SYN-RCVD
 * timeout. */
#define WIFI_TCP_BACKLOG    2
#define WIFI_POLL_MS        100    /* select timeout */

/* TCP keepalive */
#define WIFI_KEEPIDLE_S     5
#define WIFI_KEEPINTVL_S    1
#define WIFI_KEEPCNT        5

static TaskHandle_t  s_task          = NULL;
static uint16_t      s_tcp_port      = 5000;
static int           s_client        = -1;      /* the attached client, or -1 */
static volatile bool s_client_closed = false;   /* the core has let go of it */

/* Clients proving the link key, one per address, each with its own challenge.
 * A newcomer takes its address's place, else a free one, else the oldest. The
 * first to prove the key is attached and the rest are refused. */
#define LINK_PENDING_MAX    3u

typedef struct {
  int        fd;                                       /* -1: free */
  uint32_t   addr;                                     /* network order */
  TickType_t since;
  uint8_t    challenge[N6_LINK_CHALLENGE_BYTES];
  uint8_t    answer[sizeof(n6_frame_hdr_t) + 4u + 32u];
  uint32_t   got;
} link_pending_t;

static link_pending_t s_pending[LINK_PENDING_MAX];
static uint32_t       s_challenges    = 0u;

/* The client. The core's USB task calls these. */

/* Returns the bytes taken, 0 for a full buffer or an error. client_recv
 * reports a connection that has gone. */
static int32_t client_send(const uint8_t *data, uint32_t len)
{
  const int n = lwip_send(s_client, data, len, MSG_DONTWAIT);
  return (n > 0) ? (int32_t)n : 0;
}

/* Returns the bytes read, 0 for none, -1 when the connection is gone. */
static int32_t client_recv(uint8_t *data, uint32_t len)
{
  if (len == 0u) return 0;

  fd_set rd;
  FD_ZERO(&rd);
  FD_SET(s_client, &rd);
  struct timeval now = { 0, 0 };
  if (lwip_select(s_client + 1, &rd, NULL, NULL, &now) <= 0) return 0;

  const int n = lwip_recv(s_client, data, len, MSG_DONTWAIT);
  return (n > 0) ? (int32_t)n : -1;
}

/* The core has let go of the client. This task closes it. */
static void client_closed(void)
{
  s_client_closed = true;
}

static const n6_net_link_t s_link = { client_send, client_recv, client_closed };

/* Sockets */

/* Zero linger: a connection with data queued is reset and freed at once.
 * Needs LWIP_SO_LINGER. */
static void close_client(int fd)
{
  const struct linger now = { 1, 0 };
  (void)lwip_setsockopt(fd, SOL_SOCKET, SO_LINGER, &now, sizeof(now));
  lwip_close(fd);
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
  if (lwip_listen(s, WIFI_TCP_BACKLOG) < 0) {
    lwip_close(s);
    return -1;
  }
  return s;
}

/* Discovery probes arrive over UDP on the stream's port number. */
static int open_probe_socket(void)
{
  int s = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) return -1;

  struct sockaddr_in addr;
  (void)memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port        = htons(s_tcp_port);

  if (lwip_bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    lwip_close(s);
    return -1;
  }
  return s;
}

/* Non-blocking: a blocked lwip_send stalls wlcmgr_task. */
static void client_setup(int fd)
{
  int on = 1;
  (void)lwip_setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
  (void)lwip_setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
  int v = WIFI_KEEPIDLE_S;
  (void)lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &v, sizeof(v));
  v = WIFI_KEEPINTVL_S;
  (void)lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &v, sizeof(v));
  v = WIFI_KEEPCNT;
  (void)lwip_setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &v, sizeof(v));
  (void)lwip_ioctl(fd, FIONBIO, &on);
}

/* mDNS responder */
int  maya_w4_mdns_open(void);
void maya_w4_mdns_announce(int s);
void maya_w4_mdns_answer(int s);
void maya_w4_mdns_close(int s);

/* Discovery reply: the core's META_BOARD_IPV4 frame, then META_BOARD_NAME and,
 * with a link key set, META_BOARD_LINK. */
static uint32_t probe_reply(uint8_t *out, uint32_t cap)
{
  uint32_t len = NeuroN6_IpAnnounceFrame(out, cap);
  if (len == 0u) return 0u;

  const char *name = maya_w4_get_hostname();
  size_t nl = strlen(name);
  if (nl > 32u) nl = 32u;
  if (len + 3u + nl <= cap) {
    out[len]      = N6_META_BOARD_NAME;
    out[len + 1u] = (uint8_t)nl;
    out[len + 2u] = 0u;
    memcpy(&out[len + 3u], name, nl);
    len += 3u + (uint32_t)nl;
  }
  uint8_t key[64];
  if (maya_w4_get_link_key(key, sizeof(key)) > 0u && len + 4u <= cap) {
    out[len]      = N6_META_BOARD_LINK;
    out[len + 1u] = 1u;
    out[len + 2u] = 0u;
    out[len + 3u] = N6_BOARD_LINK_KEYED;
    len += 4u;
  }
  maya_w4_wipe(key, sizeof(key));

  /* meta_len covers every TLV after the header. */
  const uint32_t meta = len - (uint32_t)sizeof(n6_frame_hdr_t);
  memcpy(&out[8], &meta, sizeof(meta));
  return len;
}

/* Replies to the prober only, on the board's own subnet. A probe must start
 * with a frame header. */
static void answer_probe(int udp)
{
  uint8_t in[64];
  struct sockaddr_in from;
  socklen_t from_len = (socklen_t)sizeof(from);
  const int n = lwip_recvfrom(udp, in, sizeof(in), MSG_DONTWAIT,
                              (struct sockaddr *)&from, &from_len);
  if (n < (int)sizeof(n6_frame_hdr_t)) return;
  if (!maya_w4_wifi_on_link(from.sin_addr.s_addr)) return;

  uint32_t magic;
  memcpy(&magic, in, sizeof(magic));
  if (magic != N6_FRAME_MAGIC) return;

  uint8_t out[96];
  const uint32_t len = probe_reply(out, sizeof(out));
  if (len != 0u) {
    (void)lwip_sendto(udp, out, len, 0, (struct sockaddr *)&from, from_len);
  }
}

/* Link key */

/* The challenge must not repeat, so an overheard answer is no use again. */
static void make_challenge(link_pending_t *p, const struct sockaddr_in *peer)
{
  maya_w4_sha256_t c;
  uint8_t d[32];
  const uint32_t mix[6] = {
    HAL_GetUIDw0(), HAL_GetUIDw1() ^ HAL_GetUIDw2(), ++s_challenges,
    DWT->CYCCNT, (uint32_t)xTaskGetTickCount(),
    peer->sin_addr.s_addr ^ ((uint32_t)peer->sin_port << 16),
  };
  maya_w4_sha256_init(&c);
  maya_w4_sha256_update(&c, mix, sizeof(mix));
  maya_w4_sha256_final(&c, d);
  memcpy(p->challenge, d, sizeof(p->challenge));
}

/* Challenge frame: a header and one TLV. */
static bool send_challenge(const link_pending_t *p)
{
  uint8_t f[sizeof(n6_frame_hdr_t) + 3u + N6_LINK_CHALLENGE_BYTES];
  n6_frame_hdr_t h = { N6_FRAME_MAGIC, 0u, 3u + N6_LINK_CHALLENGE_BYTES, 0u, 0u };
  memcpy(f, &h, sizeof(h));
  f[sizeof(h)]      = N6_META_BOARD_CHALLENGE;
  f[sizeof(h) + 1u] = N6_LINK_CHALLENGE_BYTES;
  f[sizeof(h) + 2u] = 0u;
  memcpy(&f[sizeof(h) + 3u], p->challenge, N6_LINK_CHALLENGE_BYTES);
  return lwip_send(p->fd, f, sizeof(f), MSG_DONTWAIT) == (int)sizeof(f);
}

/* 1: the answer matches. 0: incomplete. -1: no match, or not an answer. */
static int check_answer(const link_pending_t *p)
{
  n6_frame_hdr_t h;
  if (p->got < sizeof(h)) return 0;
  memcpy(&h, p->answer, sizeof(h));
  if (h.magic != N6_FRAME_MAGIC || h.meta_len != 0u || h.payload_len != 4u + 32u) return -1;
  if (p->got < sizeof(p->answer)) return 0;

  uint32_t magic;
  memcpy(&magic, &p->answer[sizeof(h)], sizeof(magic));
  if (magic != N6_CMD_KEY0) return -1;

  uint8_t key[64], want[32];
  const size_t kn = maya_w4_get_link_key(key, sizeof(key));
  maya_w4_hmac_sha256(key, kn, N6_LINK_CONTEXT, sizeof(N6_LINK_CONTEXT) - 1u,
                      p->challenge, sizeof(p->challenge), want);
  uint8_t diff = (kn == 0u) ? 1u : 0u;
  for (uint32_t i = 0u; i < 32u; i++) diff |= (uint8_t)(want[i] ^ p->answer[sizeof(h) + 4u + i]);
  maya_w4_wipe(key, sizeof(key));
  maya_w4_wipe(want, sizeof(want));
  return (diff == 0u) ? 1 : -1;
}

/* Reset, not closed: a peer that stops acknowledging cannot hold a TCP
 * connection in FIN_WAIT. */
static void drop_pending(link_pending_t *p, const char *why)
{
  if (p->fd < 0) return;
  close_client(p->fd);
  p->fd = -1;
  if (why != NULL) {
    char dbg[112];
    (void)snprintf(dbg, sizeof(dbg), "[MayaW476] a Neuro host was refused: %s.\r\n", why);
    NeuroN6_DbgPrint(dbg);
  }
}

static void attach(int fd)
{
  s_client_closed = false;
  s_client = fd;
  if (!NeuroN6_NetAttach(&s_link)) {
    s_client = -1;
    lwip_close(fd);
    NeuroN6_DbgPrint("[MayaW476] the core could not take the host - closed.\r\n");
    return;
  }
  NeuroN6_DbgPrint("[MayaW476] Neuro host connected - stream and controls on WiFi.\r\n");
}

static void drop_all_pending(void)
{
  for (uint32_t i = 0u; i < LINK_PENDING_MAX; i++) drop_pending(&s_pending[i], NULL);
}

/* The place a newcomer from addr takes. */
static link_pending_t *pending_place(uint32_t addr)
{
  link_pending_t *free_place = NULL, *oldest = &s_pending[0];
  for (uint32_t i = 0u; i < LINK_PENDING_MAX; i++) {
    link_pending_t *p = &s_pending[i];
    if (p->fd >= 0 && p->addr == addr) return p;
    if (p->fd < 0) {
      if (free_place == NULL) free_place = p;
    } else if (oldest->fd >= 0 && (TickType_t)(p->since - oldest->since) > (TickType_t)(1UL << 31)) {
      oldest = p;
    }
  }
  return (free_place != NULL) ? free_place : oldest;
}

/* Returns true once the client has proved the key and been attached. */
static bool pending_readable(link_pending_t *p)
{
  const int n = lwip_recv(p->fd, &p->answer[p->got], sizeof(p->answer) - p->got, MSG_DONTWAIT);
  if (n <= 0) { drop_pending(p, "it left without the link key"); return false; }
  p->got += (uint32_t)n;
  const int r = check_answer(p);
  if (r < 0) { drop_pending(p, "wrong link key"); return false; }
  if (r == 0) return false;

  const int fd = p->fd;
  p->fd = -1;
  drop_all_pending();
  NeuroN6_DbgPrint("[MayaW476] link key proved.\r\n");
  attach(fd);
  return true;
}

/* Task */

/* Waits until the core has let go of the client, so that its socket number is
 * not reused under it. The shutdown makes the core's recv report it gone. */
static void release_client(void)
{
  if (s_client < 0) return;
  (void)lwip_shutdown(s_client, SHUT_RDWR);
  for (uint32_t i = 0U; !s_client_closed; ++i) {
    MAYA_DELAY_MS(50U);
    if ((i % 40U) == 39U) (void)lwip_shutdown(s_client, SHUT_RDWR);
  }
  close_client(s_client);
  s_client = -1;
  NeuroN6_DbgPrint("[MayaW476] Neuro host disconnected - link down.\r\n");
}

/* The server runs while the board hosts its own access point. */
static bool ap_link_ready(void)
{
  return maya_w4_wifi_link_ready() && maya_w4_wifi_mode() == MAYA_W4_MODE_AP;
}

/* Serves one link until it goes down or changes. */
static void serve_link(uint32_t generation)
{
  int probe_fd  = open_probe_socket();
  int listen_fd = -1;
  int mdns_fd   = maya_w4_mdns_open();
  for (uint32_t i = 0u; i < LINK_PENDING_MAX; i++) s_pending[i].fd = -1;
  maya_w4_mdns_announce(mdns_fd);
  const TickType_t up_at = xTaskGetTickCount();
  bool announced_twice = false;
  {
    char dbg[160];
    (void)snprintf(dbg, sizeof(dbg),
                   "[MayaW476] WiFi up - Neuro host server on TCP :%u, discovery on "
                   "UDP :%u%s, mDNS %s.local\r\n", (unsigned)s_tcp_port, (unsigned)s_tcp_port,
                   (probe_fd < 0) ? " (FAILED)" : "", maya_w4_get_hostname());
    NeuroN6_DbgPrint(dbg);
  }

  while (ap_link_ready() && maya_w4_wifi_link_generation() == generation) {
    if (s_client >= 0 && s_client_closed) {
      close_client(s_client);
      s_client = -1;
      NeuroN6_DbgPrint("[MayaW476] Neuro host disconnected - listening again.\r\n");
    }
    for (uint32_t i = 0u; i < LINK_PENDING_MAX; i++) {
      if (s_pending[i].fd >= 0 &&
          (xTaskGetTickCount() - s_pending[i].since) > pdMS_TO_TICKS(N6_LINK_ANSWER_MS)) {
        drop_pending(&s_pending[i], "no answer to the link key challenge");
      }
    }
    /* The listener stays open until a host has proved the key. */
    if (s_client < 0 && listen_fd < 0) {
      listen_fd = open_listen_socket();
      if (listen_fd < 0) { MAYA_DELAY_MS(1000U); continue; }
    }

    /* RFC 6762: a second announcement one second after the first. */
    if (!announced_twice && (xTaskGetTickCount() - up_at) > pdMS_TO_TICKS(1000U)) {
      maya_w4_mdns_announce(mdns_fd);
      announced_twice = true;
    }

    fd_set rd;
    FD_ZERO(&rd);
    int maxfd = -1;
    if (probe_fd >= 0)  { FD_SET(probe_fd, &rd);  maxfd = probe_fd; }
    if (listen_fd >= 0) { FD_SET(listen_fd, &rd); if (listen_fd > maxfd) maxfd = listen_fd; }
    if (mdns_fd >= 0)   { FD_SET(mdns_fd, &rd);   if (mdns_fd > maxfd)   maxfd = mdns_fd; }
    for (uint32_t i = 0u; i < LINK_PENDING_MAX; i++) {
      const int fd = s_pending[i].fd;
      if (fd >= 0) { FD_SET(fd, &rd); if (fd > maxfd) maxfd = fd; }
    }
    if (maxfd < 0) { MAYA_DELAY_MS(WIFI_POLL_MS); continue; }

    struct timeval wait = { 0, WIFI_POLL_MS * 1000 };
    if (lwip_select(maxfd + 1, &rd, NULL, NULL, &wait) <= 0) continue;

    if (probe_fd >= 0 && FD_ISSET(probe_fd, &rd)) {
      answer_probe(probe_fd);
    }
    if (mdns_fd >= 0 && FD_ISSET(mdns_fd, &rd)) {
      maya_w4_mdns_answer(mdns_fd);
    }
    for (uint32_t i = 0u; i < LINK_PENDING_MAX; i++) {
      if (s_pending[i].fd >= 0 && FD_ISSET(s_pending[i].fd, &rd) &&
          pending_readable(&s_pending[i]) && listen_fd >= 0) {
        lwip_close(listen_fd);   /* one host at a time */
        listen_fd = -1;
      }
    }

    if (listen_fd >= 0 && FD_ISSET(listen_fd, &rd)) {
      struct sockaddr_in peer;
      socklen_t peer_len = (socklen_t)sizeof(peer);
      const int fd = lwip_accept(listen_fd, (struct sockaddr *)&peer, &peer_len);
      if (fd < 0) continue;

      client_setup(fd);
      uint8_t key[64];
      const size_t keyed = maya_w4_get_link_key(key, sizeof(key));
      maya_w4_wipe(key, sizeof(key));
      if (keyed == 0u) {
        lwip_close(listen_fd);   /* one host at a time */
        listen_fd = -1;
        drop_all_pending();
        attach(fd);
        continue;
      }
      link_pending_t *p = pending_place(peer.sin_addr.s_addr);
      drop_pending(p, NULL);
      p->fd    = fd;
      p->addr  = peer.sin_addr.s_addr;
      p->since = xTaskGetTickCount();
      p->got   = 0u;
      make_challenge(p, &peer);
      if (!send_challenge(p)) drop_pending(p, "the challenge could not be sent");
    }
  }

  drop_all_pending();
  release_client();
  if (listen_fd >= 0) lwip_close(listen_fd);
  if (probe_fd >= 0)  lwip_close(probe_fd);
  maya_w4_mdns_close(mdns_fd);
}

static void maya_w4_video_wifi_task(void *arg)
{
  (void)arg;

  for (;;) {
    bool told = false;
    for (uint32_t i = 0U; !ap_link_ready(); ++i) {
      if (i == 600U && !told) {
        NeuroN6_DbgPrint("[MayaW476] The board's access point is not up - the stream stays "
                         "on USB.\r\n");
        told = true;
      }
      MAYA_DELAY_MS(100U);
    }
    const uint32_t generation = maya_w4_wifi_link_generation();
    MAYA_DELAY_MS(200U);   /* settle */
    serve_link(generation);
    NeuroN6_DbgPrint("[MayaW476] WiFi link down - host server waiting for the next one.\r\n");
  }
}

int maya_w4_video_wifi_start(uint16_t tcp_port)
{
  if (s_task != NULL) {
    return 0;
  }

  s_tcp_port = (tcp_port != 0U) ? tcp_port : maya_w4_get_tcp_port();

  /* Stack: 1536 words, 6 kB. */
  BaseType_t ok = xTaskCreate(maya_w4_video_wifi_task,
                              "maya_w4_vid",
                              1536U,
                              NULL,
                              tskIDLE_PRIORITY + 3,   /* above the connect task */
                              &s_task);
  if (ok != pdPASS) {
    s_task = NULL;
    NeuroN6_DbgPrint("[MayaW476] host server task create FAILED.\r\n");
    return -1;
  }
  return 0;
}
