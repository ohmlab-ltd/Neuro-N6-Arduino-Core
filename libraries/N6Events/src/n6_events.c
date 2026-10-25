/**
 ******************************************************************************
 * @file    n6_events.c
 * @brief   Event state machine, clip assembly, sinks and host TLV.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "n6_events.h"
#include <string.h>

#include "NeuroN6.h"
#include "NeuroN6_FrameQueue.h"
#include "n6_host_contract.h"

extern uint32_t millis(void);
extern void    *n6_psram_alloc(uint32_t bytes);

/* PostProcess. The core's weak stubs return false when it is not linked.
 * N6_EVENTS_META_SLOT mirrors VND_META_SLOT_EVENT. */
extern bool vnd_meta_publish_tlv(uint32_t slot, uint8_t type, const void *value, uint16_t len);
extern bool vnd_meta_get_slot(uint32_t slot, const uint8_t **ptr, uint32_t *len);
#define N6_EVENTS_META_SLOT   18u
#define N6_EVENTS_MODEL_SLOT  0u
#define META_BOXES_TAG        2u

static n6_events_cfg_t   s_cfg;
static n6_ring_t         s_ring;
static bool              s_armed = false;
static const char       *s_err   = "not begun";

static n6_events_state_t s_state = N6_EVT_IDLE;
static uint32_t          s_t_state = 0u;        /* when the state was entered */
static n6_event_t        s_ev;                  /* current or last event */
static uint32_t          s_next_id = 1u;

static bool              s_pending = false;
static uint8_t           s_pending_src = 0u;
static uint16_t          s_pending_score = 0u;

static n6_event_sink_t   s_sinks[N6_EVENTS_MAX_SINKS];
static uint8_t           s_nsinks = 0u;

static n6_events_stats_t s_st;

/* Tap decimation */
static uint32_t s_keep_interval_ms = 0u;
static uint32_t s_last_kept = 0u;
static bool     s_kept_any  = false;

/* Announce buffer with 3 bytes of headroom. It must outlive the transfer. */
static uint8_t  s_ann[3u + N6_EVENTS_TLV_BYTES];
static uint32_t s_announce_until = 0u;
static uint32_t s_last_announce  = 0u;
static bool     s_announce_now   = false;

uint32_t n6_events_now(void) { return millis(); }

static bool begin_common(const n6_events_cfg_t *cfg, uint8_t *buf, uint32_t bytes)
{
  if (cfg == NULL) { s_err = "no config"; return false; }
  s_cfg = *cfg;
  if (!n6_ring_init(&s_ring, buf, bytes)) { s_err = "ring too small"; return false; }

  s_keep_interval_ms = (s_cfg.clip_fps != 0u) ? (1000u / s_cfg.clip_fps) : 0u;
  s_last_kept = 0u; s_kept_any = false;

  memset(&s_st, 0, sizeof(s_st));
  memset(&s_ev, 0, sizeof(s_ev));
  s_nsinks = 0u;
  s_state = N6_EVT_IDLE;
  s_t_state = n6_events_now();
  s_next_id = 1u;
  s_pending = false; s_pending_src = 0u; s_pending_score = 0u;
  s_announce_until = 0u; s_last_announce = 0u; s_announce_now = false;

  s_armed = true;
  s_err = "ok";
  return true;
}

bool n6_events_begin(const n6_events_cfg_t *cfg)
{
  if (cfg == NULL) { s_err = "no config"; return false; }
  if (cfg->ring_bytes < (64u * 1024u)) { s_err = "ring_bytes below 64 KB"; return false; }
  uint8_t *buf = (uint8_t *)n6_psram_alloc(cfg->ring_bytes);
  if (buf == NULL) { s_err = "PSRAM pool exhausted (is this after Vision.begin?)"; return false; }
  return begin_common(cfg, buf, cfg->ring_bytes);
}

bool n6_events_begin_buf(const n6_events_cfg_t *cfg, uint8_t *buf, uint32_t bytes)
{
  if (buf == NULL || bytes == 0u) { s_err = "no buffer"; return false; }
  return begin_common(cfg, buf, bytes);
}

bool n6_events_add_sink(const n6_event_sink_t *sink)
{
  if (sink == NULL || sink->open == NULL || sink->frame == NULL || sink->close == NULL) return false;
  if (s_nsinks >= N6_EVENTS_MAX_SINKS) return false;
  s_sinks[s_nsinks++] = *sink;
  return true;
}

const char *n6_events_last_error(void) { return s_err; }

/* Ring ingest */

bool n6_events_push(const uint8_t *jpeg, uint32_t jpeg_len,
                    const uint8_t *meta, uint32_t meta_len, uint32_t t_ms)
{
  if (!s_armed) return false;
  uint16_t flags = 0u;
  if (meta_len > s_cfg.meta_max) {
    meta = NULL; meta_len = 0u;
    flags |= N6_RING_F_META_DROPPED;
    s_st.tap_meta_dropped++;
  }
  return n6_ring_push(&s_ring, jpeg, jpeg_len, meta, (uint16_t)meta_len, t_ms, flags);
}

/* Frame tap, called from the capture task for every encoded frame. */
void n6_frame_tap(const uint8_t *jpeg, uint32_t jpeg_len,
                  const uint8_t *meta, uint32_t meta_len)
{
  if (!s_armed) return;
  s_st.tap_frames++;

  if (n6_ring_paused(&s_ring)) { s_st.tap_dropped_paused++; return; }

  const uint32_t now = n6_events_now();
  if (s_keep_interval_ms != 0u && s_kept_any &&
      (uint32_t)(now - s_last_kept) < s_keep_interval_ms) {
    return;
  }

  if (n6_events_push(jpeg, jpeg_len, meta, meta_len, now)) {
    s_last_kept = now;
    s_kept_any  = true;
    s_st.tap_kept++;
  }
}

/* Host TLV */

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v) { put_u16(p, (uint16_t)v); put_u16(p + 2, (uint16_t)(v >> 16)); }

void n6_events_encode_tlv(uint8_t out[N6_EVENTS_TLV_BYTES])
{
  if (out == NULL) return;
  put_u32(&out[0],  s_ev.id);
  put_u32(&out[4],  s_ev.t_ms);
  out[8]  = s_ev.sources;
  out[9]  = (uint8_t)s_state;
  put_u16(&out[10], s_ev.score);
  put_u16(&out[12], s_ev.pre_frames);
  put_u16(&out[14], (uint16_t)(s_ev.n_frames - s_ev.pre_frames));
  out[16] = s_ev.sinks_ok;
}

/* Publishes the EVENT TLV: tag N6_HOST_TAG_EVENT, 0x67, RESULT class. */
static void publish(void)
{
  uint8_t *v = &s_ann[3];
  n6_events_encode_tlv(v);
  (void)vnd_meta_publish_tlv(N6_EVENTS_META_SLOT, N6_HOST_TAG_EVENT, v, N6_EVENTS_TLV_BYTES);
  (void)NeuroN6_MetaAnnounce(s_ann, N6_HOST_TAG_EVENT, N6_EVENTS_TLV_BYTES);
}

static void enter(n6_events_state_t st)
{
  const uint32_t now = n6_events_now();
  s_state   = st;
  s_t_state = now;
  s_announce_until = now + s_cfg.announce_ms;
  s_announce_now   = true;
}

static void service_announce(void)
{
  const uint32_t now = n6_events_now();
  if (s_ev.id == 0u) return;                                   /* no event yet */
  const bool window_open = (int32_t)(s_announce_until - now) > 0;
  if (s_announce_now || (window_open && (uint32_t)(now - s_last_announce) >= 1000u)) {
    publish();
    s_last_announce = now;
    s_announce_now  = false;
  }
}

/* Clip */

static void write_clip(void)
{
  const uint32_t t_start = n6_events_now();
  const uint32_t t0 = s_ev.t_ms - s_cfg.pre_ms;
  const uint32_t t1 = s_ev.t_ms + s_cfg.post_ms;

  n6_ring_pause(&s_ring, true);

  /* Counted first, so open() sees n_frames. */
  {
    n6_ring_iter_t it; n6_ring_rec_t rec;
    uint32_t n = 0u, pre = 0u;
    n6_ring_iter_begin(&s_ring, &it);
    while (n6_ring_iter_next(&it, &rec)) {
      if (!n6_ring_in_window(rec.t_ms, t0, t1)) continue;
      n++;
      if ((int32_t)(rec.t_ms - s_ev.t_ms) < 0) pre++;
    }
    s_ev.n_frames   = (uint16_t)((n > 65535u) ? 65535u : n);
    s_ev.pre_frames = (uint16_t)((pre > 65535u) ? 65535u : pre);
  }

  s_ev.sinks_ok = 0u;
  for (uint8_t i = 0u; i < s_nsinks; i++) {
    n6_event_sink_t *sk = &s_sinks[i];
    if (!sk->open(sk->ctx, &s_ev)) { s_st.sink_failures++; continue; }

    bool ok = true;
    n6_ring_iter_t it; n6_ring_rec_t rec;
    n6_ring_iter_begin(&s_ring, &it);
    while (n6_ring_iter_next(&it, &rec)) {
      if (!n6_ring_in_window(rec.t_ms, t0, t1)) continue;
      n6_clip_frame_t f;
      f.seq = rec.seq; f.t_ms = rec.t_ms;
      f.jpeg = rec.jpeg; f.jpeg_len = rec.jpeg_len;
      f.meta = rec.meta; f.meta_len = rec.meta_len;
      if (!sk->frame(sk->ctx, &f)) { ok = false; break; }
    }
    if (!sk->close(sk->ctx, ok)) ok = false;

    if (ok) s_ev.sinks_ok |= (uint8_t)(1u << i);
    else    s_st.sink_failures++;
  }

  n6_ring_pause(&s_ring, false);

  s_st.clips++;
  s_st.last_clip_frames = s_ev.n_frames;
  s_st.last_sinks_ok    = s_ev.sinks_ok;
  s_st.last_write_ms    = n6_events_now() - t_start;
}

/* State machine */

static void start_event(uint8_t sources, uint16_t score)
{
  memset(&s_ev, 0, sizeof(s_ev));
  s_ev.id      = s_next_id++;
  s_ev.t_ms    = n6_events_now();
  s_ev.sources = sources;
  s_ev.score   = score;
  s_st.events++;
  enter(N6_EVT_POST_ROLL);
}

bool n6_events_fire(uint8_t sources, uint16_t score)
{
  if (!s_armed) return false;
  switch (s_state) {
    case N6_EVT_IDLE:
      start_event(sources, score);
      return true;

    case N6_EVT_POST_ROLL:
      s_ev.sources |= sources;
      if (score > s_ev.score) s_ev.score = score;
      s_st.fires_folded++;
      s_announce_now = true;
      return true;

    case N6_EVT_WRITING:
    case N6_EVT_REFRACTORY:
    default:
      if (!s_pending) s_st.fires_pending++;
      s_pending = true;
      s_pending_src |= sources;
      if (score > s_pending_score) s_pending_score = score;
      return true;
  }
}

void n6_events_service(void)
{
  if (!s_armed) return;
  const uint32_t now = n6_events_now();

  switch (s_state) {
    case N6_EVT_IDLE:
      if (s_pending) {
        const uint8_t src = s_pending_src; const uint16_t sc = s_pending_score;
        s_pending = false; s_pending_src = 0u; s_pending_score = 0u;
        start_event(src, sc);
      }
      break;

    case N6_EVT_POST_ROLL:
      if ((uint32_t)(now - s_ev.t_ms) >= s_cfg.post_ms) {
        enter(N6_EVT_WRITING);
        service_announce();            /* announces WRITING before blocking */
        write_clip();
        enter(N6_EVT_REFRACTORY);
      }
      break;

    case N6_EVT_WRITING:               /* not reached: write_clip is synchronous */
      enter(N6_EVT_REFRACTORY);
      break;

    case N6_EVT_REFRACTORY:
    default:
      if ((uint32_t)(now - s_t_state) >= s_cfg.refractory_ms) {
        enter(N6_EVT_IDLE);
      }
      break;
  }

  service_announce();
}

n6_events_state_t n6_events_state(void) { return s_state; }
const n6_event_t *n6_events_current(void) { return &s_ev; }

void n6_events_stats(n6_events_stats_t *out)
{
  if (out == NULL) return;
  *out = s_st;
  n6_ring_stats(&s_ring, &out->ring);
}

/* Generic NN trigger */

uint8_t n6_events_count_boxes(const uint8_t *tlv, uint32_t len, uint8_t min_score)
{
  if (tlv == NULL) return 0u;
  uint32_t n = 0u;
  uint32_t i = 0u;
  while (i + 3u <= len) {
    const uint8_t  type = tlv[i];
    const uint32_t l    = (uint32_t)tlv[i + 1] | ((uint32_t)tlv[i + 2] << 8);
    const uint32_t vstart = i + 3u;
    if (vstart + l > len) break;                    /* truncated */
    if (type == META_BOXES_TAG && l >= 1u) {
      const uint32_t count = tlv[vstart];
      for (uint32_t b = 0u; b < count; b++) {
        const uint32_t off = vstart + 1u + b * 10u;
        if (off + 10u > vstart + l) break;          /* count claims more than sent */
        if (tlv[off + 8] >= min_score) n++;
      }
    }
    i = vstart + l;
  }
  return (uint8_t)((n > 255u) ? 255u : n);
}

uint8_t n6_events_last_boxes(uint8_t min_score)
{
  const uint8_t *p = NULL; uint32_t len = 0u;
  if (!vnd_meta_get_slot(N6_EVENTS_MODEL_SLOT, &p, &len) || p == NULL) return 0u;
  return n6_events_count_boxes(p, len, min_score);
}
