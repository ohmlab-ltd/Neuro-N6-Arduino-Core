/**
 ******************************************************************************
 * @file    n6_events.h
 * @brief   Events: fire() saves the frames around it as a clip for the sinks.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6_EVENTS_H
#define N6_EVENTS_H

#include <stdint.h>
#include <stdbool.h>
#include "n6_ring.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Event sources */
#define N6_EVT_SRC_MOTION   0x01u
#define N6_EVT_SRC_NN       0x02u
#define N6_EVT_SRC_AUDIO    0x04u
#define N6_EVT_SRC_THERMAL  0x08u
#define N6_EVT_SRC_TOF      0x10u
#define N6_EVT_SRC_GPIO     0x20u
#define N6_EVT_SRC_TIMER    0x40u
#define N6_EVT_SRC_HOST     0x80u

typedef enum {
  N6_EVT_IDLE       = 0,
  N6_EVT_POST_ROLL  = 1,
  N6_EVT_WRITING    = 2,
  N6_EVT_REFRACTORY = 3,
} n6_events_state_t;

typedef struct {
  uint32_t id;          /* from 1, per boot */
  uint32_t t_ms;        /* when it fired, ms since boot */
  uint8_t  sources;     /* N6_EVT_SRC_* bits, ORed across folded fires */
  uint16_t score;       /* sketch-defined, maximum across folded fires */
  uint16_t n_frames;    /* frames in the clip, valid from WRITING */
  uint16_t pre_frames;  /* of those, before t_ms */
  uint8_t  sinks_ok;    /* bit i: sink i completed the clip */
} n6_event_t;

/* One frame of a clip. The pointers are into the ring, valid for the call. */
typedef struct {
  uint32_t       seq;
  uint32_t       t_ms;
  const uint8_t *jpeg;
  uint32_t       jpeg_len;
  const uint8_t *meta;      /* TLV metadata, or NULL */
  uint16_t       meta_len;
} n6_clip_frame_t;

/* Called on the sketch task: open() once per event, frame() per frame oldest
 * first, close() with whether every frame was accepted. False abandons the
 * clip on this sink only. */
typedef struct {
  const char *name;
  bool (*open )(void *ctx, const n6_event_t *ev);
  bool (*frame)(void *ctx, const n6_clip_frame_t *f);
  bool (*close)(void *ctx, bool ok);
  void *ctx;
} n6_event_sink_t;

#define N6_EVENTS_MAX_SINKS  4u

typedef struct {
  uint32_t ring_bytes;      /* PSRAM for the ring */
  uint16_t clip_fps;        /* frames the tap keeps per second, 0 for every one */
  uint16_t pre_ms;          /* clip length before the fire */
  uint16_t post_ms;         /* clip length after it */
  uint16_t refractory_ms;   /* fires are deferred for this long after a clip */
  uint16_t meta_max;        /* per-frame metadata limit, larger is dropped */
  uint16_t announce_ms;     /* how long the EVENT TLV is re-announced */
} n6_events_cfg_t;

/* At 5 fps and 160 KB a frame, 8 MB holds ten seconds. */
#define N6_EVENTS_CFG_DEFAULT  { 8u << 20, 5, 5000, 5000, 10000, 4096, 5000 }

typedef struct {
  uint32_t events;             /* folded fires count once */
  uint32_t clips;              /* clip writes completed, any sink outcome */
  uint32_t sink_failures;      /* sink open, frame or close returning false */
  uint32_t fires_folded;       /* fire() during POST_ROLL */
  uint32_t fires_pending;      /* fire() deferred during WRITING or REFRACTORY */
  uint32_t tap_frames;         /* frames the tap was offered */
  uint32_t tap_kept;           /* of those, pushed */
  uint32_t tap_dropped_paused; /* offered while the ring was paused */
  uint32_t tap_meta_dropped;   /* metadata over meta_max, stored without it */
  uint32_t last_write_ms;
  uint32_t last_clip_frames;
  uint8_t  last_sinks_ok;
  n6_ring_stats_t ring;
} n6_events_stats_t;

/* Allocates the ring from the PSRAM pool, so it comes after Vision.begin(),
 * which resets that pool. On false, n6_events_last_error() gives the reason. */
bool n6_events_begin(const n6_events_cfg_t *cfg);

/* The same on a caller's buffer. bytes replaces cfg->ring_bytes. */
bool n6_events_begin_buf(const n6_events_cfg_t *cfg, uint8_t *buf, uint32_t bytes);

/* Sinks run in the order added and the struct is copied. Either begin() clears
 * the list, so sinks are added after it. */
bool n6_events_add_sink(const n6_event_sink_t *sink);

/* Puts one frame into the ring. The core's frame tap calls it while streaming;
 * a sketch that captures its own JPEGs calls it itself. */
bool n6_events_push(const uint8_t *jpeg, uint32_t jpeg_len,
                    const uint8_t *meta, uint32_t meta_len, uint32_t t_ms);

/* Sketch task only. A fire during POST_ROLL folds into the current event, and
 * one during WRITING or REFRACTORY is held as one pending fire until IDLE. */
bool n6_events_fire(uint8_t sources, uint16_t score);

/* Called from every loop(). Blocks while the sinks write a clip. */
void n6_events_service(void);

n6_events_state_t  n6_events_state(void);
const n6_event_t  *n6_events_current(void);      /* the current or last event */
void               n6_events_stats(n6_events_stats_t *out);
const char        *n6_events_last_error(void);

/* The clock for every t_ms here: millis(). */
uint32_t n6_events_now(void);

/* Counts the boxes in the primary model's META_BOXES that score at least
 * min_score. Returns 0 when PostProcess is not linked or has published
 * nothing. */
uint8_t n6_events_last_boxes(uint8_t min_score);

/* The same count on any TLV buffer. */
uint8_t n6_events_count_boxes(const uint8_t *tlv, uint32_t len, uint8_t min_score);

/* Encodes the current event and state as the 17-byte META_EVENT value. */
#define N6_EVENTS_TLV_BYTES  17u
void n6_events_encode_tlv(uint8_t out[N6_EVENTS_TLV_BYTES]);

#ifdef __cplusplus
}
#endif

#endif /* N6_EVENTS_H */
