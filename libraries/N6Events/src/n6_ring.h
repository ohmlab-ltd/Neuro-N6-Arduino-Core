/**
 ******************************************************************************
 * @file    n6_ring.h
 * @brief   Ring of recent encoded frames, oldest evicted first.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6_RING_H
#define N6_RING_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define N6_RING_MAGIC_REC    0x31525645u   /* 'EVR1' as it reads in memory */
#define N6_RING_MAGIC_SKIP   0x50494B53u   /* 'SKIP' */
#define N6_RING_HDR_BYTES    20u
#define N6_RING_ALIGN        4u

#define N6_RING_F_META_DROPPED   0x0001u   /* metadata exceeded the cap, none stored */

/* Record header, little-endian and 4-byte aligned, followed by jpeg[], meta[]
 * and padding. Magic N6_RING_MAGIC_SKIP marks a skipped wrap region instead. */
typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t seq;
  uint32_t t_ms;
  uint32_t jpeg_len;    /* for a SKIP record: bytes from this header to the end */
  uint16_t meta_len;
  uint16_t flags;
} n6_ring_hdr_t;

/* One record as handed to a reader. The pointers are into the ring. */
typedef struct {
  uint32_t       seq;
  uint32_t       t_ms;
  const uint8_t *jpeg;
  uint32_t       jpeg_len;
  const uint8_t *meta;      /* NULL when meta_len == 0 */
  uint16_t       meta_len;
  uint16_t       flags;
} n6_ring_rec_t;

typedef struct {
  uint32_t pushed;            /* records accepted */
  uint32_t dropped_paused;    /* push() while paused */
  uint32_t dropped_toolarge;  /* records larger than the whole ring */
  uint32_t evicted;           /* oldest records overwritten */
  uint32_t corrupt;           /* header failed its magic, ring reset */
  uint32_t count;             /* records held */
  uint32_t used;              /* bytes in use, skip regions included */
  uint32_t cap;               /* buffer size */
  uint32_t seq;               /* next sequence number */
} n6_ring_stats_t;

typedef struct {
  uint8_t *buf;
  uint32_t cap;
  uint32_t head;              /* next write offset */
  uint32_t tail;              /* oldest record offset */
  uint32_t used;
  uint32_t count;
  uint32_t seq;
  volatile uint8_t paused;
  volatile uint8_t busy;      /* a push is between its first and last write */
  uint32_t pushed, dropped_paused, dropped_toolarge, evicted, corrupt;
} n6_ring_t;

/* bytes must exceed one record header. */
bool n6_ring_init(n6_ring_t *r, uint8_t *buf, uint32_t bytes);

/* Called from one writer task. Evicts the oldest records as needed and returns
 * false when paused or when the record cannot fit an empty ring. */
bool n6_ring_push(n6_ring_t *r,
                  const uint8_t *jpeg, uint32_t jpeg_len,
                  const uint8_t *meta, uint16_t meta_len,
                  uint32_t t_ms, uint16_t flags);

/* Stops pushes so the ring can be read. Returns once no push is in flight. */
void n6_ring_pause(n6_ring_t *r, bool pause);
bool n6_ring_paused(const n6_ring_t *r);

/* Discards every record. seq and the cumulative counters are kept. */
void n6_ring_clear(n6_ring_t *r);

void n6_ring_stats(const n6_ring_t *r, n6_ring_stats_t *out);

/* Oldest to newest. Valid only while paused, or on the writer's own task. */
typedef struct {
  const n6_ring_t *r;
  uint32_t pos;
  uint32_t left;      /* bytes not yet walked */
} n6_ring_iter_t;

void n6_ring_iter_begin(const n6_ring_t *r, n6_ring_iter_t *it);
bool n6_ring_iter_next(n6_ring_iter_t *it, n6_ring_rec_t *out);

/* Counts the records with t0 <= t_ms <= t1. */
uint32_t n6_ring_count_window(const n6_ring_t *r, uint32_t t0, uint32_t t1);

/* True when t0 <= t <= t1 on a wrapping 32-bit millisecond clock. */
static inline bool n6_ring_in_window(uint32_t t, uint32_t t0, uint32_t t1)
{
  return (int32_t)(t - t0) >= 0 && (int32_t)(t1 - t) >= 0;
}

#ifdef __cplusplus
}
#endif

#endif /* N6_RING_H */
