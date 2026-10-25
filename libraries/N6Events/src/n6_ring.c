/**
 ******************************************************************************
 * @file    n6_ring.c
 * @brief   Byte ring of variable-size frame records.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "n6_ring.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

/* Keeps the writes to busy and paused in order around the copies they guard. */
#define BARRIER()  __sync_synchronize()

static inline uint32_t align_up(uint32_t v)
{
  return (v + (N6_RING_ALIGN - 1u)) & ~(N6_RING_ALIGN - 1u);
}

static inline uint32_t rec_size(uint32_t jpeg_len, uint32_t meta_len)
{
  return align_up(N6_RING_HDR_BYTES + jpeg_len + meta_len);
}

static void reset(n6_ring_t *r)
{
  r->head = 0u; r->tail = 0u; r->used = 0u; r->count = 0u;
}

bool n6_ring_init(n6_ring_t *r, uint8_t *buf, uint32_t bytes)
{
  if (r == NULL || buf == NULL || bytes <= N6_RING_HDR_BYTES) return false;
  memset(r, 0, sizeof(*r));
  r->buf = buf;
  r->cap = bytes & ~(N6_RING_ALIGN - 1u);   /* keeps every record 4-aligned */
  return true;
}

void n6_ring_clear(n6_ring_t *r)
{
  if (r == NULL) return;
  reset(r);
}

/* Drops the oldest record or skip region. A skip region's size comes from its
 * position, not its header. */
static void evict_oldest(n6_ring_t *r)
{
  const uint32_t rem = r->cap - r->tail;

  if (rem < N6_RING_HDR_BYTES) {          /* implicit wrap region */
    r->used -= (rem <= r->used) ? rem : r->used;
    r->tail = 0u;
    return;
  }

  n6_ring_hdr_t h;
  memcpy(&h, r->buf + r->tail, sizeof(h));

  if (h.magic == N6_RING_MAGIC_SKIP) {    /* explicit wrap region */
    r->used -= (rem <= r->used) ? rem : r->used;
    r->tail = 0u;
    return;
  }

  if (h.magic != N6_RING_MAGIC_REC) {
    r->corrupt++;
    reset(r);
    return;
  }

  const uint32_t sz = rec_size(h.jpeg_len, h.meta_len);
  if (sz > r->used || sz > rem) { r->corrupt++; reset(r); return; }

  r->used -= sz;
  r->tail += sz;
  if (r->tail == r->cap) r->tail = 0u;
  if (r->count) r->count--;
  r->evicted++;
}

bool n6_ring_push(n6_ring_t *r,
                  const uint8_t *jpeg, uint32_t jpeg_len,
                  const uint8_t *meta, uint16_t meta_len,
                  uint32_t t_ms, uint16_t flags)
{
  if (r == NULL || r->buf == NULL) return false;
  if (jpeg == NULL || jpeg_len == 0u) return false;
  if (meta == NULL) meta_len = 0u;

  if (r->paused) { r->dropped_paused++; return false; }

  const uint32_t need = rec_size(jpeg_len, meta_len);
  if (need > r->cap) { r->dropped_toolarge++; return false; }

  r->busy = 1u;
  BARRIER();

  /* A record never wraps: skip is the unused space to the end of the buffer. */
  uint32_t skip;
  for (;;) {
    const bool wrap = (r->cap - r->head) < need;
    skip = wrap ? (r->cap - r->head) : 0u;
    if ((r->cap - r->used) >= need + skip) break;
    if (r->used == 0u) { r->head = 0u; r->tail = 0u; continue; }
    evict_oldest(r);
  }

  if (skip != 0u) {
    if (skip >= N6_RING_HDR_BYTES) {
      n6_ring_hdr_t s;
      memset(&s, 0, sizeof(s));
      s.magic    = N6_RING_MAGIC_SKIP;
      s.jpeg_len = skip;
      memcpy(r->buf + r->head, &s, sizeof(s));
    }
    r->used += skip;
    r->head  = 0u;
  }

  n6_ring_hdr_t h;
  h.magic    = N6_RING_MAGIC_REC;
  h.seq      = r->seq;
  h.t_ms     = t_ms;
  h.jpeg_len = jpeg_len;
  h.meta_len = meta_len;
  h.flags    = flags;

  uint8_t *p = r->buf + r->head;
  memcpy(p, &h, sizeof(h));
  memcpy(p + N6_RING_HDR_BYTES, jpeg, jpeg_len);
  if (meta_len) memcpy(p + N6_RING_HDR_BYTES + jpeg_len, meta, meta_len);

  r->head += need;
  if (r->head == r->cap) r->head = 0u;
  r->used += need;
  r->count++;
  r->seq++;
  r->pushed++;

  BARRIER();
  r->busy = 0u;
  return true;
}

void n6_ring_pause(n6_ring_t *r, bool pause)
{
  if (r == NULL) return;
  r->paused = pause ? 1u : 0u;
  BARRIER();
  if (!pause) return;
  /* Waits out a push that began before the pause. */
  while (r->busy) { vTaskDelay(1); }
}

bool n6_ring_paused(const n6_ring_t *r)
{
  return (r != NULL) && (r->paused != 0u);
}

void n6_ring_stats(const n6_ring_t *r, n6_ring_stats_t *out)
{
  if (out == NULL) return;
  memset(out, 0, sizeof(*out));
  if (r == NULL) return;
  out->pushed           = r->pushed;
  out->dropped_paused   = r->dropped_paused;
  out->dropped_toolarge = r->dropped_toolarge;
  out->evicted          = r->evicted;
  out->corrupt          = r->corrupt;
  out->count            = r->count;
  out->used             = r->used;
  out->cap              = r->cap;
  out->seq              = r->seq;
}

void n6_ring_iter_begin(const n6_ring_t *r, n6_ring_iter_t *it)
{
  if (it == NULL) return;
  it->r    = r;
  it->pos  = (r != NULL) ? r->tail : 0u;
  it->left = (r != NULL) ? r->used : 0u;
}

bool n6_ring_iter_next(n6_ring_iter_t *it, n6_ring_rec_t *out)
{
  if (it == NULL || it->r == NULL || out == NULL) return false;
  const n6_ring_t *r = it->r;

  for (;;) {
    if (it->left == 0u) return false;

    const uint32_t rem = r->cap - it->pos;
    if (rem < N6_RING_HDR_BYTES) {              /* implicit wrap */
      if (rem > it->left) return false;
      it->left -= rem; it->pos = 0u;
      continue;
    }

    n6_ring_hdr_t h;
    memcpy(&h, r->buf + it->pos, sizeof(h));

    if (h.magic == N6_RING_MAGIC_SKIP) {        /* explicit wrap */
      if (rem > it->left) return false;
      it->left -= rem; it->pos = 0u;
      continue;
    }
    if (h.magic != N6_RING_MAGIC_REC) return false;   /* corrupt */

    const uint32_t sz = rec_size(h.jpeg_len, h.meta_len);
    if (sz > it->left || sz > rem) return false;

    const uint8_t *p = r->buf + it->pos;
    out->seq      = h.seq;
    out->t_ms     = h.t_ms;
    out->jpeg     = p + N6_RING_HDR_BYTES;
    out->jpeg_len = h.jpeg_len;
    out->meta     = h.meta_len ? (p + N6_RING_HDR_BYTES + h.jpeg_len) : NULL;
    out->meta_len = h.meta_len;
    out->flags    = h.flags;

    it->left -= sz;
    it->pos  += sz;
    if (it->pos == r->cap) it->pos = 0u;
    return true;
  }
}

uint32_t n6_ring_count_window(const n6_ring_t *r, uint32_t t0, uint32_t t1)
{
  n6_ring_iter_t it;
  n6_ring_rec_t rec;
  uint32_t n = 0u;
  n6_ring_iter_begin(r, &it);
  while (n6_ring_iter_next(&it, &rec)) {
    if (n6_ring_in_window(rec.t_ms, t0, t1)) n++;
  }
  return n;
}
