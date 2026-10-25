/**
 ******************************************************************************
 * @file    vnd_meta_audio.c
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "vnd_meta_audio.h"
#include "vnd_meta.h"
#include "NeuroN6.h"

#include <string.h>
#include <stddef.h>

/* Static: NeuroN6_MetaAnnounce returns before the transfer finishes. The first
 * 3 bytes are for the TLV header. */
static uint8_t s_result_buf[3u + 6u + (64u * 2u)];
static uint8_t s_labels_buf[3u + 1u + (64u * 33u)];

__attribute__((aligned(32), section(".ext_psram")))
static uint8_t s_mel_buf[3u + 4u + (128u * 128u)];

static inline void put_u16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)(v >> 8);
}

static uint16_t to_milli(float v)
{
  if (!(v > 0.0f)) return 0u;          /* also catches NaN */
  if (v >= 1.0f)   return 1000u;
  return (uint16_t)(v * 1000.0f + 0.5f);
}

bool vnd_meta_audio_result(const float *scores, uint8_t count,
                           int top, uint16_t level, float threshold)
{
  if (scores == NULL || count == 0u || count > 64u) return false;

  uint8_t *v = &s_result_buf[3];
  uint32_t n = 0;
  v[n++] = count;
  v[n++] = (top >= 0 && top < (int)count) ? (uint8_t)top : 0xFFu;
  put_u16(&v[n], level);                 n += 2;
  put_u16(&v[n], to_milli(threshold));   n += 2;

  for (uint8_t i = 0; i < count; i++) {
    put_u16(&v[n], to_milli(scores[i]));
    n += 2;
  }

  /* The slot rides video frames. The announce covers a board with no video. */
  (void)vnd_meta_publish_tlv(VND_META_SLOT_AUDIO_CLASS, META_AUDIO_CLASS, v, n);
  return NeuroN6_MetaAnnounce(s_result_buf, META_AUDIO_CLASS, (uint16_t)n);
}

bool vnd_meta_audio_labels(const char *(*label_fn)(int), uint8_t count)
{
  if (label_fn == NULL || count == 0u || count > 64u) return false;

  uint8_t *v = &s_labels_buf[3];
  uint32_t n = 0;
  v[n++] = count;

  for (uint8_t i = 0; i < count; i++) {
    const char *lab = label_fn((int)i);
    if (lab == NULL) lab = "?";

    uint32_t len = 0;
    while (lab[len] != 0 && len < 32u) len++;

    if (3u + n + 1u + len > sizeof(s_labels_buf)) return false;
    v[n++] = (uint8_t)len;
    memcpy(&v[n], lab, len);
    n += len;
  }

  (void)vnd_meta_publish_tlv(VND_META_SLOT_AUDIO_LABELS, META_AUDIO_LABELS, v,
                             (uint16_t)n);
  return NeuroN6_MetaAnnounce(s_labels_buf, META_AUDIO_LABELS, (uint16_t)n);
}

bool vnd_meta_audio_mel(const uint8_t *data, uint8_t n_mel, uint8_t n_frames,
                        uint8_t layout)
{
  if (data == NULL || n_mel == 0u || n_frames == 0u) return false;

  const uint32_t body = (uint32_t)n_mel * (uint32_t)n_frames;
  const uint32_t val  = 4u + body;
  if (val > 0xFFFFu) return false;
  if (3u + val > sizeof(s_mel_buf)) return false;

  uint8_t *v = &s_mel_buf[3];
  v[0] = n_mel;
  v[1] = n_frames;
  v[2] = layout;
  v[3] = 0u;
  memcpy(&v[4], data, body);

  (void)vnd_meta_publish_tlv(VND_META_SLOT_AUDIO_MEL, META_AUDIO_MEL, v,
                             (uint16_t)val);
  return NeuroN6_MetaAnnounce(s_mel_buf, META_AUDIO_MEL, (uint16_t)val);
}

void vnd_meta_audio_clear(void)
{
  vnd_meta_invalidate_slot(VND_META_SLOT_AUDIO_CLASS);
  vnd_meta_invalidate_slot(VND_META_SLOT_AUDIO_MEL);
  vnd_meta_invalidate_slot(VND_META_SLOT_AUDIO_LABELS);
}
