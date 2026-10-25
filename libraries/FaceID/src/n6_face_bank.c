/**
 ******************************************************************************
 * @file    n6_face_bank.c
 * @brief   In-RAM set of enrolled face embeddings, and the match test.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "n6_face_bank.h"

#include <string.h>
#include <stddef.h>

void n6_face_bank_init(n6_face_bank_t *b, float *storage, uint16_t slots)
{
  if (b == NULL) return;
  b->store = storage;
  b->slots = (storage != NULL) ? slots : 0u;
  b->used  = 0u;
}

void n6_face_bank_clear(n6_face_bank_t *b)
{
  if (b != NULL) b->used = 0u;
}

int n6_face_bank_add(n6_face_bank_t *b, const float *emb)
{
  if (b == NULL || b->store == NULL || emb == NULL) return -1;
  if (b->used >= b->slots) return -1;

  const int slot = (int)b->used;
  memcpy(&b->store[(size_t)slot * N6_FACE_EMB_DIM], emb,
         sizeof(float) * N6_FACE_EMB_DIM);
  b->used++;
  return slot;
}

int n6_face_bank_match(const n6_face_bank_t *b, const float *emb,
                       float threshold, float *out_sim)
{
  if (out_sim != NULL) *out_sim = 0.0f;
  if (b == NULL || b->store == NULL || emb == NULL || b->used == 0u) return -1;

  int   best      = -1;
  float best_sim  = -2.0f;

  for (uint16_t s = 0u; s < b->used; s++) {
    const float *e = &b->store[(size_t)s * N6_FACE_EMB_DIM];

    /* Both sides are L2-normalised, so the dot product is the cosine. */
    float dot = 0.0f;
    for (int i = 0; i < N6_FACE_EMB_DIM; i++) dot += e[i] * emb[i];

    if (dot > best_sim) { best_sim = dot; best = (int)s; }
  }

  if (out_sim != NULL) *out_sim = best_sim;
  return (best_sim >= threshold) ? best : -1;
}
