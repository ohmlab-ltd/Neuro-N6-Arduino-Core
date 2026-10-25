/**
 ******************************************************************************
 * @file    n6_face_bank.h
 * @brief   In-RAM set of enrolled face embeddings, and the match test.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6_FACE_BANK_H
#define N6_FACE_BANK_H

#include <stdint.h>
#include <stdbool.h>

#include "n6_face.h"

#ifdef __cplusplus
extern "C" {
#endif

#define N6_FACE_BANK_STORAGE(name, slots) \
  static float name[(slots) * N6_FACE_EMB_DIM]

typedef struct {
  float   *store;      /* slots * N6_FACE_EMB_DIM, caller-owned */
  uint16_t slots;
  uint16_t used;
} n6_face_bank_t;

void n6_face_bank_init(n6_face_bank_t *b, float *storage, uint16_t slots);
void n6_face_bank_clear(n6_face_bank_t *b);

/* Stores an L2-normalised embedding. Returns its slot, or -1 when the bank is
 * full. */
int  n6_face_bank_add(n6_face_bank_t *b, const float *emb);

/* Best match at or above threshold, a cosine similarity in [-1,1] with 0.45
 * as a starting value. Returns the slot, or -1 for no match. out_sim receives
 * the best similarity found either way, 0 on an empty bank. */
int  n6_face_bank_match(const n6_face_bank_t *b, const float *emb,
                        float threshold, float *out_sim);

static inline uint16_t n6_face_bank_used(const n6_face_bank_t *b)
{
  return (b != NULL) ? b->used : 0u;
}
static inline bool n6_face_bank_full(const n6_face_bank_t *b)
{
  return (b == NULL) || (b->used >= b->slots);
}

#ifdef __cplusplus
}
#endif

#endif /* N6_FACE_BANK_H */
