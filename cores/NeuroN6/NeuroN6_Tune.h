/**
 ******************************************************************************
 * @file    NeuroN6_Tune.h
 * @brief   Live parameter tuning: expose sketch variables to the host UI.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef NEURON6_TUNE_H
#define NEURON6_TUNE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Table size and longest name. Further registrations are dropped and longer
 * names truncated. */
#define N6_TUNE_MAX       16
#define N6_TUNE_NAME_MAX  15

/* Parameter types on the wire. */
enum {
  N6_TUNE_TYPE_INT   = 0,
  N6_TUNE_TYPE_FLOAT = 1,   /* milli-units on the wire */
  N6_TUNE_TYPE_BOOL  = 2
};

#define N6_TUNE_FLOAT_SCALE  1000

/* What the number means. A host renders an unknown hint as a plain slider. */
enum {
  N6_TUNE_HINT_NONE = 0,
  N6_TUNE_HINT_HUE  = 1    /* 0..359 degrees */
};

/* Each call adds an entry. slot must outlive the program. The USB task writes
 * it, clamped to lo..hi. */
void n6_tune_int(const char *name, int32_t *slot, int32_t lo, int32_t hi);
void n6_tune_float(const char *name, float *slot, float lo, float hi);

/* The slot holds 0 or 1. */
void n6_tune_bool(const char *name, uint8_t *slot);

/* A hue in degrees, 0..359. */
void n6_tune_hue(const char *name, int32_t *slot);

/* Called by the core, not by a sketch. */

uint32_t n6_tune_count(void);

/* Writes the value of a META_TUNE_DESC TLV to buf:
 *   u8  count
 *   per parameter:
 *     u8  id            index
 *     u8  type          N6_TUNE_TYPE_*
 *     u8  hint          N6_TUNE_HINT_*
 *     u8  name_len
 *     u8  name[name_len]
 *     i32 lo, i32 hi, i32 current      little-endian, floats in milli-units
 * Returns the bytes written, or 0 when nothing is registered. Entries that do
 * not fit are left out. */
uint32_t n6_tune_build_descriptor(uint8_t *buf, uint32_t cap);

/* Largest descriptor. */
#define N6_TUNE_DESC_MAX_BYTES \
  (1u + (uint32_t)N6_TUNE_MAX * (4u + (uint32_t)N6_TUNE_NAME_MAX + 12u))

/* Applies a 'TUN0' write. wire_value is in milli-units for a float and is
 * clamped to the bounds. Returns 0, or -1 for an unknown id. */
int n6_tune_apply(uint8_t id, int32_t wire_value);

#ifdef __cplusplus
}
#endif

#endif /* NEURON6_TUNE_H */
