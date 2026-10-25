/**
 ******************************************************************************
 * @file    NeuroN6_Tune.c
 * @brief   Live parameter tuning registry.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "NeuroN6_Tune.h"
#include <string.h>

typedef struct {
  char     name[N6_TUNE_NAME_MAX + 1];
  void    *slot;
  uint8_t  type;
  uint8_t  hint;    /* N6_TUNE_HINT_* */
  int32_t  lo;      /* wire units: milli-units for a float */
  int32_t  hi;
} n6_tune_entry_t;

static n6_tune_entry_t s_entries[N6_TUNE_MAX];
static uint32_t        s_count = 0u;

static void tune_add(const char *name, void *slot, uint8_t type, uint8_t hint,
                     int32_t lo, int32_t hi)
{
  if (s_count >= (uint32_t)N6_TUNE_MAX) return;
  if (name == NULL || slot == NULL) return;

  n6_tune_entry_t *e = &s_entries[s_count];

  uint32_t n = 0u;
  while ((name[n] != '\0') && (n < (uint32_t)N6_TUNE_NAME_MAX)) {
    e->name[n] = name[n];
    n++;
  }
  e->name[n] = '\0';

  e->slot = slot;
  e->type = type;
  e->hint = hint;
  if (lo <= hi) { e->lo = lo; e->hi = hi; }
  else          { e->lo = hi; e->hi = lo; }

  s_count++;
}

void n6_tune_int(const char *name, int32_t *slot, int32_t lo, int32_t hi)
{
  tune_add(name, (void *)slot, (uint8_t)N6_TUNE_TYPE_INT,
           (uint8_t)N6_TUNE_HINT_NONE, lo, hi);
}

void n6_tune_float(const char *name, float *slot, float lo, float hi)
{
  tune_add(name, (void *)slot, (uint8_t)N6_TUNE_TYPE_FLOAT,
           (uint8_t)N6_TUNE_HINT_NONE,
           (int32_t)(lo * (float)N6_TUNE_FLOAT_SCALE),
           (int32_t)(hi * (float)N6_TUNE_FLOAT_SCALE));
}

void n6_tune_bool(const char *name, uint8_t *slot)
{
  tune_add(name, (void *)slot, (uint8_t)N6_TUNE_TYPE_BOOL,
           (uint8_t)N6_TUNE_HINT_NONE, 0, 1);
}

void n6_tune_hue(const char *name, int32_t *slot)
{
  tune_add(name, (void *)slot, (uint8_t)N6_TUNE_TYPE_INT,
           (uint8_t)N6_TUNE_HINT_HUE, 0, 359);
}

uint32_t n6_tune_count(void) { return s_count; }

/* Current value in wire units. */
static int32_t tune_read(const n6_tune_entry_t *e)
{
  switch (e->type) {
    case N6_TUNE_TYPE_INT:
      return *(const int32_t *)e->slot;
    case N6_TUNE_TYPE_FLOAT:
      return (int32_t)((*(const float *)e->slot) * (float)N6_TUNE_FLOAT_SCALE);
    case N6_TUNE_TYPE_BOOL:
      return (*(const uint8_t *)e->slot) ? 1 : 0;
    default:
      return 0;
  }
}

static void put_i32_le(uint8_t *p, int32_t v)
{
  const uint32_t u = (uint32_t)v;
  p[0] = (uint8_t)(u & 0xFFu);
  p[1] = (uint8_t)((u >> 8) & 0xFFu);
  p[2] = (uint8_t)((u >> 16) & 0xFFu);
  p[3] = (uint8_t)((u >> 24) & 0xFFu);
}

uint32_t n6_tune_build_descriptor(uint8_t *buf, uint32_t cap)
{
  if (buf == NULL || s_count == 0u || cap < 1u) return 0u;

  uint32_t o = 0u;
  buf[o++] = (uint8_t)s_count;

  for (uint32_t i = 0u; i < s_count; i++) {
    const n6_tune_entry_t *e = &s_entries[i];
    const uint32_t name_len = (uint32_t)strlen(e->name);
    const uint32_t need = 4u + name_len + 12u;

    /* No partial record: the count byte becomes what fits. */
    if ((o + need) > cap) {
      buf[0] = (uint8_t)i;
      break;
    }

    buf[o++] = (uint8_t)i;          /* id */
    buf[o++] = e->type;
    buf[o++] = e->hint;
    buf[o++] = (uint8_t)name_len;
    memcpy(&buf[o], e->name, name_len);
    o += name_len;

    put_i32_le(&buf[o], e->lo);          o += 4u;
    put_i32_le(&buf[o], e->hi);          o += 4u;
    put_i32_le(&buf[o], tune_read(e));   o += 4u;
  }

  return o;
}

int n6_tune_apply(uint8_t id, int32_t wire_value)
{
  if ((uint32_t)id >= s_count) return -1;

  n6_tune_entry_t *e = &s_entries[id];

  int32_t v = wire_value;
  if (v < e->lo) v = e->lo;
  if (v > e->hi) v = e->hi;

  switch (e->type) {
    case N6_TUNE_TYPE_INT:
      *(int32_t *)e->slot = v;
      break;
    case N6_TUNE_TYPE_FLOAT:
      *(float *)e->slot = (float)v / (float)N6_TUNE_FLOAT_SCALE;
      break;
    case N6_TUNE_TYPE_BOOL:
      *(uint8_t *)e->slot = (v != 0) ? 1u : 0u;
      break;
    default:
      return -1;
  }
  return 0;
}
