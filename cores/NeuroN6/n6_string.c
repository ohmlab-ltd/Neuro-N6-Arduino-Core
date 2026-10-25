/**
 ******************************************************************************
 * @file    n6_string.c
 * @brief   Wide memcpy, memset and memmove in place of newlib-nano's.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

/* Required: the compiler otherwise rewrites these loops as calls to the
   function being defined. */
#pragma GCC optimize ("no-tree-loop-distribute-patterns")

#include <stddef.h>
#include <stdint.h>

/* may_alias: the wide accesses read and write objects declared as bytes. */
typedef uint32_t __attribute__((may_alias)) n6_word_t;
typedef uint64_t __attribute__((may_alias)) n6_dword_t;

void *memcpy(void *dst, const void *src, size_t n)
{
  uint8_t *d = (uint8_t *)dst;
  const uint8_t *s = (const uint8_t *)src;

  if ((((uintptr_t)d ^ (uintptr_t)s) & 7u) == 0u) {
    while ((n != 0u) && ((((uintptr_t)d) & 7u) != 0u)) {
      *d = *s;
      d++;
      s++;
      n--;
    }
    while (n >= 32u) {
      n6_dword_t *dd = (n6_dword_t *)(void *)d;
      const n6_dword_t *sd = (const n6_dword_t *)(const void *)s;
      dd[0] = sd[0];
      dd[1] = sd[1];
      dd[2] = sd[2];
      dd[3] = sd[3];
      d += 32;
      s += 32;
      n -= 32u;
    }
    while (n >= 8u) {
      *(n6_dword_t *)(void *)d = *(const n6_dword_t *)(const void *)s;
      d += 8;
      s += 8;
      n -= 8u;
    }
  } else if ((((uintptr_t)d ^ (uintptr_t)s) & 3u) == 0u) {
    while ((n != 0u) && ((((uintptr_t)d) & 3u) != 0u)) {
      *d = *s;
      d++;
      s++;
      n--;
    }
    while (n >= 16u) {
      n6_word_t *dw = (n6_word_t *)(void *)d;
      const n6_word_t *sw = (const n6_word_t *)(const void *)s;
      dw[0] = sw[0];
      dw[1] = sw[1];
      dw[2] = sw[2];
      dw[3] = sw[3];
      d += 16;
      s += 16;
      n -= 16u;
    }
    while (n >= 4u) {
      *(n6_word_t *)(void *)d = *(const n6_word_t *)(const void *)s;
      d += 4;
      s += 4;
      n -= 4u;
    }
  }
  while (n != 0u) {
    *d = *s;
    d++;
    s++;
    n--;
  }
  return dst;
}

void *memset(void *dst, int c, size_t n)
{
  uint8_t *d = (uint8_t *)dst;
  uint8_t b = (uint8_t)c;

  while ((n != 0u) && ((((uintptr_t)d) & 3u) != 0u)) {
    *d = b;
    d++;
    n--;
  }
  if (n >= 4u) {
    n6_word_t w = (n6_word_t)b;
    w |= w << 8;
    w |= w << 16;
    while (n >= 16u) {
      n6_word_t *dw = (n6_word_t *)(void *)d;
      dw[0] = w;
      dw[1] = w;
      dw[2] = w;
      dw[3] = w;
      d += 16;
      n -= 16u;
    }
    while (n >= 4u) {
      *(n6_word_t *)(void *)d = w;
      d += 4;
      n -= 4u;
    }
  }
  while (n != 0u) {
    *d = b;
    d++;
    n--;
  }
  return dst;
}

void *memmove(void *dst, const void *src, size_t n)
{
  uint8_t *d = (uint8_t *)dst;
  const uint8_t *s = (const uint8_t *)src;

  if ((d == s) || (n == 0u)) {
    return dst;
  }
  /* Ascending order is safe unless the destination sits inside the source. */
  if ((d < s) || (d >= (s + n))) {
    return memcpy(dst, src, n);
  }
  d += n;
  s += n;
  while (n != 0u) {
    d--;
    s--;
    *d = *s;
    n--;
  }
  return dst;
}
