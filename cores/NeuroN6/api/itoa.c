/**
 ******************************************************************************
 * @file    api/itoa.c
 * @brief   Integer-to-ASCII conversions declared by api/itoa.h.
 ******************************************************************************
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "itoa.h"

static char *utoa_rec(unsigned long value, char *dst, int radix)
{
  static const char digits[] = "0123456789abcdefghijklmnopqrstuvwxyz";
  char buf[33];  /* enough for 32-bit binary + NUL */
  char *p = buf + sizeof(buf) - 1;

  *p = '\0';
  do {
    *--p = digits[value % (unsigned long)radix];
    value /= (unsigned long)radix;
  } while (value != 0);

  char *out = dst;
  while ((*out++ = *p++) != '\0') { }
  return dst;
}

char *ultoa(unsigned long value, char *string, int radix)
{
  if (string == 0 || radix < 2 || radix > 36) { return string; }
  return utoa_rec(value, string, radix);
}

char *utoa(unsigned value, char *string, int radix)
{
  return ultoa((unsigned long)value, string, radix);
}

char *ltoa(long value, char *string, int radix)
{
  if (string == 0 || radix < 2 || radix > 36) { return string; }
  if (radix == 10 && value < 0) {
    string[0] = '-';
    utoa_rec((unsigned long)(-(value + 1)) + 1UL, string + 1, radix);
    return string;
  }
  return utoa_rec((unsigned long)value, string, radix);
}

char *itoa(int value, char *string, int radix)
{
  return ltoa((long)value, string, radix);
}
