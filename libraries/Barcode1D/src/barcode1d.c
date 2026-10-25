/* Barcode1D: self-contained 1D barcode decoder for the Neuro N6
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT AND Apache-2.0
 *
 * pattern_variance() follows ZXing's patternMatchVariance, Copyright 2008
 * ZXing authors, under the Apache License 2.0. The rest is MIT.
 */

/* Integer maths only. The work buffers are static: single-task use. */

#include <stdint.h>
#include <string.h>

#include "barcode1d.h"

/* Widest scanline decoded, in pixels. Columns beyond it are ignored. */
#ifndef BARCODE1D_MAX_WIDTH
#define BARCODE1D_MAX_WIDTH 2600
#endif
#if BARCODE1D_MAX_WIDTH > 65535
#error "BARCODE1D_MAX_WIDTH must fit in uint16_t (<= 65535)"
#endif

#define VAR_SCALE            256   /* fixed-point scale */
#define MAX_AVG_VAR          123   /* average variance limit, 0.48 * 256 */
#define MAX_INDIVIDUAL_VAR   179   /* single-run variance limit, 0.70 * 256 */
#define NO_MATCH             0x40000000

#define MIN_CONTRAST         24    /* smallest max-min grey spread of a line */
#define MIN_RUNS             43    /* runs in an EAN-8, the shortest symbol */
#define NUM_SCANLINES        11

/* L run lengths in modules. R has the same run lengths with the colours
 * swapped, and G is L reversed. */
static const uint8_t L_PATTERNS[10][4] = {
  {3, 2, 1, 1}, /* 0 */
  {2, 2, 2, 1}, /* 1 */
  {2, 1, 2, 2}, /* 2 */
  {1, 4, 1, 1}, /* 3 */
  {1, 1, 3, 2}, /* 4 */
  {1, 2, 3, 1}, /* 5 */
  {1, 1, 1, 4}, /* 6 */
  {1, 3, 1, 2}, /* 7 */
  {1, 2, 1, 3}, /* 8 */
  {3, 1, 1, 2}  /* 9 */
};

static const int START_GUARD[3]  = {1, 1, 1};       /* 101   */
static const int CENTER_GUARD[5] = {1, 1, 1, 1, 1}; /* 01010 */

/* Parity pattern of each EAN-13 leading digit: bit 5-x is set when left
 * digit x is G. Leading digit 0 is UPC-A. */
static const uint8_t FIRST_DIGIT_ENC[10] = {
  0x00, 0x0B, 0x0D, 0x0E, 0x13, 0x19, 0x1C, 0x15, 0x16, 0x1A
};

/* edge[i] is the column where run i begins, edge[n_runs] the column count. */
static uint16_t s_edge[BARCODE1D_MAX_WIDTH + 1];
static uint16_t s_edge_rev[BARCODE1D_MAX_WIDTH + 1];

/* For barcode1d_last_diag(). */
static int s_diag_guards;
static int s_diag_best;

/* Non-zero when run j is a bar. */
#define IS_BAR(first_color, j) ((first_color) ^ ((j) & 1))

static int run_w(const uint16_t *edge, int i)
{
  return (int)edge[i + 1] - (int)edge[i];
}

static void get4(const uint16_t *edge, int idx, int *c)
{
  c[0] = run_w(edge, idx);
  c[1] = run_w(edge, idx + 1);
  c[2] = run_w(edge, idx + 2);
  c[3] = run_w(edge, idx + 3);
}

/* Variance of the runs against a pattern, normalised by total run width as
 * in ZXing's patternMatchVariance, or NO_MATCH when one run is too far off.
 * pattern_total is the pattern's module count. */
static int pattern_variance(const int *counters, const int *pattern,
                            int len, int pattern_total)
{
  int total = 0;
  int unit, max_ind, tot_var = 0;
  int i;

  for (i = 0; i < len; i++)
    total += counters[i];
  if (total < pattern_total)
    return NO_MATCH; /* fewer pixels than modules */

  unit = (total * VAR_SCALE) / pattern_total;       /* module width * 256 */
  max_ind = (unit * MAX_INDIVIDUAL_VAR) / VAR_SCALE;

  for (i = 0; i < len; i++) {
    int measured = counters[i] * VAR_SCALE;
    int expected = pattern[i] * unit;
    int v = measured - expected;
    if (v < 0)
      v = -v;
    if (v > max_ind)
      return NO_MATCH;
    tot_var += v;
  }
  return tot_var / total;
}

/* Digit from its 4 run widths: 0..9 for L and, in mode 1, 10..19 for G.
 * Returns -1 when nothing is within tolerance. */
static int decode_digit(const int *counters, int mode)
{
  int best = -1;
  int best_var = MAX_AVG_VAR + 1;
  int limit = (mode == 1) ? 20 : 10;
  int d;

  for (d = 0; d < limit; d++) {
    int pat[4];
    int v;
    if (d < 10) {
      pat[0] = L_PATTERNS[d][0];
      pat[1] = L_PATTERNS[d][1];
      pat[2] = L_PATTERNS[d][2];
      pat[3] = L_PATTERNS[d][3];
    } else {
      pat[0] = L_PATTERNS[d - 10][3];
      pat[1] = L_PATTERNS[d - 10][2];
      pat[2] = L_PATTERNS[d - 10][1];
      pat[3] = L_PATTERNS[d - 10][0];
    }
    v = pattern_variance(counters, pat, 4, 7);
    if (v < best_var) {
      best_var = v;
      best = d;
    }
  }
  if (best < 0 || best_var > MAX_AVG_VAR)
    return -1;
  return best;
}

/* GS1 mod-10 check over n ASCII digits. d[n-1] is the check digit. */
static int checksum_ok(const char *d, int n)
{
  int sum = 0;
  int weight = 3;
  int i;
  for (i = n - 2; i >= 0; i--) {
    sum += (d[i] - '0') * weight;
    weight = (weight == 3) ? 1 : 3;
  }
  return ((10 - (sum % 10)) % 10) == (d[n - 1] - '0');
}

/* EAN-13 leading digit from the 6-bit parity pattern, or -1. */
static int first_digit_from_parity(int lg)
{
  int i;
  for (i = 0; i < 10; i++)
    if (FIRST_DIGIT_ENC[i] == lg)
      return i;
  return -1;
}

/* Decodes EAN-13 or UPC-A after a start guard at runs start to start+2.
 * Returns 1 and fills the outputs on success. */
static int decode_ean13(const uint16_t *edge, int n_runs, int first_color,
                        int start, barcode1d_sym_t *sym, char *text,
                        int *lcol, int *rcol)
{
  int idx, x, c[4], d, lg = 0, first, got = 0;
  char digits[14];

  (void)first_color;
  /* 3 + 6*4 + 5 + 6*4 + 3 = 59 runs. */
  if (start + 59 > n_runs)
    return 0;
  /* The quiet zone is not tested: the guards and checksum reject false hits. */

  idx = start + 3;
  for (x = 0; x < 6; x++) {
    get4(edge, idx, c);
    d = decode_digit(c, 1);
    if (d < 0)
      return 0;
    if (d >= 10) {          /* G */
      lg |= (1 << (5 - x));
      d -= 10;
    }
    digits[1 + x] = (char)('0' + d);
    if (++got > s_diag_best) s_diag_best = got;
    idx += 4;
  }

  {
    int cg[5];
    cg[0] = run_w(edge, idx);
    cg[1] = run_w(edge, idx + 1);
    cg[2] = run_w(edge, idx + 2);
    cg[3] = run_w(edge, idx + 3);
    cg[4] = run_w(edge, idx + 4);
    if (pattern_variance(cg, CENTER_GUARD, 5, 5) > MAX_AVG_VAR)
      return 0;
  }
  idx += 5;

  for (x = 0; x < 6; x++) {
    get4(edge, idx, c);
    d = decode_digit(c, 0);
    if (d < 0)
      return 0;
    digits[7 + x] = (char)('0' + d);
    if (++got > s_diag_best) s_diag_best = got;
    idx += 4;
  }

  /* The end guard has the start guard's pattern. */
  {
    int eg[3];
    eg[0] = run_w(edge, idx);
    eg[1] = run_w(edge, idx + 1);
    eg[2] = run_w(edge, idx + 2);
    if (pattern_variance(eg, START_GUARD, 3, 3) > MAX_AVG_VAR)
      return 0;
  }

  first = first_digit_from_parity(lg);
  if (first < 0)
    return 0;
  digits[0] = (char)('0' + first);
  digits[13] = '\0';

  if (!checksum_ok(digits, 13))
    return 0;

  *lcol = edge[start];
  *rcol = edge[idx + 3];      /* right edge of the end guard */

  if (first == 0) {
    /* UPC-A drops the leading zero. */
    int i;
    for (i = 0; i < 12; i++)
      text[i] = digits[i + 1];
    text[12] = '\0';
    *sym = BARCODE1D_UPCA;
  } else {
    int i;
    for (i = 0; i < 13; i++)
      text[i] = digits[i];
    text[13] = '\0';
    *sym = BARCODE1D_EAN13;
  }
  return 1;
}

/* The same for EAN-8. */
static int decode_ean8(const uint16_t *edge, int n_runs, int first_color,
                       int start, barcode1d_sym_t *sym, char *text,
                       int *lcol, int *rcol)
{
  int idx, x, c[4], d, got = 0;
  char digits[9];

  (void)first_color;
  /* 3 + 4*4 + 5 + 4*4 + 3 = 43 runs. */
  if (start + 43 > n_runs)
    return 0;

  idx = start + 3;
  for (x = 0; x < 4; x++) {          /* left digits, L */
    get4(edge, idx, c);
    d = decode_digit(c, 0);
    if (d < 0)
      return 0;
    digits[x] = (char)('0' + d);
    if (++got > s_diag_best) s_diag_best = got;
    idx += 4;
  }

  {
    int cg[5];
    cg[0] = run_w(edge, idx);
    cg[1] = run_w(edge, idx + 1);
    cg[2] = run_w(edge, idx + 2);
    cg[3] = run_w(edge, idx + 3);
    cg[4] = run_w(edge, idx + 4);
    if (pattern_variance(cg, CENTER_GUARD, 5, 5) > MAX_AVG_VAR)
      return 0;
  }
  idx += 5;

  for (x = 0; x < 4; x++) {          /* right digits, R */
    get4(edge, idx, c);
    d = decode_digit(c, 0);
    if (d < 0)
      return 0;
    digits[4 + x] = (char)('0' + d);
    if (++got > s_diag_best) s_diag_best = got;
    idx += 4;
  }

  {
    int eg[3];
    eg[0] = run_w(edge, idx);
    eg[1] = run_w(edge, idx + 1);
    eg[2] = run_w(edge, idx + 2);
    if (pattern_variance(eg, START_GUARD, 3, 3) > MAX_AVG_VAR)
      return 0;
  }

  digits[8] = '\0';
  if (!checksum_ok(digits, 8))
    return 0;

  *lcol = edge[start];
  *rcol = edge[idx + 3];
  {
    int i;
    for (i = 0; i < 8; i++)
      text[i] = digits[i];
    text[8] = '\0';
  }
  *sym = BARCODE1D_EAN8;
  return 1;
}

/* Tries EAN-13 then EAN-8 at every start guard. Returns 1 on the first valid
 * symbol. */
static int decode_core(const uint16_t *edge, int n_runs, int first_color,
                       barcode1d_sym_t *sym, char *text, int *lcol, int *rcol)
{
  int start;
  for (start = 0; start + 3 <= n_runs; start++) {
    int c3[3];
    if (!IS_BAR(first_color, start))
      continue;                       /* a guard begins on a bar */
    c3[0] = run_w(edge, start);
    c3[1] = run_w(edge, start + 1);
    c3[2] = run_w(edge, start + 2);
    if (pattern_variance(c3, START_GUARD, 3, 3) > MAX_AVG_VAR)
      continue;
    s_diag_guards++;
    if (decode_ean13(edge, n_runs, first_color, start, sym, text, lcol, rcol))
      return 1;
    if (decode_ean8(edge, n_runs, first_color, start, sym, text, lcol, rcol))
      return 1;
  }
  return 0;
}

/* Thresholds one scanline at its midpoint, with hysteresis, and run-length
 * encodes it into edge[]. Returns 1 when it has MIN_CONTRAST and MIN_RUNS. */
static int encode_edges(const uint8_t *row, int P, uint16_t *edge,
                        int *n_runs_out, int *first_color_out)
{
  int i, mn = 255, mx = 0, mid, hyst, hi, lo, color, runs;

  for (i = 0; i < P; i++) {
    int v = row[i];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  if ((mx - mn) < MIN_CONTRAST)
    return 0;

  mid = (mn + mx) / 2;
  hyst = (mx - mn) / 16;              /* hysteresis half-band */
  hi = mid + hyst;
  lo = mid - hyst;

  /* 1 = dark bar, 0 = light space */
  color = (row[0] <= mid) ? 1 : 0;
  *first_color_out = color;
  edge[0] = 0;
  runs = 0;

  for (i = 1; i < P; i++) {
    int v = row[i];
    int c;
    if (v <= lo)
      c = 1;
    else if (v >= hi)
      c = 0;
    else
      c = color;
    if (c != color) {
      runs++;
      edge[runs] = (uint16_t)i;
      color = c;
    }
  }
  runs++;
  edge[runs] = (uint16_t)P;

  *n_runs_out = runs;
  return (runs >= MIN_RUNS) ? 1 : 0;
}

static void set_out(barcode1d_result_t *out, barcode1d_sym_t sym,
                    const char *text, int x_start, int x_end, int y)
{
  int i;
  out->sym = sym;
  for (i = 0; i < 15 && text[i]; i++)
    out->text[i] = text[i];
  out->text[i] = '\0';
  out->x_start = x_start;
  out->x_end = x_end;
  out->y = y;
}

/* Decodes one scanline, as scanned and then reversed. */
static int try_decode_line(const uint8_t *row, int P, int y,
                           barcode1d_result_t *out)
{
  int n_runs, fc, i, rev_fc, l, r;
  barcode1d_sym_t sym;
  char text[16];

  if (!encode_edges(row, P, s_edge, &n_runs, &fc))
    return 0;

  if (decode_core(s_edge, n_runs, fc, &sym, text, &l, &r)) {
    set_out(out, sym, text, l, r, y);
    return 1;
  }

  /* Reversed: the run boundaries mirrored about P. */
  for (i = 0; i <= n_runs; i++)
    s_edge_rev[i] = (uint16_t)(P - (int)s_edge[n_runs - i]);
  rev_fc = fc ^ ((n_runs - 1) & 1);

  if (decode_core(s_edge_rev, n_runs, rev_fc, &sym, text, &l, &r)) {
    /* Columns back in the image frame. */
    set_out(out, sym, text, P - r, P - l, y);
    return 1;
  }
  return 0;
}

int barcode1d_scan_gray(const uint8_t *gray, int w, int h, barcode1d_result_t *out)
{
  /* Scanline rows, percent of the image height. */
  static const int PCT[NUM_SCANLINES] = {20, 26, 32, 38, 44, 50, 56, 62, 68, 74, 80};
  int k, P;

  if (!gray || !out || w <= 0 || h <= 0)
    return 0;

  out->sym = BARCODE1D_NONE;
  out->text[0] = '\0';
  out->x_start = 0;
  out->x_end = 0;
  out->y = 0;
  s_diag_guards = 0;
  s_diag_best = 0;

  P = (w < BARCODE1D_MAX_WIDTH) ? w : BARCODE1D_MAX_WIDTH;

  for (k = 0; k < NUM_SCANLINES; k++) {
    int y = (h * PCT[k]) / 100;
    if (y < 0)
      y = 0;
    if (y >= h)
      y = h - 1;
    if (try_decode_line(gray + (size_t)y * (size_t)w, P, y, out))
      return 1;
  }

  out->sym = BARCODE1D_NONE;
  return 0;
}

void barcode1d_last_diag(int *guards, int *best_digits)
{
  if (guards) *guards = s_diag_guards;
  if (best_digits) *best_digits = s_diag_best;
}
