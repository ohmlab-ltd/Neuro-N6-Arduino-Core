/* Barcode1D: self-contained 1D barcode decoder for the Neuro N6
 * Copyright (c) 2025-2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 *
 * Decodes EAN-13, UPC-A and EAN-8 barcodes from an 8-bit grayscale image.
 */

#ifndef BARCODE1D_H
#define BARCODE1D_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  BARCODE1D_NONE  = 0,
  BARCODE1D_EAN13 = 1,
  BARCODE1D_UPCA  = 2,
  BARCODE1D_EAN8  = 3
} barcode1d_sym_t;

typedef struct {
  barcode1d_sym_t sym;     /* BARCODE1D_NONE if nothing decoded */
  char            text[16];/* 13, 12 or 8 digits, NUL-terminated */
  int             x_start;  /* pixel x of the left edge on the decoded row */
  int             x_end;    /* pixel x of the right edge */
  int             y;        /* decoded row */
} barcode1d_result_t;

/* Scans an 8-bit grayscale image of stride w for one barcode. Returns 1 and
 * fills *out on success, 0 otherwise. Single-task use. */
int barcode1d_scan_gray(const uint8_t *gray, int w, int h, barcode1d_result_t *out);

/* From the last scan: start guards matched, and the most digits one attempt
 * reached, 0..12. Either argument may be NULL. */
void barcode1d_last_diag(int *guards, int *best_digits);

#ifdef __cplusplus
}
#endif
#endif
