# Barcode1D

Decodes EAN-13, UPC-A and EAN-8 barcodes from an 8-bit grayscale image.

```c
#include "barcode1d.h"

barcode1d_result_t r;
if (barcode1d_scan_gray(frame, width, height, &r)) {
    /* r.sym, r.text (digits), r.x_start, r.x_end, r.y */
}
```

- `r.sym` is `BARCODE1D_EAN13`, `BARCODE1D_UPCA` or `BARCODE1D_EAN8`. An
  EAN-13 with a leading 0 is reported as UPC-A.
- One barcode per call, read on eleven horizontal scanlines between 20% and
  80% of the image height. The bars must be vertical; either direction reads.
- A result is returned only when its GS1 checksum passes.
- Lines are read up to `BARCODE1D_MAX_WIDTH`, 2600 pixels by default.
- No allocation and no floating point. The working buffers are static, about
  10 KB: call it from one task.

Example: `BarcodeReader`.

Licence: MIT. `pattern_variance()` in `src/barcode1d.c` is ported from ZXing's
`patternMatchVariance`, Apache-2.0; see `THIRD-PARTY-NOTICES.md` at the root of
the core.
