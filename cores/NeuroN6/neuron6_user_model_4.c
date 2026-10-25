/* User model slot 4. Each generated network has its own translation unit:
 * the generated files redefine file-scope helpers and macros.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* Places the model's ec_blobs in .ext_psram_init, which is in PSRAM. */
#define ECBLOB_CONST_SECTION __attribute__((section(".ext_psram_init")))

#if defined(__has_include)
#  if __has_include("neuron6_generated_model_4.inc")
#    include "neuron6_generated_model_4.inc"
#  endif
#endif
