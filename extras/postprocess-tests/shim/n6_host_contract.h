/* Host stand-in for vnd_meta.c. The real header asserts packed struct sizes,
 * which differ once host_prefix.h has defined __attribute__ away.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#ifndef N6_HOST_CONTRACT_H_SHIM
#define N6_HOST_CONTRACT_H_SHIM

#include <stdint.h>

#define N6_HOST_TAG_EVENT 0x67u
#define N6_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)

#endif
