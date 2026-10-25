/* Forced include for the MSVC host build. The test driver is single-threaded,
 * so plain loads and stores stand in for GCC's atomics.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#ifndef HOST_PREFIX_H
#define HOST_PREFIX_H

#ifdef _MSC_VER

#define __attribute__(x)

#define __ATOMIC_RELAXED 0
#define __ATOMIC_CONSUME 1
#define __ATOMIC_ACQUIRE 2
#define __ATOMIC_RELEASE 3
#define __ATOMIC_ACQ_REL 4
#define __ATOMIC_SEQ_CST 5

static __inline unsigned char host_test_and_set(volatile unsigned char *p)
{
  const unsigned char was = *p;
  *p = 1u;
  return was;
}

#define __atomic_load_n(p, m)       (*(p))
#define __atomic_store_n(p, v, m)   ((void)(*(p) = (v)))
#define __atomic_clear(p, m)        ((void)(*(p) = 0))
#define __atomic_test_and_set(p, m) (host_test_and_set((volatile unsigned char *)(p)))

#endif /* _MSC_VER */

#endif /* HOST_PREFIX_H */
