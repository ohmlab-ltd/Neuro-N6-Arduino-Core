/**
 ******************************************************************************
 * @file    maya_w4_sha256.h
 * @brief   SHA-256 and HMAC-SHA256 for the link key.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef MAYA_W4_SHA256_H
#define MAYA_W4_SHA256_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t h[8];
  uint64_t bytes;       /* message length so far */
  uint8_t  block[64];
  uint32_t fill;        /* bytes waiting in block */
} maya_w4_sha256_t;

void maya_w4_sha256_init(maya_w4_sha256_t *c);
void maya_w4_sha256_update(maya_w4_sha256_t *c, const void *data, size_t len);
void maya_w4_sha256_final(maya_w4_sha256_t *c, uint8_t out[32]);

/* HMAC-SHA256 of m1 then m2. Either may be empty. */
void maya_w4_hmac_sha256(const uint8_t *key, size_t key_len,
                         const void *m1, size_t n1, const void *m2, size_t n2,
                         uint8_t out[32]);

/* Zeroes n bytes that held a secret. The stores are volatile: a memset of a
 * buffer nothing reads again can be removed by the compiler. */
void maya_w4_wipe(void *p, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* MAYA_W4_SHA256_H */
