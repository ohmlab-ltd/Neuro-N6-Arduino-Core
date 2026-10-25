/**
 ******************************************************************************
 * @file    n6_asset.h
 * @brief   Small persistent host-authored data, written to flash by the FSBL.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6_ASSET_H
#define N6_ASSET_H

#include <stdint.h>
#include <stdbool.h>
#include "n6_ota_layout.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Record types. Must match N6_ASSET_REC_* in n6_host_contract.h. */
#ifndef N6_ASSET_REC_USER
#define N6_ASSET_REC_USER   1u
#define N6_ASSET_REC_WIFI   2u   /* MayaW476 WiFi settings */
#define N6_ASSET_REC_AP_PASSWORD 3u   /* MayaW476: the board's own SoftAP password */
#endif

/* Reads the user asset. Returns false if none is valid or it exceeds max. */
bool n6_asset_read(void *out, uint32_t max, uint32_t *out_len,
                   uint32_t *out_version);

/* True when a valid user asset exists. */
bool n6_asset_present(uint32_t *out_version, uint32_t *out_len);

/* The highest version a store takes. */
#define N6_ASSET_VERSION_MAX  0xFFFFFFFEu

/* Stages the user asset for the FSBL to write at the next boot. A version not
 * above the stored one is raised past it. False above N6_ASSET_VERSION_MAX or
 * with the store at it; an earlier stage stays armed. */
bool n6_asset_stage(const void *data, uint32_t len, uint32_t version);

/* Reads one record by its N6_ASSET_REC_* type, or returns false if it is
 * absent or exceeds max. With out NULL it reports the length only. */
bool n6_asset_record_read(uint8_t type, void *out, uint32_t max, uint32_t *out_len);

/* Replaces one record, or removes it when len is 0, and stages the store. */
bool n6_asset_record_stage(uint8_t type, const void *data, uint32_t len);

/* Clears N6_BOOT_PRESS_KEY and resets so the FSBL writes the staged asset.
 * Does not return. */
void n6_asset_apply(void);

/* True while a stage waits for the FSBL. */
bool n6_asset_pending(void);

/* N6_ASSET_RESULT_* from the FSBL's last attempt, or 0 when there is none. */
uint32_t n6_asset_last_result(void);
const char *n6_asset_result_str(uint32_t code);

#ifdef __cplusplus
}
#endif

#endif /* N6_ASSET_H */
