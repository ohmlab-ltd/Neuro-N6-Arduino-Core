/**
 ******************************************************************************
 * @file    n6_ota_layout.h
 * @brief   OTA flash layout and control block, shared with the FSBL. The
 *          FSBL's copy of this file must match it.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6_OTA_LAYOUT_H
#define N6_OTA_LAYOUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Flash layout, 32 MB octal NOR. Offsets from the flash base.
 *   0x00000000  FSBL                    1 MB
 *   0x00100000  app slot A            1.5 MB
 *   0x00280000  control block copy 0    4 KB
 *   0x00281000  control block copy 1    4 KB
 *   0x00282000  asset store copy 0      4 KB
 *   0x00283000  asset store copy 1      4 KB
 *   0x00284000  reserved              496 KB
 *   0x00300000  NN weights pool      25.5 MB   the base cannot move
 *   0x01C80000  app slot B            1.5 MB
 *   0x01E00000  MAYA-W476 IW610 fw      2 MB
 */

#define N6_OTA_SLOT_A_OFF        0x00100000u
#define N6_OTA_SLOT_B_OFF        0x01C80000u
#define N6_OTA_SLOT_SIZE         0x00180000u   /* 1.5 MB each */

/* The loadable image limit: the FSBL copies this many bytes of a slot to
 * 0x34000000 and itself runs from 0x34180000. */
#define N6_OTA_MAX_LOADABLE      0x00180000u

#define N6_OTA_CB_OFF(copy)      (0x00280000u + ((copy) ? 0x1000u : 0u))
#define N6_OTA_CB_SECTOR         0x1000u       /* one 4 KB sector per copy */

/* Flash base as the app reads it, memory-mapped through XSPI1. */
#define N6_OTA_FLASH_MMAP_BASE   0x90000000u

/* PSRAM staging area */

/* The app fills this area, sets N6_OTA_TRIGGER_MAGIC in a backup register and
 * resets. The FSBL validates the image and copies it into the inactive slot. */

/* The top 2 MB of the 32 MB of PSRAM the core maps, reserved by the linker. */
#define N6_OTA_PSRAM_BASE        0x71E00000u
#define N6_OTA_PSRAM_HDR_MAGIC   0x4750364Eu   /* 'N6PG' little-endian */
#define N6_OTA_PSRAM_IMAGE_OFF   32u           /* image offset, bytes */

/* Backup register carrying the trigger. BKP1 and BKP2 hold the DFU state,
 * BKP23 to BKP31 the crash record. */
#define N6_OTA_TRIGGER_BKPR      20u
#define N6_OTA_TRIGGER_MAGIC     0x504F364Eu   /* 'N6OP' little-endian */

/* Backup register holding the outcome of the FSBL's last apply. */
#define N6_OTA_APPLY_BKPR        22u

#define N6_OTA_APPLY_OK             0x4F4B0000u  /* applied and pending */
#define N6_OTA_APPLY_STARTED        0x53540000u  /* trigger seen; got no further */
#define N6_OTA_APPLY_ERR_PSRAM_MAP  0x45500000u  /* could not map PSRAM */
#define N6_OTA_APPLY_ERR_HDR_MAGIC  0x45480000u  /* no staged header there */
#define N6_OTA_APPLY_ERR_HDR_CRC    0x45430000u  /* header did not check out */
#define N6_OTA_APPLY_ERR_SIZE       0x455A0000u  /* size zero or too large */
#define N6_OTA_APPLY_ERR_IMAGE_CRC  0x45490000u  /* staged image is not intact */
#define N6_OTA_APPLY_ERR_FLASH_CRC  0x45460000u  /* flash read-back did not match */

/* The FSBL's warm-reset mark. The app never writes it. */
#define N6_OTA_WARM_BKPR         14u

/* The FSBL writes this key to BKP2 before it starts the app. A reset in the
 * same RTC second with the key in place enters DFU. An app reset that is not
 * a bootloader request clears the key, and no other value. */
#define N6_BOOT_PRESS_BKPR       2u
#define N6_BOOT_PRESS_KEY        0x00FE4ADEu

/* Set by the app on every healthy boot. The FSBL then commits the pending
 * slot and stops counting tries. */
#define N6_OTA_CONFIRM_BKPR      21u
#define N6_OTA_CONFIRM_MAGIC     0x4B4F364Eu   /* 'N6OK' little-endian */

/* Staging header at N6_OTA_PSRAM_BASE, 20 bytes. */
typedef struct {
  uint32_t magic;                 /* N6_OTA_PSRAM_HDR_MAGIC */
  uint32_t size;                  /* image bytes */
  uint32_t version;               /* recorded in the control block */
  uint32_t crc32;                 /* zlib CRC32 over size image bytes */
  uint32_t hdr_crc;               /* zlib CRC32 over the preceding 16 bytes */
} n6_ota_psram_hdr_t;

/* Control block */

#define N6_OTA_CB_MAGIC          0x4E364F54u   /* 'TO6N' little-endian */

/* Slot state.
 *   EMPTY      not booted through the control block
 *   VALID      written and CRC-checked
 *   CONFIRMED  an app booted from it called n6_ota_confirm()
 */
#define N6_OTA_SLOT_EMPTY        0x00000000u
#define N6_OTA_SLOT_VALID        0x56414C44u   /* 'VALD' */
#define N6_OTA_SLOT_CONFIRMED    0x434F4E46u   /* 'CONF' */

#define N6_OTA_NO_PENDING        0xFFu

/* Boots a pending image gets to call n6_ota_confirm() before the FSBL
 * reverts to the slot that was active. */
#define N6_OTA_DEFAULT_TRIES     3u

/* 64 bytes, little-endian. With no valid copy the FSBL boots slot A. */
typedef struct {
  uint32_t magic;                /* N6_OTA_CB_MAGIC */
  uint32_t seq;                  /* monotonic; higher valid copy wins */
  uint8_t  active;               /* 0 = slot A, 1 = slot B */
  uint8_t  pending;              /* slot index, or N6_OTA_NO_PENDING */
  uint8_t  tries_left;           /* decremented by the FSBL per pending boot */
  uint8_t  reserved;
  struct {
    uint32_t crc32;              /* over size bytes from the slot start */
    uint32_t size;               /* bytes of the signed image as flashed */
    uint32_t version;            /* declared by the app, monotonic */
    uint32_t state;              /* EMPTY / VALID / CONFIRMED */
  } slot[2];
  uint8_t  pad[16];              /* zero-filled */
  uint32_t crc;                  /* CRC32 of the preceding 60 bytes */
} n6_ota_cb_t;

typedef char n6_ota_cb_size_check[(sizeof(n6_ota_cb_t) == 64) ? 1 : -1];

/* Shared helpers */

static inline uint32_t n6_ota_slot_off(uint8_t slot)
{
  return slot ? N6_OTA_SLOT_B_OFF : N6_OTA_SLOT_A_OFF;
}

/* Slot a flash offset falls in: 0, 1, or -1 for neither. */
static inline int n6_ota_slot_of_off(uint32_t off)
{
  if (off >= N6_OTA_SLOT_A_OFF && off < N6_OTA_SLOT_A_OFF + N6_OTA_SLOT_SIZE) return 0;
  if (off >= N6_OTA_SLOT_B_OFF && off < N6_OTA_SLOT_B_OFF + N6_OTA_SLOT_SIZE) return 1;
  return -1;
}

/* zlib-compatible CRC32: reflected, polynomial 0xEDB88320. */
static inline uint32_t n6_ota_crc32(uint32_t crc, const uint8_t *p, uint32_t len)
{
  static const uint32_t tab[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
    0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
    0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
  };
  crc = ~crc;
  while (len--) {
    crc ^= *p++;
    crc = (crc >> 4) ^ tab[crc & 0x0Fu];
    crc = (crc >> 4) ^ tab[crc & 0x0Fu];
  }
  return ~crc;
}

static inline uint32_t n6_ota_cb_crc(const n6_ota_cb_t *cb)
{
  return n6_ota_crc32(0u, (const uint8_t *)cb, sizeof(*cb) - 4u);
}

static inline int n6_ota_cb_valid(const n6_ota_cb_t *cb)
{
  return cb->magic == N6_OTA_CB_MAGIC && cb->crc == n6_ota_cb_crc(cb);
}

/* Asset store */

/* Two copies, one 4 KB sector each. A write lands in the copy that is not
 * valid, and the reader takes the highest version that passes its CRC. */

#define N6_ASSET_OFF(copy)       (0x00282000u + ((copy) ? 0x1000u : 0u))
#define N6_ASSET_SECTOR          0x1000u
#define N6_ASSET_COPIES          2u

/* The header starts the sector and the payload follows it. */
#define N6_ASSET_MAGIC           0x54534136u   /* '6AST' little-endian */
#define N6_ASSET_HDR_BYTES       20u
#define N6_ASSET_MAX_BYTES       (N6_ASSET_SECTOR - N6_ASSET_HDR_BYTES)

#define N6_ASSET_READ_TRIES      3u

typedef struct __attribute__((packed)) {
  uint32_t magic;      /* N6_ASSET_MAGIC */
  uint32_t version;    /* monotonic; highest valid copy wins */
  uint32_t len;        /* payload bytes following this header */
  uint32_t crc32;      /* over the payload */
  uint32_t hdr_crc;    /* over the first 16 bytes */
} n6_asset_hdr_t;

/* Asset staging area in PSRAM. */
#define N6_ASSET_PSRAM_BASE      0x71F80000u
#define N6_ASSET_PSRAM_MAGIC     0x47534136u   /* '6ASG' */

#define N6_ASSET_TRIGGER_BKPR    16u
#define N6_ASSET_TRIGGER_MAGIC   0x57534136u   /* '6ASW' */
#define N6_ASSET_RESULT_BKPR     15u

#define N6_ASSET_RESULT_OK          1u
#define N6_ASSET_RESULT_ERR_HDR     2u
#define N6_ASSET_RESULT_ERR_CRC     3u
#define N6_ASSET_RESULT_ERR_SIZE    4u
#define N6_ASSET_RESULT_ERR_VERIFY  5u

#ifdef __cplusplus
}
#endif

#endif /* N6_OTA_LAYOUT_H */
