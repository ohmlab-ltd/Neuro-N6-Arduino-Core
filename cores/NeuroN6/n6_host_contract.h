/**
 ******************************************************************************
 * @file    n6_host_contract.h
 * @brief   The contract between a Neuro N6 board and an attached host.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6_HOST_CONTRACT_H
#define N6_HOST_CONTRACT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__cplusplus)
#  define N6_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#  define N6_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#else
#  define N6_STATIC_ASSERT(cond, msg) /* unavailable on this dialect */
#endif

/* Traffic classes. Uplink, device to host, is TLV on the vendor bulk-IN stream.
 * Downlink is magic-prefixed binary on the vendor bulk-OUT endpoint.
 *   CAPABILITY  uplink    what this build can do, at about 1 Hz
 *   RESULT      uplink    what the device decided, never dropped
 *   EVIDENCE    uplink    for a human, droppable and host-gated
 *   PARAMETER   downlink  a live value, not persisted
 *   ASSET       downlink  data kept across a power cycle */
typedef enum
{
  N6_HOST_CLASS_CAPABILITY = 0,
  N6_HOST_CLASS_RESULT     = 1,
  N6_HOST_CLASS_EVIDENCE   = 2,
  N6_HOST_CLASS_PARAMETER  = 3,
  N6_HOST_CLASS_ASSET      = 4
} n6_host_class_t;

/* RESULT class. Must equal META_EVENT in vnd_meta.h. */
#define N6_HOST_TAG_EVENT           0x67u

/* Evidence tags. A transport may drop a tag in this range. */
#define N6_HOST_EVIDENCE_TAG_FIRST  0x70u
#define N6_HOST_EVIDENCE_TAG_LAST   0x7Fu

#define N6_HOST_TAG_IS_EVIDENCE(tag) \
  ((uint8_t)(tag) >= N6_HOST_EVIDENCE_TAG_FIRST && \
   (uint8_t)(tag) <= N6_HOST_EVIDENCE_TAG_LAST)

/* Class of an uplink TLV tag. An unknown tag is a RESULT. */
static inline n6_host_class_t n6_host_tag_class(uint8_t tag)
{
  if (N6_HOST_TAG_IS_EVIDENCE(tag)) return N6_HOST_CLASS_EVIDENCE;

  switch (tag) {
    case 0x20u:   /* META_BOARD_IPV4        */
    case 0x21u:   /* META_BOARD_NAME        */
    case 0x22u:   /* META_BOARD_LINK        */
    case 0x23u:   /* META_BOARD_CHALLENGE   */
    case 0x40u:   /* META_VD66_ISP_ENABLED  */
    case 0x53u:   /* META_AUDIO_LABELS      */
    case 0x60u:   /* META_VISION_ID         */
    case 0x64u:   /* META_CAM_CAPS          */
    case 0x65u:   /* META_TUNE_DESC         */
      return N6_HOST_CLASS_CAPABILITY;

    /* Evidence tags outside the range. */
    case 0x17u:   /* META_DEBUG_TEXT        */
    case 0x52u:   /* META_AUDIO_MEL         */
      return N6_HOST_CLASS_EVIDENCE;

    default:
      return N6_HOST_CLASS_RESULT;
  }
}

/* zlib-compatible CRC32: reflected, polynomial 0xEDB88320. Must match
 * n6_ota_crc32 in n6_ota_layout.h and the FSBL's copy. */
static inline uint32_t n6_host_crc32(uint32_t crc, const uint8_t *p, uint32_t len)
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

/* Downlink commands */

/* Largest host-to-device command, in bytes. */
#define N6_HOST_CMD_MAX  64u

/* Four-character magic, little-endian. */
#define N6_HOST_MAGIC(a, b, c, d)                                    \
  ((uint32_t)(uint8_t)(a)        | ((uint32_t)(uint8_t)(b) << 8) |   \
   ((uint32_t)(uint8_t)(c) << 16) | ((uint32_t)(uint8_t)(d) << 24))

#define N6_CMD_ISP0  N6_HOST_MAGIC('I', 'S', 'P', '0')  /* live ISP tuning     */
#define N6_CMD_CAM0  N6_HOST_MAGIC('C', 'A', 'M', '0')  /* capture controls    */
#define N6_CMD_TUN0  N6_HOST_MAGIC('T', 'U', 'N', '0')  /* one tunable         */
#define N6_CMD_SUB0  N6_HOST_MAGIC('S', 'U', 'B', '0')  /* evidence subscribe  */
#define N6_CMD_AST0  N6_HOST_MAGIC('A', 'S', 'T', '0')  /* asset: begin        */
#define N6_CMD_AST1  N6_HOST_MAGIC('A', 'S', 'T', '1')  /* asset: chunk        */
#define N6_CMD_AST2  N6_HOST_MAGIC('A', 'S', 'T', '2')  /* asset: commit+reset */

/* 'ISP0' is variable length. */
#define N6_CMD_ISP0_MIN_BYTES  13u

typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_CAM0 */
  uint8_t  flags;        /* bit0 auto exposure, bit1 auto gain, bit2 auto AWB */
  uint32_t exposure_us;
  uint16_t gain_x256;    /* 256 = 1.0x */
  uint8_t  apply_mask;   /* which fields of this packet are meaningful */
} n6_cmd_cam0_t;

/* Evidence subscription. It lapses N6_HOST_SUB_TTL_MS after the last SUB0. */
typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_SUB0 */
  uint32_t mask;         /* N6_EVIDENCE_* bits; 0 unsubscribes */
} n6_cmd_sub0_t;

#define N6_EVIDENCE_CROP  (1u << 0)   /* per-result crop thumbnails */

#define N6_HOST_SUB_TTL_MS  3000u

/* Asset transfer. AST0 begins and discards a partial transfer. AST1 carries a
 * chunk at an offset. AST2 checks the CRC, commits and resets the board. */
typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_AST0 */
  uint32_t total_len;
  uint32_t crc32;        /* over the whole payload */
  uint32_t version;      /* 0: one past the stored. At most 0xFFFFFFFE */
} n6_cmd_ast0_t;

#define N6_ASSET_CHUNK_MAX  48u

typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_AST1 */
  uint16_t offset;
  uint8_t  len;          /* <= N6_ASSET_CHUNK_MAX */
  /* uint8_t data[len] follows */
} n6_cmd_ast1_t;

typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_AST2 */
} n6_cmd_ast2_t;

/* Record transfer: AST1 chunks and an AST2 commit, as for an asset. The commit
 * replaces one typed record and keeps the asset and the other records. */
#define N6_CMD_ASR0  N6_HOST_MAGIC('A', 'S', 'R', '0')  /* record: begin       */

typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_ASR0 */
  uint32_t total_len;    /* 0 removes the record */
  uint32_t crc32;        /* over the whole record */
  uint8_t  type;         /* N6_ASSET_REC_* */
  uint8_t  reserved[3];
} n6_cmd_asr0_t;

/* Record types. */
#define N6_ASSET_REC_USER   1u   /* the asset AST0 writes */
#define N6_ASSET_REC_WIFI   2u   /* MayaW476 WiFi settings */
#define N6_ASSET_REC_AP_PASSWORD 3u   /* MayaW476: the board's own SoftAP password */

/* The WiFi record is TLV: { tag:u8 len:u8 value[len] }, repeated. Unknown tags
 * are skipped. */
#define N6_WIFI_TLV_MODE      1u   /* u8: 1 join a network, 2 host one        */
#define N6_WIFI_TLV_SSID      2u   /* 1..32 bytes                              */
#define N6_WIFI_TLV_KEY       3u   /* passphrase 8..63 bytes; absent: hosting
                                      uses the board's own password            */
#define N6_WIFI_TLV_SECURITY  4u   /* u8 WifiSecurity, WPA2 or WPA3; absent: WPA2 */
#define N6_WIFI_TLV_CHANNEL   5u   /* u8, hosting only; absent: 6              */
#define N6_WIFI_TLV_HOSTNAME  6u   /* 1..32 bytes: DHCP and mDNS name          */
#define N6_WIFI_TLV_LINK_KEY  7u   /* 8..64 bytes: a host must prove it has it */

typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_CMD_TUN0 */
  uint8_t  id;           /* index into the tune registry */
  uint8_t  type;         /* 0 int, 1 float (scaled x1000), 2 bool */
  int32_t  value;
} n6_cmd_tun0_t;

/* Network link */

/* Discovery: a datagram of at least a header's length that starts with
 * N6_FRAME_MAGIC, sent to UDP port N6_NET_PORT, is answered with one frame:
 * META_BOARD_IPV4 (0x20), the IPv4 address and TCP port, then the TLVs below. */
#define N6_META_BOARD_NAME       0x21u   /* UTF-8, up to 32 bytes, no terminator */
#define N6_META_BOARD_LINK       0x22u   /* u8 flags, only with a link key */
#define N6_BOARD_LINK_KEYED      0x01u   /* a link key must be proved first */

/* USB only, never on a network link: the access point the board hosts, about
 * once a second while it is up. { ssid_len:u8 ssid pass_len:u8 passphrase } */
#define N6_META_BOARD_SOFTAP     0x24u

/* Link key, optional. A connecting host first receives one frame whose only
 * TLV is META_BOARD_CHALLENGE, then has N6_LINK_ANSWER_MS to send one command:
 *   'KEY0'  HMAC-SHA256(key, "N6LINK1" || challenge)        4 + 32 bytes
 * Anything else closes the connection. The key is 8 to 64 bytes. */
#define N6_META_BOARD_CHALLENGE  0x23u
#define N6_LINK_CHALLENGE_BYTES  16u
#define N6_LINK_ANSWER_MS        10000u
#define N6_LINK_CONTEXT          "N6LINK1"   /* HMAC'd ahead of the challenge */
#define N6_CMD_KEY0  N6_HOST_MAGIC('K', 'E', 'Y', '0')

/* The first four bytes of every vendor frame, "DNV0" on the wire. */
#define N6_FRAME_MAGIC  N6_HOST_MAGIC('D', 'N', 'V', '0')

/* Uplink frames are this header, the TLVs, then the payload. On a network link
 * a downlink command follows the same header, with meta_len 0 and payload_len
 * the command length. A network host may send ISP0, CAM0, TUN0 and SUB0 only.
 * Any other framing closes the connection. */
typedef struct __attribute__((packed))
{
  uint32_t magic;        /* N6_FRAME_MAGIC */
  uint32_t frame_id;
  uint32_t meta_len;     /* TLV bytes after the header */
  uint32_t payload_len;  /* bytes after the TLVs */
  uint32_t flags;        /* uplink bit0: the payload is a JPEG */
} n6_frame_hdr_t;

/* Default TCP port of the stream, and the UDP port for discovery. */
#define N6_NET_PORT  5000u

N6_STATIC_ASSERT(sizeof(n6_cmd_cam0_t) == 12u, "CAM0 wire format changed");
N6_STATIC_ASSERT(sizeof(n6_cmd_tun0_t) == 10u, "TUN0 wire format changed");
N6_STATIC_ASSERT(sizeof(n6_cmd_sub0_t) == 8u,  "SUB0 wire format changed");
N6_STATIC_ASSERT(sizeof(n6_cmd_ast0_t) == 16u, "AST0 wire format changed");
N6_STATIC_ASSERT(sizeof(n6_cmd_ast1_t) == 7u,  "AST1 wire format changed");
N6_STATIC_ASSERT(sizeof(n6_cmd_asr0_t) == 16u, "ASR0 wire format changed");
N6_STATIC_ASSERT(sizeof(n6_cmd_ast1_t) + N6_ASSET_CHUNK_MAX <= N6_HOST_CMD_MAX,
                 "an AST1 chunk does not fit the command buffer");
N6_STATIC_ASSERT(sizeof(n6_frame_hdr_t) == 20u, "frame header wire format changed");
N6_STATIC_ASSERT(N6_FRAME_MAGIC == 0x30564E44u, "frame magic changed");

N6_STATIC_ASSERT(sizeof(n6_cmd_cam0_t) <= N6_HOST_CMD_MAX,
                 "CAM0 does not fit the command buffer");
N6_STATIC_ASSERT(sizeof(n6_cmd_tun0_t) <= N6_HOST_CMD_MAX,
                 "TUN0 does not fit the command buffer");
N6_STATIC_ASSERT(sizeof(n6_cmd_sub0_t) <= N6_HOST_CMD_MAX,
                 "SUB0 does not fit the command buffer");
N6_STATIC_ASSERT(N6_CMD_ISP0_MIN_BYTES <= N6_HOST_CMD_MAX,
                 "ISP0 does not fit the command buffer");

N6_STATIC_ASSERT(N6_HOST_EVIDENCE_TAG_FIRST > N6_HOST_TAG_EVENT,
                 "evidence range overlaps an allocated tag");
N6_STATIC_ASSERT(N6_HOST_EVIDENCE_TAG_LAST >= N6_HOST_EVIDENCE_TAG_FIRST,
                 "evidence range is inverted");

#ifdef __cplusplus
}
#endif

#endif /* N6_HOST_CONTRACT_H */
