/**
 * @file    vnd_meta.h
 * @brief   Vendor metadata TLV format and API.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef VND_META_H
#define VND_META_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* arm_math.h is not included: its arm_mve.h conflicts with extern "C". */
#ifndef _ARM_MATH_TYPES_H_
typedef float float32_t;
#endif

/* TLV types */
enum {
  META_INFERENCE_MS      = 1,     /* u16 ms */
  META_BOXES             = 2,     /* u8 count, per box u16 cx, cy, w, h,
                                     u8 conf, u8 class */
  META_KEYPOINTS         = 3,     /* u8 count, per point u8 id, colour,
                                     u16 x, y, u8 conf */
  META_BINDINGS          = 4,     /* u8 count, per line u8 a, b, colour */
  META_ISEG_CLASSSTR     = 10,    /* u8 obj_id, name */
  META_ISEG_MASK         = 11,    /* u8 obj_id, w, h, u16 bytes, w*h bits,
                                     LSB first */
  META_TOF_FRAME_LEGACY  = 0x14,  /* 0x15 is also a ToF frame on the host. */
  META_DEBUG_TEXT        = 0x17,  /* ASCII text, the latest line, no terminator. */
  META_MAG_FRAME         = 0x1A,  /* MMC5603NJ: 4 x float32 LE: x, y, z, degC. */
  META_IMU_FRAME         = 0x1B,  /* LSM6DSL: 7 x float32 LE: accel xyz in g,
                                     gyro xyz in dps, degC. */
  META_THERMAL_FRAME     = 0x30,  /* FLIR Lepton:
                                       u16 width, height
                                       u8  format     0 = RAW16 LE
                                       u8  flags      bit0: radiometric, centikelvin
                                       u16 reserved
                                       u16 LE pixels[width * height], row-major */
  META_BOARD_IPV4        = 0x20,  /* 4 B IPv4, MSB-first, then u32 LE TCP port.
                                     Sent at about 1 Hz while on WiFi. */
  META_BOARD_SOFTAP      = 0x24,  /* u8 ssid_len, ssid, u8 pass_len, passphrase
                                     of the board's access point. USB only, at
                                     about 1 Hz while it is up. */
  META_VD66_ISP_ENABLED  = 0x40,  /* u8 mode, 1 = VD66 manual ISP. Sent at about
                                     1 Hz while it is enabled. */
  META_AUDIO_PCM         = 0x50,  /* Microphone audio, an ordered stream:
                                       u32 sample_rate   Hz, LE
                                       u8  channels      1 = mono
                                       u8  bits          16
                                       u16 sample_count  LE
                                       i16 pcm[sample_count], LE */

  META_AUDIO_CLASS       = 0x51,  /* Audio classifier result:
                                       u8  count
                                       u8  top              argmax, 0xFF = none
                                                            above the threshold
                                       u16 level            peak sample, 0..32767
                                       u16 threshold_x1000
                                       per class: u16 score_x1000 */
  META_AUDIO_LABELS      = 0x53,  /* Names for META_AUDIO_CLASS, at about 1 Hz:
                                     u8 count, then per class u8 len, chars. */
  META_AUDIO_MEL         = 0x52,  /* Log-mel spectrogram, the model's input:
                                       u8  n_mel
                                       u8  n_frames
                                       u8  layout     0 = frame-major, 1 = mel-major
                                       u8  reserved
                                       u8  mel[n_mel * n_frames] */
  META_IMAGE_CLASS       = 0x54,  /* Image classifier result:
                                       u8  count            0..8, best first
                                       u8  reserved         0
                                       u16 num_classes
                                       u16 threshold_x1000
                                       per entry:
                                         u16 class_index
                                         u16 score_x1000    0..1000
                                         u8  name_len       0..48
                                         name, UTF-8, no terminator */

  /* Vision records. A box is u16 LE cx, cy, w, h, 0..65535 of the full frame.
   * crop is u8 crop_w, crop_h, then crop_w * crop_h u16 LE RGB565 pixels, with
   * 0, 0 for no thumbnail. */
  META_VISION_ID         = 0x60,  /* u8 bitmask: bit0 QR, bit1 barcode, bit2 blob,
                                     bit3 AprilTag. Sent at about 0.5 Hz. */
  META_QR_FRAME          = 0x61,  /* u8 count, then per result:
                                       u16 cx, cy, w, h
                                       u8  version        QR symbol version
                                       u8  ecc_level      0..3 = M, L, H, Q
                                       u16 text_len, then the payload
                                       crop */
  META_BARCODE_FRAME     = 0x62,  /* u8 count, then per result:
                                       u16 cx, cy, w, h
                                       u8  symbology      barcode1d_sym_t
                                       u16 text_len, then the digits
                                       crop */
  META_BLOB_FRAME        = 0x63,  /* u8 count, 0 or 1, then:
                                       u16 cx, cy, w, h
                                       u16 area           fraction of the analysis
                                                          window
                                       u16 hue            degrees, 0..359
                                       u16 pixels         cluster size, clamped
                                       crop */
  META_APRILTAG          = 0x66,  /* u8 count, then per tag:
                                       u16 cx, cy, w, h
                                       u16 id
                                       u8  family         0 = tag36h11
                                       u8  hamming        bit errors corrected
                                       u16 margin         0..65535
                                       i16 angle_cdeg     rotation, centi-degrees
                                       u8  pose_valid     1 = the six below are valid
                                       i16 tx, ty, tz     millimetres
                                       i16 rx, ry, rz     Euler XYZ, centi-degrees
                                       crop */

  META_CAM_CAPS          = 0x64,  /* u8 bitmask of ImageSensor::N6_CAM_CAP_*: bit0
                                     exposure, bit1 gain, bit2 white balance, bit3
                                     autofocus. Sent at about 1 Hz when non-zero. */

  META_EVENT             = 0x67,  /* N6Events event, 17 B. Equals N6_HOST_TAG_EVENT.
                                       u32 id           monotonic per boot
                                       u32 t_ms         ms since boot
                                       u8  sources      bit0 motion, bit1 NN,
                                                        bit2 audio, bit3 thermal,
                                                        bit4 ToF, bit5 GPIO,
                                                        bit6 timer, bit7 host
                                       u8  state        0 idle, 1 post-roll,
                                                        2 writing, 3 refractory
                                       u16 score        sketch-defined, 0..65535
                                       u16 pre_frames   clip frames before t
                                       u16 post_frames  clip frames after t
                                       u8  sinks_ok     bit i = sink i completed */

  /* Evidence records, 0x70 to 0x7F. A transport may drop them. */
  META_EVIDENCE_CROP     = 0x70,  /* Crops for the same frame's result TLV, only
                                     while a host is subscribed with
                                     N6_CMD_SUB0:
                                       u8  count
                                       per crop:
                                         u8  result_index  into the result TLV
                                         u8  crop_w
                                         u8  crop_h
                                         u16 LE RGB565 pixels[crop_w * crop_h] */

  META_DEBUG_LOG         = 0x71,  /* Terminal lines of the last 2 s, numbered and
                                     repeated in every frame:
                                       u32 first    number of the first line, or of
                                                    the next line when count is 0.
                                                    Lines start at 1 each boot.
                                       u8  count
                                       per line: u8 len, then ASCII text of at most
                                                 160 bytes, no terminator
                                       u8  boot     differs from the last boot's
                                                    with odds of 255 in 256.
                                                    Older firmware omits it. */

};

/* One slot per source of TLV bytes. Each slot has one writer. */
typedef enum
{
  VND_META_SLOT_MODEL_0 = 0,
  VND_META_SLOT_MODEL_1 = 1,
  VND_META_SLOT_MODEL_2 = 2,
  VND_META_SLOT_MODEL_3 = 3,
  VND_META_SLOT_TOF     = 4,
  VND_META_SLOT_IMU     = 5,
  VND_META_SLOT_MAG     = 6,
  VND_META_SLOT_THERMAL = 7,
  VND_META_SLOT_DEBUG   = 8,
  VND_META_SLOT_QR        = 9,
  VND_META_SLOT_BARCODE   = 10,
  VND_META_SLOT_BLOB      = 11,
  VND_META_SLOT_VISION_ID = 12,
  VND_META_SLOT_APRILTAG  = 13,
  VND_META_SLOT_AUDIO_CLASS  = 14,
  VND_META_SLOT_AUDIO_MEL    = 15,
  VND_META_SLOT_AUDIO_LABELS = 16,
  VND_META_SLOT_EVIDENCE     = 17,

  /* Mirrored as N6_EVENTS_META_SLOT in N6Events. */
  VND_META_SLOT_EVENT        = 18,
  VND_META_SLOT_COUNT,

  VND_META_SLOT_PRIMARY = VND_META_SLOT_MODEL_0
} vnd_meta_slot_t;

typedef struct
{
  uint8_t *buf;
  uint32_t capacity;
  uint32_t len;
  vnd_meta_slot_t slot;
} vnd_meta_builder_t;

void vnd_meta_init(uint8_t *meta_buf, uint32_t max_bytes);
/* Concatenates the ready slots in id order, without the thermal one. */
bool vnd_meta_get(const uint8_t **out_meta, uint32_t *out_len);

/* As vnd_meta_get, into the buffer chosen by idx & 1, the frame parity. */
bool vnd_meta_get_pingpong(uint32_t idx, const uint8_t **out_meta, uint32_t *out_len);
bool vnd_meta_is_ready(void);
void vnd_meta_invalidate(void);

/* Counter incremented by every vnd_meta_builder_finish. */
uint32_t vnd_meta_pub_seq(void);

/* Counter incremented when the thermal slot finishes. */
uint32_t vnd_meta_thermal_pub_seq(void);

/* One slot's published bytes, valid until the next vnd_meta_builder_finish on
 * the slot. False if the slot is empty. */
bool vnd_meta_get_slot(vnd_meta_slot_t slot, const uint8_t **ptr, uint32_t *len);

/* Publishes one ASCII line, cut at 160 characters, to Neuro Studio's Terminal,
 * from any task. A burst over about 1 KB loses its oldest lines. */
bool vnd_meta_publish_debug_text(const char *text);

/* Replaces a slot's content with one TLV. Returns false and leaves the slot
 * unchanged if the slot is out of range or the value exceeds its capacity. */
bool vnd_meta_publish_tlv(vnd_meta_slot_t slot, uint8_t type,
                          const void *value, uint16_t len);

/* Boot log: the first 2 KB of debug lines, replayed when a host attaches. */

/* Restarts the replay. Called on USB mount. */
void vnd_meta_debug_mark_replay(void);
/* Called once per n6_usb_task pass, the DEBUG slot's only writer. link_idle is
 * 1 when no vendor TX is in flight, an int to match the weak stub in NeuroN6.c. */
void vnd_meta_debug_replay_step(int link_idle);

/* Empties one slot. */
void vnd_meta_invalidate_slot(vnd_meta_slot_t slot);

/* TLV builder */

void vnd_meta_put_u16_le(uint8_t *p, uint16_t v);
void vnd_meta_put_u32_le(uint8_t *p, uint32_t v);
void vnd_meta_builder_begin(vnd_meta_builder_t *builder, vnd_meta_slot_t slot);
void vnd_meta_builder_finish(const vnd_meta_builder_t *builder);
bool vnd_meta_builder_append_bytes(vnd_meta_builder_t *builder, const uint8_t *src, uint32_t len);
bool vnd_meta_builder_put_tlv(vnd_meta_builder_t *builder, uint8_t type, const void *value, uint16_t len);
uint16_t vnd_meta_f01_to_u16(float32_t v);
uint8_t  vnd_meta_conf_to_q8(float32_t c);

/* A box corner clipped to the frame. NaN gives 0. */
static inline float32_t vnd_meta_clamp01(float32_t v)
{
  return !(v > 0.f) ? 0.f : ((v > 1.f) ? 1.f : v);
}

#ifdef __cplusplus
}
#endif

#endif /* VND_META_H */
