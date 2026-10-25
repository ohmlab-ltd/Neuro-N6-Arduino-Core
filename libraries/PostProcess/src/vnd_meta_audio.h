/**
 ******************************************************************************
 * @file    vnd_meta_audio.h
 * @brief   Audio classifier metadata.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef VND_META_AUDIO_H
#define VND_META_AUDIO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* scores are 0..1, at most 64 of them. A negative top means nothing reached
 * the threshold. */
bool vnd_meta_audio_result(const float *scores, uint8_t count,
                           int top, uint16_t level, float threshold);

/* label_fn(i) returns the name of class i. Called at about 1 Hz. */
bool vnd_meta_audio_labels(const char *(*label_fn)(int), uint8_t count);

/* data is the model's input buffer. layout is 0 for frame-major, 1 for
 * mel-major. */
bool vnd_meta_audio_mel(const uint8_t *data, uint8_t n_mel, uint8_t n_frames,
                        uint8_t layout);

/* Empties the audio slots. */
void vnd_meta_audio_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* VND_META_AUDIO_H */
