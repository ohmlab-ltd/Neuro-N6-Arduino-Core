/**
 ******************************************************************************
 * @file    Microphone.h
 * @brief   On-board PDM microphone on the MDF: 16-bit mono PCM.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#ifndef MICROPHONE_H
#define MICROPHONE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* C API */
int      n6_audio_begin(void);                              /* 0 on success */
bool     n6_audio_ready(void);
uint32_t n6_audio_sample_rate(void);                        /* Hz */
uint32_t n6_audio_available(void);                          /* int16 samples buffered */
uint32_t n6_audio_read(int16_t *dst, uint32_t max_samples); /* pops, returns the count */
void     n6_audio_stream_enable(bool on);
bool     n6_audio_streaming(void);
uint32_t n6_audio_peak(void);                               /* largest |sample| since the last call */

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Microphone {
  public:
    /* Starts capture and streaming to Neuro Studio. */
    bool     begin();
    /* Hz, 32552. */
    uint32_t sampleRate();
    /* int16 samples buffered. The ring holds about 0.5 s. */
    uint32_t available();
    /* Pops up to maxSamples into dst and returns the count. Needs
     * streamToNS(false): while streaming, the core consumes the ring. */
    uint32_t read(int16_t *dst, uint32_t maxSamples);
    /* Largest |sample| since the last call, 0 to 32767. Pops nothing. */
    uint16_t peak();
    /* Streaming to Neuro Studio, on after begin(). */
    void     streamToNS(bool on);
    bool     streaming();
};

#endif /* __cplusplus */

#endif /* MICROPHONE_H */
