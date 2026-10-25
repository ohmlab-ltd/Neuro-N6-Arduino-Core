/**
 ******************************************************************************
 * @file    Speaker.h
 * @brief   Mono speaker output: SAI1 Block B I2S to a MAX98357A.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#ifndef SPEAKER_H
#define SPEAKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* C API */
int      n6_spk_begin(uint32_t want_fs);                 /* 0 on success; fs 0 = default */
bool     n6_spk_ready(void);
uint32_t n6_spk_sample_rate(void);                       /* Hz */
void     n6_spk_tone(uint32_t freq, uint32_t ms);        /* ms 0: until n6_spk_no_tone */
void     n6_spk_no_tone(void);
uint32_t n6_spk_write(const int16_t *samples, uint32_t n); /* mono PCM; returns the count */
uint32_t n6_spk_available_for_write(void);               /* free int16 slots in the ring */
bool     n6_spk_playing(void);
void     n6_spk_set_volume(uint16_t vol_q15);            /* 0..32768, Q15 */
void     n6_spk_end(void);
void     n6_spk_debug_dump(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class Speaker {
  public:
    /* Starts the I2S stream, silent until tone() or write(). sampleRate is a
     * request: sampleRate() returns the nearest rate the clock reaches. A DMA
     * error stops the stream until begin() runs again. */
    bool     begin(uint32_t sampleRate = 48000) { return n6_spk_begin(sampleRate) == 0; }

    /* Hz, 48828 by default. */
    uint32_t sampleRate() { return n6_spk_sample_rate(); }

    /* Sine tone until noTone(). */
    void     tone(uint32_t frequencyHz) { n6_spk_tone(frequencyHz, 0); }
    void     tone(uint32_t frequencyHz, uint32_t durationMs) { n6_spk_tone(frequencyHz, durationMs); }
    void     noTone() { n6_spk_no_tone(); }

    /* Queues 16-bit mono PCM and returns the count accepted, less than n when
     * the ring is full. A tone takes priority over the ring. */
    uint32_t write(const int16_t *samples, uint32_t n) { return n6_spk_write(samples, n); }
    /* Free space in the ring, in samples. */
    uint32_t availableForWrite() { return n6_spk_available_for_write(); }

    /* True while a tone is active or the ring holds samples. */
    bool     playing() { return n6_spk_playing(); }

    /* Digital volume, 0.0 to 1.0. */
    void     setVolume(float gain01) {
      if (gain01 < 0.0f) gain01 = 0.0f;
      if (gain01 > 1.0f) gain01 = 1.0f;
      n6_spk_set_volume((uint16_t)(gain01 * 32768.0f));
    }

    void     end() { n6_spk_end(); }

    /* Prints [SPKDBG] lines to the CDC console. */
    void     debugDump() { n6_spk_debug_dump(); }
};

#endif /* __cplusplus */

#endif /* SPEAKER_H */
