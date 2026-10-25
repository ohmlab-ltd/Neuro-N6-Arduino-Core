/**
 ******************************************************************************
 * @file    Microphone.cpp
 * @brief   C++ wrapper for the C audio API.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#include "Microphone.h"

bool     Microphone::begin()                               { return n6_audio_begin() == 0; }
uint32_t Microphone::sampleRate()                          { return n6_audio_sample_rate(); }
uint32_t Microphone::available()                           { return n6_audio_available(); }
uint32_t Microphone::read(int16_t *dst, uint32_t maxSamples) { return n6_audio_read(dst, maxSamples); }
uint16_t Microphone::peak()                                { uint32_t p = n6_audio_peak(); return (uint16_t)(p > 32767u ? 32767u : p); }
void     Microphone::streamToNS(bool on)                   { n6_audio_stream_enable(on); }
bool     Microphone::streaming()                           { return n6_audio_streaming(); }
