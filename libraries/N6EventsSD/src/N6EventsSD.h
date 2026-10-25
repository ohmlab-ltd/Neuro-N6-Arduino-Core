/**
 ******************************************************************************
 * @file    N6EventsSD.h
 * @brief   N6Events sink that writes each clip to the SD card.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6EVENTS_SD_H
#define N6EVENTS_SD_H

#include <SdFat.h>
#include <N6Events.h>

/* sd must be begun already. root is copied, and created if missing. A second
 * call reconfigures the one instance.
 *   /events/.seq              clip counter
 *   /events/NNNNNN/NNN.jpg    frames, oldest first
 *   /events/NNNNNN/meta.bin   per frame: u32 seq, t_ms, index,
 *                             u16 meta_len, reserved, meta[]
 *   /events/NNNNNN/event.txt  id, time, sources, score, frames, result */
const n6_event_sink_t *n6_event_sink_sd(SdFat *sd, const char *root);

/* The folder of the last clip, "" before the first. */
const char *n6_event_sink_sd_last_dir(void);

/* Frames written, bytes written and write failures since boot. */
void n6_event_sink_sd_stats(uint32_t *frames, uint32_t *bytes, uint32_t *failures);

#endif /* N6EVENTS_SD_H */
