/**
 ******************************************************************************
 * @file    n6_aed.h
 * @brief   Audio Event Detection: microphone -> log-mel -> NPU -> class.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef N6_AED_H
#define N6_AED_H

#include <stdint.h>
#include <stdbool.h>

#include "ll_aton_runtime.h"
#include "n6_aed_tables.h"

#ifdef __cplusplus
extern "C" {
#endif

#define N6_AED_NUM_CLASSES  10u

/* Mel frames between inferences, 10 ms each. */
#ifndef N6_AED_INFER_HOP
#define N6_AED_INFER_HOP    24u
#endif

/* mic_sample_rate is in Hz. Returns false when the model's input is not
 * 64 x 96 int8. */
bool  n6_aed_begin(NN_Instance_TypeDef *model, uint32_t mic_sample_rate);

/* Microphone samples, any chunk size. */
void  n6_aed_push(const int16_t *pcm, uint32_t n);

/* True when enough audio has arrived for another inference. */
bool  n6_aed_ready(void);

/* Runs the network on the current window and clears the ready flag. */
void  n6_aed_run(void);

/* Results of the last run. */
const float *n6_aed_scores(void);                 /* N6_AED_NUM_CLASSES, 0..1 */
int          n6_aed_top(void);                    /* argmax, or -1 */
const char  *n6_aed_label(int cls);

/* n6_aed_top() returns -1 when no score is above t. The default is 0.5. */
void  n6_aed_set_threshold(float t);

/* Copy of the network's input: NMEL*NFRAMES bytes, frame-major, uint8. */
const uint8_t *n6_aed_input_image(void);

/* Peak |sample| between the last two runs, 0..32767. */
uint16_t n6_aed_level(void);

#ifdef __cplusplus
}
#endif

#endif /* N6_AED_H */
