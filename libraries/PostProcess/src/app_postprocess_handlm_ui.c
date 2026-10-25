/**
 * @file    app_postprocess_handlm_ui.c
 * @brief   MediaPipe hand-landmark post-processing.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "app_postprocess_handlm_ui.h"
#include <stddef.h>
#include <string.h>
#include <math.h>

handlm_result_t handlm_output;
float32_t       handlm_min_score = AI_HANDLM_MIN_SCORE;

int32_t app_postprocess_handlm_ui_init(float min_score)
{
  handlm_min_score = min_score;
  memset(&handlm_output, 0, sizeof(handlm_output));
  return 0;
}

int32_t app_postprocess_handlm_ui_run(NN_Instance_TypeDef *instance)
{
  const LL_Buffer_InfoTypeDef *out = LL_ATON_Output_Buffers_Info(instance);

  handlm_output.valid = false;
  if (out == NULL) return -1;

  /* Two [1,63] outputs: screen landmarks in input pixels and world landmarks
   * in metres, told apart by magnitude. Two [1,1] outputs: handedness, then
   * the presence score, unless AI_HANDLM_SCORE_IS_SECOND is 0. */
#ifndef AI_HANDLM_SCORE_IS_SECOND
#define AI_HANDLM_SCORE_IS_SECOND  (1)
#endif
  const float *lm63[2] = { NULL, NULL };
  int          n63     = 0;
  const float *one[2]  = { NULL, NULL };
  int          n1      = 0;

  for (int i = 0; out[i].name != NULL; i++) {
    const LL_Buffer_InfoTypeDef *b = &out[i];
    const uint8_t *s = LL_Buffer_addr_start(b);
    const uint8_t *e = LL_Buffer_addr_end(b);
    if (s == NULL || e == NULL) continue;

    const uint32_t nfloats = (uint32_t)(e - s) / sizeof(float);
    if (nfloats >= (uint32_t)(AI_HANDLM_NB_LANDMARKS * 3) && n63 < 2) {
      lm63[n63++] = (const float *)s;
    } else if (nfloats >= 1u && nfloats < 4u && n1 < 2) {
      one[n1++] = (const float *)s;
    }
  }

  if (n63 == 0) return -1;

  const float *screen = (n63 == 2) ? lm63[1] : lm63[0];
  if (n63 == 2) {
    float mag0 = 0.0f, mag1 = 0.0f;
    for (int i = 0; i < AI_HANDLM_NB_LANDMARKS * 3; i++) {
      const float a = fabsf(lm63[0][i]); if (a > mag0) mag0 = a;
      const float b = fabsf(lm63[1][i]); if (b > mag1) mag1 = b;
    }
    screen = (mag0 >= mag1) ? lm63[0] : lm63[1];
  }

  if (n1 >= 2 && one[0] && one[1]) {
#if AI_HANDLM_SCORE_IS_SECOND
    handlm_output.score      = one[1][0];
    handlm_output.handedness = one[0][0];
#else
    handlm_output.score      = one[0][0];
    handlm_output.handedness = one[1][0];
#endif
  } else if (n1 >= 1 && one[0]) {
    handlm_output.score      = one[0][0];
    handlm_output.handedness = 0.0f;
  } else {
    handlm_output.score      = 1.0f;
    handlm_output.handedness = 0.0f;
  }

  /* A score outside 0..1 is taken as a logit. */
  if (handlm_output.score < 0.0f || handlm_output.score > 1.0f) {
    handlm_output.score = 1.0f / (1.0f + expf(-handlm_output.score));
  }

  const float inv = 1.0f / AI_HANDLM_INPUT_SIZE;
  for (int k = 0; k < AI_HANDLM_NB_LANDMARKS; k++) {
    handlm_output.kps[k].x = screen[k * 3 + 0] * inv;   /* pixels to 0..1 */
    handlm_output.kps[k].y = screen[k * 3 + 1] * inv;
    handlm_output.kps[k].z = screen[k * 3 + 2] * inv;
  }

  handlm_output.valid = (handlm_output.score >= handlm_min_score);
  return 0;
}
