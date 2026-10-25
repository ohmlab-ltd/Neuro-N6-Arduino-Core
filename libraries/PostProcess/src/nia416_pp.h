/**
 * @file    nia416_pp.h
 * @brief   Post-processing for the Nia 416x416 detector.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef NIA416_PP_H
#define NIA416_PP_H

#include <stdint.h>
#include <stdbool.h>

/* Outside extern "C" so arm_mve.h keeps its C++ overloads. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "nia_pp.h"
#include "ll_aton_rt_user_api.h"

#ifndef NIA416_PP_CONF_THRESHOLD
#define NIA416_PP_CONF_THRESHOLD  (0.25f)
#endif
#ifndef NIA416_PP_IOU_THRESHOLD
#define NIA416_PP_IOU_THRESHOLD   (0.70f)  /* class-aware NMS */
#endif
#ifndef NIA416_PP_MAX_DET
#define NIA416_PP_MAX_DET         (100)
#endif
#ifndef NIA416_PP_NB_CLASSES
#define NIA416_PP_NB_CLASSES      (80)     /* COCO-80, contiguous order */
#endif
/* Pre-NMS candidate cap. The decode stops once it is reached. */
#ifndef NIA416_PP_MAX_CAND
#define NIA416_PP_MAX_CAND        (512)
#endif
/* Model input side in pixels. */
#ifndef NIA416_PP_INPUT_DIM
#define NIA416_PP_INPUT_DIM       (416.0f)
#endif

/**
 * @brief  Decodes the NPU outputs into out.
 * @return 0, or a negative value on bad arguments.
 */
int32_t nia416_pp_process(const LL_Buffer_InfoTypeDef *outs,
                          nia_pp_out_t *out,
                          const nia_pp_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* NIA416_PP_H */
