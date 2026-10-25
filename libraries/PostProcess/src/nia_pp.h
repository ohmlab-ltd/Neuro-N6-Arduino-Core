/**
 * @file    nia_pp.h
 * @brief   Post-processing for the Nia detector.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef NIA_PP_H
#define NIA_PP_H

#include <stdint.h>
#include <stdbool.h>

/* Outside extern "C" so arm_mve.h keeps its C++ overloads. */
#include "arm_math_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "ll_aton_rt_user_api.h"

#ifndef NIA_PP_CONF_THRESHOLD
#define NIA_PP_CONF_THRESHOLD   (0.25f)
#endif
#ifndef NIA_PP_IOU_THRESHOLD
#define NIA_PP_IOU_THRESHOLD    (0.60f)   /* unused: there is no NMS */
#endif
#ifndef NIA_PP_MAX_DET
#define NIA_PP_MAX_DET          (100)
#endif
#ifndef NIA_PP_NB_CLASSES
#define NIA_PP_NB_CLASSES       (80)      /* COCO-80 */
#endif
/* One candidate per cell: 32^2 + 16^2 + 8^2. */
#ifndef NIA_PP_MAX_CAND
#define NIA_PP_MAX_CAND         (1344)
#endif
#ifndef NIA_PP_MAX_GRID_CELLS
#define NIA_PP_MAX_GRID_CELLS   (32 * 32)
#endif
/* Side of the EQ-gate window, the local-maximum test used instead of NMS. */
#ifndef NIA_PP_GATE_KERNEL
#define NIA_PP_GATE_KERNEL      (3)
#endif
/* Model input side in pixels. */
#ifndef NIA_PP_INPUT_DIM
#define NIA_PP_INPUT_DIM        (256.0f)
#endif

/* One detection. Centre and size are 0..1 within the NN input. */
typedef struct {
  float32_t x_center;
  float32_t y_center;
  float32_t width;
  float32_t height;
  float32_t conf;
  int32_t   class_index;
} nia_det_t;

/* Tagged for the forward declaration in LCD_Overlay_Nia.h. */
typedef struct nia_pp_out {
  nia_det_t *pOutBuff;   /* caller's array of at least max_det entries */
  int32_t    nb_detect;  /* valid entries in pOutBuff */
} nia_pp_out_t;

typedef struct {
  float32_t conf_threshold;
  float32_t iou_threshold;   /* unused by nia_pp_process */
  int32_t   max_det;
} nia_pp_params_t;

/**
 * @brief  Decodes the NPU outputs into out.
 * @return 0, or a negative value on bad arguments.
 */
int32_t nia_pp_process(const LL_Buffer_InfoTypeDef *outs,
                       nia_pp_out_t *out,
                       const nia_pp_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* NIA_PP_H */
