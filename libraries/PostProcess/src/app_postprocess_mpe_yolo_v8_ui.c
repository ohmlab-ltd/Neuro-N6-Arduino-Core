/**
 ******************************************************************************
 * @file    app_postprocess_mpe_yolo_v8_ui.c
 * @brief   YOLOv8 multi-person pose post-processing wrapper.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "app_postprocess_mpe_yolo_v8_ui.h"
#include <stddef.h>

mpe_pp_out_t                 mpe_pp_output;
mpe_yolov8_pp_static_param_t mpe_pp_params;

/* Sized to TOTAL_BOXES, not max_boxes_limit: mpe_yolov8_pp_process writes
 * every pre-NMS detection into pOutBuff. */
__attribute__((section(".ext_psram")))
static mpe_pp_outBuffer_t out_detections[AI_MPE_YOLOV8_PP_TOTAL_BOXES];

__attribute__((section(".ext_psram")))
static mpe_pp_keyPoints_t out_keyPoints[AI_MPE_YOLOV8_PP_TOTAL_BOXES
                                        * AI_POSE_PP_POSE_KEYPOINTS_NB];

int32_t app_postprocess_mpe_yolo_v8_ui_init(float conf_threshold, float iou_threshold)
{
    mpe_pp_params.nb_classes      = AI_MPE_YOLOV8_PP_NB_CLASSES;
    mpe_pp_params.nb_total_boxes  = AI_MPE_YOLOV8_PP_TOTAL_BOXES;
    mpe_pp_params.max_boxes_limit = AI_MPE_YOLOV8_PP_MAX_BOXES_LIMIT;
    mpe_pp_params.conf_threshold  = conf_threshold;
    mpe_pp_params.iou_threshold   = iou_threshold;
    mpe_pp_params.nb_keypoints    = AI_POSE_PP_POSE_KEYPOINTS_NB;

    for (int i = 0; i < AI_MPE_YOLOV8_PP_TOTAL_BOXES; i++)
    {
        out_detections[i].pKeyPoints =
            &out_keyPoints[i * AI_POSE_PP_POSE_KEYPOINTS_NB];
    }

    /* Not used by the float path. */
    mpe_pp_params.pScratchBuffer = NULL;

    return mpe_yolov8_pp_reset(&mpe_pp_params);
}

int32_t app_postprocess_mpe_yolo_v8_ui_run(NN_Instance_TypeDef *instance)
{
    const LL_Buffer_InfoTypeDef *nn_out_info =
        LL_ATON_Output_Buffers_Info(instance);

    mpe_pp_output.pOutBuff = out_detections;
    mpe_pp_output.nb_detect = 0;

    /* One output, at least what the post-processing reads: float32 box,
     * class scores and three values a keypoint, for every anchor. Another
     * network is refused, not read past. */
    if ((nn_out_info == NULL) || (nn_out_info[0].name == NULL)) {
        return AI_MPE_PP_ERROR;
    }
    {
        const uint32_t want = (uint32_t)(4 + mpe_pp_params.nb_classes +
                                         (int32_t)(3U * mpe_pp_params.nb_keypoints)) *
                              (uint32_t)mpe_pp_params.nb_total_boxes * (uint32_t)sizeof(float32_t);
        const uint8_t *b0 = LL_Buffer_addr_start(&nn_out_info[0]);

        if ((b0 == NULL) || ((uint32_t)(LL_Buffer_addr_end(&nn_out_info[0]) - b0) < want)) {
            return AI_MPE_PP_ERROR;
        }
    }

    mpe_yolov8_pp_in_centroid_t pp_input = {
        .pRaw_detections = (float32_t *)LL_Buffer_addr_start(&nn_out_info[0])
    };

    return mpe_yolov8_pp_process(&pp_input, &mpe_pp_output, &mpe_pp_params);
}
