/*
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef POSTPROCESS_H
#define POSTPROCESS_H

/* Not wrapped in extern "C": arm_mve.h's C++ overloads conflict with it. */

#include "app_postprocess_iseg_yolo_v8_ui.h"
#include "app_postprocess_mpe_yolo_v8_ui.h"
#include "app_postprocess_nia_ui.h"
#include "app_postprocess_nia416_ui.h"
#include "app_postprocess_pd_ui.h"
#include "app_postprocess_handlm_ui.h"

#include "vnd_meta.h"
#include "vnd_meta_yolov8_Iseg.h"
#include "vnd_meta_yolov8_MPE.h"
#include "vnd_meta_nia.h"
#include "vnd_meta_pd.h"
#include "vnd_meta_handlm.h"
#include "vnd_meta_vision.h"

#include "LCD_Overlay_Iseg.h"
#include "LCD_Overlay_MPE.h"
#include "LCD_Overlay_PD.h"

#include "PostProcessor_YOLOv8.h"
#include "PostProcessor_NIA.h"
#include "PostProcessor_NIA416.h"
#include "PostProcessor_PD.h"
#include "PostProcessor_HandLM.h"
#include "PostProcessor_Zoo.h"

#endif /* POSTPROCESS_H */
