/*
 * VD66GY_ISP_mw.c: compiles the ST ISP middleware as one translation unit.
 * extras/ is off the include path, so isp/flat/ keeps headers beside sources.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include "VD66GY_ISP_config.h"

#ifdef VD66_ISP_AUTO_ENABLED

/* Also defined in flat/isp_conf.h. */
#ifndef ISP_MW_SW_AEC_ALGO_SUPPORT
#define ISP_MW_SW_AEC_ALGO_SUPPORT
#endif
#ifndef ISP_MW_SW_AWB_ALGO_SUPPORT
#define ISP_MW_SW_AWB_ALGO_SUPPORT
#endif

#include <string.h>
#include <math.h>

/* The IQTune tool is left out: ISP_MW_TUNING_TOOL_SUPPORT stays undefined. */
#include "../extras/st-driver/isp/flat/isp_services.c"
#include "../extras/st-driver/isp/flat/isp_ae_algo.c"
#include "../extras/st-driver/isp/flat/isp_awb_algo.c"
#include "../extras/st-driver/isp/flat/isp_algo.c"
#include "../extras/st-driver/isp/flat/isp_core.c"

/* The IQ parameters stay in C: C++ rejects their nested designated initializers. */
#include "../extras/st-driver/isp/flat/vd66gy_mai1_isp_param_conf.h"
const ISP_IQParamTypeDef *vd66_isp_iqparam(void) { return &ISP_IQParamCacheInit_VD66GY; }

#endif /* VD66_ISP_AUTO_ENABLED */
