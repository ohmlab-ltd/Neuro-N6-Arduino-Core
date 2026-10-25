/*
 * VD66GY_ISP.cpp: automatic white balance through the ST ISP middleware.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include "VD66GY_ISP_config.h"

#ifdef VD66_ISP_AUTO_ENABLED

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

extern "C" {
#include "../extras/st-driver/isp/flat/isp_api.h"
#include "../extras/st-driver/sensors/vd6g/vd6g.h"

/* Defined by the core. */
void *n6_dcmipp_handle(void);
extern volatile uint8_t g_isp_tuning_mode;   /* 2: VD66_isp_tuning_auto */

/* Defined in VD66GY_ISP_mw.c. */
const ISP_IQParamTypeDef *vd66_isp_iqparam(void);
}

static ISP_HandleTypeDef s_hIsp;
static volatile uint8_t  s_isp_started = 0u;
/* The sensor's Bayer order. A mirror or flip changes it. */
static ISP_DemosTypeTypeDef s_demos_type = ISP_DEMOS_TYPE_GRBG;
/* The IQ parameters with that order. ISP_Init copies them. */
static ISP_IQParamTypeDef s_iqparam;

/* ISP_Init requires every helper. With AEC off only GetSensorInfo is called. */
static ISP_StatusTypeDef App_GetSensorInfo(uint32_t Instance, ISP_SensorInfoTypeDef *Info)
{
  (void)Instance;
  if (Info == 0) { return ISP_OK; }
  memset(Info, 0, sizeof(*Info));
  strncpy(Info->name, "VD66GY", ISP_SENSOR_INFO_MAX_LENGTH - 1);
  Info->bayer_pattern = s_demos_type;
  Info->color_depth   = 8u;                  /* RAW8 */
  Info->width         = 1120u;               /* nominal */
  Info->height        = 1364u;
  Info->gain_min      = 1000u;               /* mdB */
  Info->gain_max      = 16000u;
  Info->again_max     = 16000u;
  Info->exposure_min  = 10u;                 /* microseconds */
  Info->exposure_max  = 33000u;
  return ISP_OK;
}
static ISP_StatusTypeDef App_SetSensorGain(uint32_t i, int32_t g)      { (void)i; (void)g; return ISP_OK; }
static ISP_StatusTypeDef App_GetSensorGain(uint32_t i, int32_t *g)     { (void)i; if (g) { *g = 1000; }  return ISP_OK; }
static ISP_StatusTypeDef App_SetSensorExposure(uint32_t i, int32_t e)  { (void)i; (void)e; return ISP_OK; }
static ISP_StatusTypeDef App_GetSensorExposure(uint32_t i, int32_t *e) { (void)i; if (e) { *e = 10000; } return ISP_OK; }

/* Interrupt context. Overrides the HAL's weak callback. */
extern "C" void HAL_DCMIPP_PIPE_VsyncEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe)
{
  (void)hdcmipp;
  if (s_isp_started && (Pipe == DCMIPP_PIPE1)) {
    ISP_IncMainFrameId(&s_hIsp);
    ISP_GatherStatistics(&s_hIsp);
  }
}

/* AWB task. */
static void vd66_isp_task(void *arg)
{
  (void)arg;
  for (;;) {
    ISP_BackgroundProcess(&s_hIsp);
    vTaskDelay(pdMS_TO_TICKS(20));   /* about once per frame */
  }
}

/* Called from VD66GY::bringUp() once the sensor is streaming. vd6g_bayer is a
 * VD6G_BayerType_t. */
extern "C" void vd66_isp_auto_start(int vd6g_bayer)
{
  if (s_isp_started) { return; }
  if (g_isp_tuning_mode != 2u) { return; }   /* VD66_isp_tuning_auto only */

  switch (vd6g_bayer) {
    case VD6G_BAYER_RGGB: s_demos_type = ISP_DEMOS_TYPE_RGGB; break;
    case VD6G_BAYER_GRBG: s_demos_type = ISP_DEMOS_TYPE_GRBG; break;
    case VD6G_BAYER_GBRG: s_demos_type = ISP_DEMOS_TYPE_GBRG; break;
    case VD6G_BAYER_BGGR: s_demos_type = ISP_DEMOS_TYPE_BGGR; break;
    default:              s_demos_type = ISP_DEMOS_TYPE_MONO; break;
  }
  s_iqparam = *vd66_isp_iqparam();
  s_iqparam.demosaicing.type = s_demos_type;

  ISP_AppliHelpersTypeDef helpers;
  memset(&helpers, 0, sizeof(helpers));
  helpers.GetSensorInfo     = App_GetSensorInfo;
  helpers.SetSensorGain     = App_SetSensorGain;
  helpers.GetSensorGain     = App_GetSensorGain;
  helpers.SetSensorExposure = App_SetSensorExposure;
  helpers.GetSensorExposure = App_GetSensorExposure;

  if (ISP_Init(&s_hIsp, n6_dcmipp_handle(), 0u, &helpers, &s_iqparam) != ISP_OK) {
    return;
  }
  (void)ISP_Start(&s_hIsp);
  (void)ISP_SetAECState(&s_hIsp, 0u);      /* exposure stays with the sensor's AE */
  (void)ISP_SetWBRefMode(&s_hIsp, 1u, 0u); /* automatic white balance */
  s_isp_started = 1u;

  (void)xTaskCreate(vd66_isp_task, "vd66_isp", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);
}

#else

extern "C" void vd66_isp_auto_start(int vd6g_bayer) { (void)vd6g_bayer; }

#endif /* VD66_ISP_AUTO_ENABLED */
