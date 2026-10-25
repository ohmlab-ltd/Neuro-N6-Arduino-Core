/**
  ******************************************************************************
  * @file    wifi_config.h
  * @brief   wifi_nxp configuration overrides.
  *
  * Copyright (c) 2026 Ohm Lab Ltd.
  * SPDX-License-Identifier: MIT
  ******************************************************************************
  */

#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include "wifi_bt_module_config.h"

/* 2.4 GHz only, for regulatory compliance. It must match the build's
 * -DCONFIG_5GHz_SUPPORT=0: the flag changes shared struct layouts. */
#ifndef CONFIG_5GHz_SUPPORT
#define CONFIG_5GHz_SUPPORT 0
#endif

#if (defined(CONFIG_RF_TEST_MODE) && CONFIG_RF_TEST_MODE) || \
    (defined(CONFIG_WIFI_RF_TEST_MODE) && CONFIG_WIFI_RF_TEST_MODE) || \
    (defined(CONFIG_NXP_WIFI_RF_TEST_MODE) && CONFIG_NXP_WIFI_RF_TEST_MODE)
#error "RF test mode is not part of this core."
#endif

/* IW610 feature flags, as in the MCUXpresso SDK wifi_cli example. The scan
 * path depends on them. */
#if defined(IW610)
#define CONFIG_WMM                     1
#define CONFIG_SDIO_MULTI_PORT_TX_AGGR 1
#define CONFIG_WIFI_FEATURES           1
#define CONFIG_OFFLOAD                 1

/* At 1 the driver enables deep sleep and IEEE PS during chip init. */
#define CONFIG_WIFI_AUTO_POWER_SAVE    0
#define CONFIG_WIFI_CAPA               1

/* Unused features are off for code size. */
#define CONFIG_ROAMING                 0   /* multi-AP roaming */
#define CONFIG_TURBO_MODE              0   /* NXP proprietary turbo bit */
#define CONFIG_CLOUD_KEEP_ALIVE        0   /* cloud persistent-connection helper */
#define CONFIG_EXT_SCAN_SUPPORT        1   /* required by the IW610 scan path */
#define CONFIG_WIFI_EU_CRYPTO          0   /* SAE-H2E extensions */
#define CONFIG_11R                     0   /* Fast BSS Transition */
#define CONFIG_DRIVER_OWE              0   /* Opportunistic Wireless Encryption */
#define CONFIG_11K                     0   /* Radio Resource Management */
#define CONFIG_11V                     0   /* BSS Transition Management */

/* 802.11ac is 5 GHz only. With it off the driver sets CONFIG_11AX to 0 too. */
#define CONFIG_11AC                    0
#define CONFIG_WIFI_IND_RESET          1   /* in-band chip recovery */

/* CONFIG_AMSDU_IN_AMPDU and CONFIG_TX_RX_ZERO_COPY stay off: this port does
 * not support them. */

/* SDIO multi-port aggregation depth. The driver clamps it to 8 when WMM and
 * multi-port transmit are both on. */
#undef  CONFIG_SDIO_MP_AGGR_DEF_PKT_LIMIT
#define CONFIG_SDIO_MP_AGGR_DEF_PKT_LIMIT 32

/* WMM U-APSD defaults on with CONFIG_WMM. */
#define CONFIG_WMM_UAPSD 0

/* SoftAP support. It must also be defined on the compile command line: the
 * uAP sources test it before they include this header. */
#ifndef CONFIG_NXP_WIFI_SOFTAP_SUPPORT
#define CONFIG_NXP_WIFI_SOFTAP_SUPPORT 1
#endif
#endif

/* Network stack. */

/* IPv6 off. Enabling it also needs -DCONFIG_IPV6=1 on the command line: lwIP
 * derives LWIP_IPV6 from it. */
#define CONFIG_IPV6              0
#define CONFIG_MAX_IPV6_ADDRESSES 0

/* Debug verbosity. */

#define CONFIG_WLCMGR_DEBUG       0
#define CONFIG_WIFI_EXTRA_DEBUG   0
#define CONFIG_WIFI_EVENTS_DEBUG  0
#define CONFIG_WIFI_CMD_RESP_DEBUG 0
#define CONFIG_WIFI_PKT_DEBUG     0
#define CONFIG_WIFI_SCAN_DEBUG    0
#define CONFIG_WIFI_IO_INFO_DUMP  0
#define CONFIG_WIFI_IO_DEBUG      0
#define CONFIG_WIFI_IO_DUMP       0
#define CONFIG_WIFI_MEM_DEBUG     0
#define CONFIG_WIFI_AMPDU_DEBUG   0
#define CONFIG_WIFI_TIMER_DEBUG   0
#define CONFIG_WIFI_SDIO_DEBUG    0
#define CONFIG_WIFI_FW_DEBUG      0
#define CONFIG_WIFI_UAP_DEBUG     0
#define CONFIG_WPS_DEBUG          0
#define CONFIG_FW_VDLL_DEBUG      0
#define CONFIG_DHCP_SERVER_DEBUG  0
#define CONFIG_FWDNLD_IO_DEBUG    0

/* Heap diagnostics. */

#define CONFIG_HEAP_DEBUG         0
#define CONFIG_HEAP_STAT          0

#endif /* WIFI_CONFIG_H */
