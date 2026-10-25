/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdint.h>
#include "wifi_bt_module_config.h"

#if !CONFIG_UART_WIFI_BRIDGE

#if defined(IW610)

#if !(CONFIG_WIFI_IND_DNLD) && !defined(CONFIG_BT_IND_DNLD)

/* Ohm Lab: the IW610 firmware is read from memory-mapped flash, not compiled in.
 * The build stops if the length or CRC32 differs from sduartspi_iw610.bin.se. */
#define IW610_FW_BLOB_ADDR  (0x91E00000u)
#define IW610_FW_BLOB_LEN   (995992u)      /* bytes */
#define IW610_FW_BLOB_CRC   (0x8BEF4C0Cu)  /* CRC32, as zlib computes it */

const unsigned char *wlan_fw_bin   = (const unsigned char *)IW610_FW_BLOB_ADDR;
const unsigned int wlan_fw_bin_len = IW610_FW_BLOB_LEN;
const unsigned int wlan_fw_bin_crc = IW610_FW_BLOB_CRC;

const unsigned char bt_fw_bin[]  = {0x00};
const unsigned int bt_fw_bin_len = 0;

#endif

#endif

#endif
