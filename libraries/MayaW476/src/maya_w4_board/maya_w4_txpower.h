/**
 ******************************************************************************
 * @file    maya_w4_txpower.h
 * @brief   Regulatory limits for the MAYA-W476: 2.4 GHz channels 1 to 11 and
 *          an 8 dBm TX power cap.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef MAYA_W4_TXPOWER_H
#define MAYA_W4_TXPOWER_H

#ifdef __cplusplus
extern "C" {
#endif

/** Conducted TX power cap in dBm. */
#define MAYA_W4_TX_POWER_DBM   8u

/** Highest 2.4 GHz channel the radio may use. */
#define MAYA_W4_MAX_CHANNEL    11u

/** Applies the cap to every 2.4 GHz channel and modulation group. The driver
 *  calls it after each region power table it loads, and the supervisor after
 *  wlan_init and wlan_start. */
void maya_w4_apply_tx_power_cap(void);

/** Prints the access point's own power setting, which the firmware keeps
 *  per channel and does not let the host change. Called once it is up. */
void maya_w4_report_softap_tx_power(void);

/** Prints the region, the channel list and the TX power limits read back
 *  from the firmware. */
void maya_w4_report_regulatory(void);

#ifdef __cplusplus
}
#endif

#endif /* MAYA_W4_TXPOWER_H */
