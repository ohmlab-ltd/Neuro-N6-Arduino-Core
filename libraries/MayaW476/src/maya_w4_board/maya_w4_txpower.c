/**
 ******************************************************************************
 * @file    maya_w4_txpower.c
 * @brief   Regulatory limits: channels 1 to 11 and the TX power cap.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "maya_w4_txpower.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "wlan.h"
#include "wifi_driver.h"

extern void NeuroN6_DbgPrint(const char *msg);
extern void NeuroN6_Notice(const char *msg);     /* kept and replayed to each host */

/* The access point's own TX power, in wifi-uap.c. Action 0 reads, 1 writes. */
extern int wifi_uap_tx_power_getset(uint8_t action, uint8_t *tx_power_dbm);

/* Modulation groups in this build's TRPC entry array. */
#define MAYA_W4_NUM_MOD_GRPS \
    ((t_u8)(sizeof(((wifi_txpwrlimit_config_t *)0)->txpwrlimit_entry) / \
            sizeof(wifi_txpwrlimit_entry_t)))

/* 2.4 GHz channels 1 to 14. */
#define MAYA_W4_NUM_2G4_CHANS  14u

/* TRPC entries in a table. */
#define MAYA_W4_MAX_TRPC_CHANS \
    ((t_u8)(sizeof(((wifi_txpwrlimit_t *)0)->txpwrlimit_config) / \
            sizeof(wifi_txpwrlimit_config_t)))

static wifi_txpwrlimit_t s_txpwrlimit_2g4;
static wifi_txpwrlimit_t s_txpwrlimit_read;
static wlan_chanlist_t   s_chanlist_read;

static void maya_w4_build_2g4_table(void)
{
    const t_u8 n_grps = MAYA_W4_NUM_MOD_GRPS;

    (void)memset(&s_txpwrlimit_2g4, 0, sizeof(s_txpwrlimit_2g4));
    s_txpwrlimit_2g4.subband   = (wifi_SubBand_t)0x00;   /* 2.4 GHz */
    s_txpwrlimit_2g4.num_chans = (t_u8)MAYA_W4_NUM_2G4_CHANS;

    for (t_u8 ch = 0U; ch < (t_u8)MAYA_W4_NUM_2G4_CHANS; ch++)
    {
        wifi_txpwrlimit_config_t *cfg = &s_txpwrlimit_2g4.txpwrlimit_config[ch];

        cfg->num_mod_grps         = n_grps;
        cfg->chan_desc.start_freq = 2407U;
        cfg->chan_desc.chan_width = 20U;
        cfg->chan_desc.chan_num   = (t_u8)(ch + 1U);

        for (t_u8 g = 0U; g < n_grps; g++)
        {
            cfg->txpwrlimit_entry[g].mod_group = g;
            cfg->txpwrlimit_entry[g].tx_power  = (t_u8)MAYA_W4_TX_POWER_DBM;
        }
    }
}

void maya_w4_apply_tx_power_cap(void)
{
    char dbg[176];

    maya_w4_build_2g4_table();

    /* Per-channel TRPC limits, for station and access point. */
    int rc = wlan_set_txpwrlimit(&s_txpwrlimit_2g4);
    if (rc != WM_SUCCESS)
    {
        snprintf(dbg, sizeof(dbg),
                 "[MayaW476] the TX power limit was not applied (rc=%d); the "
                 "radio may exceed %u dBm.\r\n", rc,
                 (unsigned)MAYA_W4_TX_POWER_DBM);
        NeuroN6_Notice(dbg);
    }

    /* The station ceiling too, for a firmware that applies the two limits
     * independently. */
    rc = wlan_set_sta_tx_power((t_u32)MAYA_W4_TX_POWER_DBM);
    if (rc != WM_SUCCESS)
    {
        snprintf(dbg, sizeof(dbg),
                 "[MayaW476] the station TX power was not set (rc=%d).\r\n", rc);
        NeuroN6_Notice(dbg);
    }
}

void maya_w4_report_softap_tx_power(void)
{
    char dbg[176];
    uint8_t got = 0U;

    /* The IW610 firmware keeps its own per-channel figure here and ignores
     * writes to it; the limit table above caps what the access point sends. */
    int rc = wifi_uap_tx_power_getset(0U, &got);
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] SoftAP power setting (firmware's, read only): %u dBm, "
             "capped by the %u dBm limit table (rc=%d).\r\n",
             (unsigned)got, (unsigned)MAYA_W4_TX_POWER_DBM, rc);
    NeuroN6_DbgPrint(dbg);
}

/* The region and the channels the driver scans, joins and hosts on. */
static void report_channels(void)
{
    char dbg[176];
    unsigned int region = 0xFFU;
    bool bad;

    (void)wlan_get_region_code(&region);
    int rc = wlan_get_chanlist(&s_chanlist_read);
    bad = (rc != WM_SUCCESS) || (s_chanlist_read.num_chans == 0U) || (region != 0x00U);

    int n = snprintf(dbg, sizeof(dbg), "[MayaW476] Region 0x%02X, channels:",
                     region & 0xFFU);
    for (t_u8 i = 0U; (rc == WM_SUCCESS) && (i < s_chanlist_read.num_chans) &&
                      (n > 0) && ((size_t)n < sizeof(dbg) - 8U); i++)
    {
        const wifi_chan_info_t *c = &s_chanlist_read.chan_info[i];
        n += snprintf(dbg + n, sizeof(dbg) - (size_t)n, " %u%s", (unsigned)c->chan_num,
                      c->passive_scan_or_radar_detect ? "p" : "");
        if (c->chan_num > MAYA_W4_MAX_CHANNEL)
        {
            bad = true;
        }
    }
    if ((n > 0) && ((size_t)n < sizeof(dbg)))
    {
        (void)snprintf(dbg + n, sizeof(dbg) - (size_t)n, ".\r\n");
    }
    NeuroN6_DbgPrint(dbg);

    if (bad)
    {
        snprintf(dbg, sizeof(dbg),
                 "[MayaW476] the region or channel list is not WWSM with channels "
                 "1 to %u (rc=%d).\r\n", (unsigned)MAYA_W4_MAX_CHANNEL, rc);
        NeuroN6_Notice(dbg);
    }
}

/* The firmware's per-channel TRPC limits. */
static void report_trpc(void)
{
    char dbg[176];
    unsigned lo = 0xFFU, hi = 0U, groups = 0U;
    unsigned over_ch = 0U, over_grp = 0U, over_dbm = 0U;

    int rc = wlan_get_txpwrlimit((wifi_SubBand_t)0x00, &s_txpwrlimit_read);
    const t_u8 n_chans = (s_txpwrlimit_read.num_chans < MAYA_W4_MAX_TRPC_CHANS)
                       ? s_txpwrlimit_read.num_chans : MAYA_W4_MAX_TRPC_CHANS;

    for (t_u8 c = 0U; (rc == WM_SUCCESS) && (c < n_chans); c++)
    {
        const wifi_txpwrlimit_config_t *cfg = &s_txpwrlimit_read.txpwrlimit_config[c];
        const t_u8 n_grps = (cfg->num_mod_grps < MAYA_W4_NUM_MOD_GRPS)
                          ? cfg->num_mod_grps : MAYA_W4_NUM_MOD_GRPS;

        if (n_grps > groups)
        {
            groups = n_grps;
        }
        for (t_u8 g = 0U; g < n_grps; g++)
        {
            const unsigned p = cfg->txpwrlimit_entry[g].tx_power;
            if (p < lo)
            {
                lo = p;
            }
            if (p > hi)
            {
                hi = p;
            }
            if ((p > MAYA_W4_TX_POWER_DBM) && (over_dbm == 0U))
            {
                over_ch  = cfg->chan_desc.chan_num;
                over_grp = cfg->txpwrlimit_entry[g].mod_group;
                over_dbm = p;
            }
        }
    }

    if ((rc != WM_SUCCESS) || (n_chans == 0U) || (groups == 0U))
    {
        snprintf(dbg, sizeof(dbg),
                 "[MayaW476] the TX power limits could not be read back (rc=%d, "
                 "%u channels).\r\n", rc, (unsigned)n_chans);
        NeuroN6_Notice(dbg);
        return;
    }

    snprintf(dbg, sizeof(dbg),
             "[MayaW476] TX power limits read back: %u channels, %u groups each, "
             "%u to %u dBm.\r\n", (unsigned)n_chans, groups, lo, hi);
    NeuroN6_DbgPrint(dbg);

    if (over_dbm != 0U)
    {
        snprintf(dbg, sizeof(dbg),
                 "[MayaW476] channel %u, group %u allows %u dBm, above the %u dBm "
                 "cap.\r\n", over_ch, over_grp, over_dbm,
                 (unsigned)MAYA_W4_TX_POWER_DBM);
        NeuroN6_Notice(dbg);
    }
}

void maya_w4_report_regulatory(void)
{
    char dbg[144];
    t_u32 sta = 0U;

    report_channels();
    report_trpc();

    int rc = wlan_get_sta_tx_power(&sta);
    if ((rc != WM_SUCCESS) || (sta == 0U) || (sta > MAYA_W4_TX_POWER_DBM))
    {
        snprintf(dbg, sizeof(dbg),
                 "[MayaW476] the station TX power is not capped or not read back "
                 "(rc=%d, read %u dBm).\r\n", rc, (unsigned)sta);
        NeuroN6_Notice(dbg);
        return;
    }
    snprintf(dbg, sizeof(dbg), "[MayaW476] Station TX power read back: %u dBm.\r\n",
             (unsigned)sta);
    NeuroN6_DbgPrint(dbg);
}
