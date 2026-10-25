/**
 ******************************************************************************
 * @file    eth_if.h
 * @brief   Internal interface between the EthernetN6 source files.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#ifndef ETHERNETN6_ETH_IF_H
#define ETHERNETN6_ETH_IF_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Starts the MAC and the LAN8742 PHY, adds the lwIP netif and returns 0 on
 * success. Addresses are in host byte order: 192.168.20.1 is 0xC0A81401. */
int eth_netif_start(uint32_t ip_be, uint32_t mask_be, uint32_t gw_be);

/* 1 when the netif is up and the PHY reports link. */
int eth_link_ready(void);

/* Called by eth_netif_start once the netif is up. */
void eth_video_start(void);

/* True while a frame is being sent. */
bool eth_video_tx_busy(void);

#ifdef __cplusplus
}
#endif

#endif /* ETHERNETN6_ETH_IF_H */
