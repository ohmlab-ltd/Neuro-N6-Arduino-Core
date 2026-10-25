/**
 ******************************************************************************
 * @file    EthernetN6.h
 * @brief   Wired Ethernet video transport: JPEG over TCP, static IP, no DHCP.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#ifndef ETHERNETN6_H
#define ETHERNETN6_H

/* Makes the Arduino build compile and link the shared lwIP library. */
#include <N6lwip.h>

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus

#include "IPAddress.h"

class EthernetN6_t
{
public:
  EthernetN6_t() = default;

  /* Takes effect only before Vision.begin(). The default is 192.168.20.1:5000. */
  void      setStaticIp(IPAddress ip, uint16_t port = 5000);

  IPAddress localIP(void) const;

  uint16_t  port(void) const;
};

extern EthernetN6_t EthernetN6;

#endif /* __cplusplus */

/* Hooks called by the core. They override the weak defaults in NeuroN6.c. */
#ifdef __cplusplus
extern "C" {
#endif

/* Starts the MAC, the PHY, the netif and the TCP sender. Returns 0, or
 * non-zero on failure, when the core falls back to USB. */
int      EthernetN6_Start(int transport_mode);

/* True while a frame is being sent. */
bool     EthernetN6_TxBusy(void);

/* The static IPv4, stored so that a memcpy puts the first octet first, and the
 * TCP port. */
uint32_t EthernetN6_GetIPv4_BE(void);
uint16_t EthernetN6_GetIPv4_Port(void);

#ifdef __cplusplus
}
#endif

#endif /* ETHERNETN6_H */
