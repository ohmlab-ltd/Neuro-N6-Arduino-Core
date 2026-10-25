/**
 ******************************************************************************
 * @file    EthernetN6.cpp
 * @brief   EthernetN6 sketch object and the core's Ethernet transport hooks.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#include "EthernetN6.h"
#include "eth_if.h"

extern "C" void NeuroN6_DbgPrint(const char *s);
extern "C" void NeuroN6_Notice(const char *s);   /* kept and replayed to each host */

/* Default address, packed so that 192.168.20.1 is 0xC0A81401. */
static uint32_t s_ip_be   = ((uint32_t)192u << 24) | ((uint32_t)168u << 16) | ((uint32_t)20u << 8) | (uint32_t)1u;
static uint16_t s_ip_port = 5000u;

EthernetN6_t EthernetN6;

void EthernetN6_t::setStaticIp(IPAddress ip, uint16_t port)
{
  s_ip_be   = ((uint32_t)ip[0] << 24) | ((uint32_t)ip[1] << 16) |
              ((uint32_t)ip[2] <<  8) |  (uint32_t)ip[3];
  s_ip_port = port;
}

IPAddress EthernetN6_t::localIP(void) const
{
  return IPAddress((uint8_t)((s_ip_be >> 24) & 0xFFu),
                   (uint8_t)((s_ip_be >> 16) & 0xFFu),
                   (uint8_t)((s_ip_be >>  8) & 0xFFu),
                   (uint8_t)( s_ip_be        & 0xFFu));
}

uint16_t EthernetN6_t::port(void) const { return s_ip_port; }

/* C hooks. They override the weak defaults in NeuroN6.c. */
extern "C" {

int EthernetN6_Start(int transport_mode)
{
  (void)transport_mode;
  NeuroN6_DbgPrint("[EthernetN6] bringing up MAC + LAN8742A (static IP)...\r\n");
  /* Direct cable, no router: the gateway is the board's own IP. */
  int rc = eth_netif_start(s_ip_be, 0xFFFFFF00u /* 255.255.255.0 */, s_ip_be);
  if (rc != 0) NeuroN6_Notice("[EthernetN6] bring-up FAILED (cable/PHY?), core falls back to USB.\r\n");
  else         NeuroN6_DbgPrint("[EthernetN6] netif UP. Ping the board's static IP to verify.\r\n");
  return rc;
}

bool     EthernetN6_TxBusy(void)      { return eth_video_tx_busy(); }

/* In memory as the core's announce copies it: the first octet first. */
uint32_t EthernetN6_GetIPv4_BE(void)
{
  uint32_t v;
  uint8_t *b = (uint8_t *)&v;
  b[0] = (uint8_t)(s_ip_be >> 24);
  b[1] = (uint8_t)(s_ip_be >> 16);
  b[2] = (uint8_t)(s_ip_be >>  8);
  b[3] = (uint8_t) s_ip_be;
  return v;
}

uint16_t EthernetN6_GetIPv4_Port(void){ return s_ip_port; }

} /* extern "C" */
