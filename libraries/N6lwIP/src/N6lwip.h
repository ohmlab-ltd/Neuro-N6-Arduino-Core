/**
 ******************************************************************************
 * @file    N6lwip.h
 * @brief   lwIP for the network libraries. Including it links N6lwIP.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#ifndef N6LWIP_H
#define N6LWIP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Starts lwIP's tcpip thread once, for whichever library asks first, and waits
 * up to 2 s for it. Returns 0 when it runs. */
int n6_lwip_tcpip_start(void);

#ifdef __cplusplus
}
#endif

#endif /* N6LWIP_H */
