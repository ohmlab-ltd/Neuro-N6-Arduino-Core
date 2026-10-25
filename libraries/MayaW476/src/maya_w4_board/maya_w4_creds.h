/**
 ******************************************************************************
 * @file    maya_w4_creds.h
 * @brief   Internal bridge between the MayaW476 object and the C WiFi code.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef MAYA_W4_CREDS_H
#define MAYA_W4_CREDS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Settings the sketch made */

/* The strings live for the whole program. security is a WifiSecurity, 0..6. */
typedef struct {
  const char *ssid;
  const char *psk;
  int         security;
} maya_w4_sta_creds_t;

/* ip is the board's own address as {a,b,c,d}. */
typedef struct {
  const char *ssid;
  const char *psk;
  int         security;
  uint8_t     channel;
  bool        hidden;
  uint8_t     ip[4];
  uint8_t     netmask[4];
  uint8_t     dhcp_first[4];   /* lease pool, used when inside the subnet */
  uint8_t     dhcp_last[4];
} maya_w4_ap_creds_t;

/* Station addressing in network byte order. ip == 0 means DHCP. */
typedef struct {
  uint32_t ip;
  uint32_t gateway;
  uint32_t netmask;
  uint32_t dns1;
  uint32_t dns2;
} maya_w4_ip_cfg_t;

void maya_w4_get_sta_creds(maya_w4_sta_creds_t *out);
void maya_w4_get_ap_creds (maya_w4_ap_creds_t  *out);
void maya_w4_get_sta_ip   (maya_w4_ip_cfg_t    *out);

/* DNS servers from setDNS(). 0 means none. */
void maya_w4_get_dns(uint32_t *dns1, uint32_t *dns2);

void maya_w4_wifi_apply_dns(void);

/* Name for DHCP and mDNS: setHostname()'s, else the stored one, else
 * neuron6-xxxxxx. Never NULL, lives for the whole program. */
const char *maya_w4_get_hostname(void);

/* Copies the link key to out. Returns its length, 0 when there is none. */
size_t maya_w4_get_link_key(uint8_t *out, size_t cap);

/* TCP port the host server listens on. Default 5000. */
uint16_t maya_w4_get_tcp_port(void);

/* ipv4_be is the assigned address in network byte order, or 0. */
void maya_w4_set_net_status(bool net_ready, uint32_t ipv4_be);

/* Requests to the WiFi supervisor task */

enum {
  MAYA_W4_OP_STA = 1,     /* join the network in the STA settings */
  MAYA_W4_OP_AP,          /* host the access point in the AP settings */
  MAYA_W4_OP_DISCONNECT,
  MAYA_W4_OP_OFF,         /* disconnect, then put the WiFi radio to sleep */
  MAYA_W4_OP_SCAN,        /* returns the count */
  MAYA_W4_OP_PS           /* arg 1: IEEE power save on, 0: off */
};

/* Starts the connection manager and the supervisor. Returns 0 on success. */
int maya_w4_wifi_connect_start(void);

/* Runs one request on the supervisor task. Returns >= 0 on success, negative
 * on failure or timeout. */
int maya_w4_wifi_request(int op, int arg, uint32_t timeout_ms);

/* Link state */

enum { MAYA_W4_MODE_NONE = 0, MAYA_W4_MODE_STA = 1, MAYA_W4_MODE_AP = 2 };

/* Why the station is not connected. */
enum {
  MAYA_W4_REASON_NONE = 0,
  MAYA_W4_REASON_NOT_FOUND,
  MAYA_W4_REASON_AUTH,        /* the network refused the passphrase */
  MAYA_W4_REASON_FAILED,      /* association or DHCP failed */
  MAYA_W4_REASON_LINK_LOST,
  MAYA_W4_REASON_USER         /* disconnect() or end() */
};

int      maya_w4_wifi_mode                  (void);
int      maya_w4_wifi_last_reason           (void);
bool     maya_w4_wifi_sta_connected         (void);   /* associated, with an address */
bool     maya_w4_wifi_softap_started        (void);
bool     maya_w4_wifi_softap_failed         (void);
uint32_t maya_w4_wifi_softap_client_count   (void);
unsigned maya_w4_wifi_ps_bits               (void);   /* 1: IEEE power save, 2: deep sleep */
bool     maya_w4_wifi_link_ready            (void);   /* usable for sockets */

/* Incremented every time the link goes down. */
uint32_t maya_w4_wifi_link_generation       (void);

/* Getters for the Arduino API */

int32_t  maya_w4_wifi_rssi(void);                         /* dBm, 0 if not joined */
bool     maya_w4_wifi_bssid(uint8_t out[6]);              /* the joined AP's, or the board's */
bool     maya_w4_wifi_mac(uint8_t out[6]);                /* interface in use */
uint8_t  maya_w4_wifi_channel(void);                      /* 0 if none */
bool     maya_w4_wifi_addr(uint32_t *ip, uint32_t *gw, uint32_t *mask);
bool     maya_w4_wifi_on_link(uint32_t addr);             /* network order */
uint32_t maya_w4_wifi_dns_server(int n);                  /* network order, 0 none */
int      maya_w4_wifi_resolve(const char *host, uint32_t *ip_out);   /* 0 ok */

/* Scan results */

#define MAYA_W4_SCAN_MAX  32u

typedef struct {
  char    ssid[33];
  uint8_t bssid[6];
  int8_t  rssi;        /* dBm */
  uint8_t channel;
  uint8_t security;    /* WifiSecurity: 0 open .. 6 WPA2/WPA3 */
} maya_w4_scan_entry_t;

/* The last scan's results, strongest first. Valid until the next scan. */
unsigned maya_w4_wifi_scan_results(const maya_w4_scan_entry_t **out);

#ifdef __cplusplus
}
#endif

#endif /* MAYA_W4_CREDS_H */
