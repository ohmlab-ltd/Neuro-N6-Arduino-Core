/**
 ******************************************************************************
 * @file    maya_w4_connect.c
 * @brief   WiFi supervisor: one task owns every change to the link.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "maya_w4_creds.h"
#include "maya_w4_txpower.h"

#include <string.h>
#include <stdio.h>

#include "wlan.h"
#include "dhcp-server.h"
#include "wm_net.h"
#include "lwip/tcpip.h"
#include "lwip/dns.h"
#include "lwip/api.h"
#include "lwip/netifapi.h"

extern void NeuroN6_DbgPrint(const char *msg);
extern void NeuroN6_Notice(const char *msg);     /* kept and replayed to each host */

#define MAYA_DELAY_MS(ms)   vTaskDelay(pdMS_TO_TICKS(ms))

/* wifi_nxp profile names. The SSID is separate. */
static const char k_sta_profile_name[] = "n6_sta";
static const char k_uap_profile_name[] = "n6_uap";

/* Station rejoin back-off. It doubles up to the maximum. */
#define STA_BACKOFF_FIRST_MS   2000u
#define STA_BACKOFF_MAX_MS     15000u
/* A join that makes no progress for this long is restarted. */
#define STA_STUCK_MS           60000u
/* A SoftAP client is disconnected if its WPA2 handshake takes longer. */
#define SOFTAP_AUTH_MS         3000u
#define SOFTAP_AUTH_SLOTS      8u

/* Link state */
static volatile int      s_mode                = MAYA_W4_MODE_NONE;
static volatile bool     s_sta_wanted          = false;  /* keep the station joined */
static volatile bool     s_connected           = false;  /* STA: WLAN_REASON_SUCCESS */
static volatile bool     s_softap_started      = false;  /* SoftAP: WLAN_REASON_UAP_SUCCESS */
static volatile bool     s_softap_failed       = false;
static volatile bool     s_dhcp_server_started = false;
static volatile bool     s_radio_off           = false;  /* end(): deep sleep */
static volatile unsigned s_ps_bits             = 0u;     /* 1 IEEE PS, 2 deep sleep */
static volatile int      s_last_reason         = MAYA_W4_REASON_NONE;
static volatile uint32_t s_initialized_count   = 0U;
static volatile uint32_t s_client_count        = 0U;
static volatile uint32_t s_link_generation     = 0U;
/* Set while the supervisor ends a join itself: the USER_DISCONNECT that
 * follows is not the sketch's disconnect(). */
static volatile bool     s_internal_disc       = false;
/* SoftAP: WPA in use, and the clients that have not completed its handshake. */
static volatile bool     s_softap_wpa          = false;
typedef struct {
  uint8_t  mac[6];
  uint32_t since_ms;
  bool     used;
} softap_joining_t;
static softap_joining_t  s_joining_clients[SOFTAP_AUTH_SLOTS];

/* Supervisor requests */
static TaskHandle_t      s_task       = NULL;
static SemaphoreHandle_t s_api_lock   = NULL;   /* one requester at a time */
static SemaphoreHandle_t s_req_done   = NULL;
static volatile int      s_req_op     = 0;
static volatile int      s_req_arg    = 0;
static volatile int      s_req_result = 0;
static volatile uint32_t s_req_seq    = 0U;     /* last request posted */
static volatile uint32_t s_done_seq   = 0U;     /* last request finished */

/* Station keeper */
static uint32_t s_next_attempt_ms = 0U;
static uint32_t s_backoff_ms      = 0U;
static uint32_t s_busy_since_ms   = 0U;

/* Scan */
static SemaphoreHandle_t    s_scan_done = NULL;
static volatile unsigned    s_scan_count = 0U;
static maya_w4_scan_entry_t s_scan[MAYA_W4_SCAN_MAX];
static volatile unsigned    s_scan_n = 0U;

static uint32_t now_ms(void)
{
  return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* Accessors */

bool maya_w4_wifi_link_ready(void)
{
  return s_connected || (s_softap_started && s_dhcp_server_started);
}

int      maya_w4_wifi_mode               (void) { return s_mode; }
int      maya_w4_wifi_last_reason        (void) { return s_last_reason; }
bool     maya_w4_wifi_sta_connected      (void) { return s_connected; }
bool     maya_w4_wifi_softap_started     (void) { return s_softap_started; }
bool     maya_w4_wifi_softap_failed      (void) { return s_softap_failed; }
uint32_t maya_w4_wifi_softap_client_count(void) { return s_client_count; }
unsigned maya_w4_wifi_ps_bits            (void) { return s_ps_bits; }
uint32_t maya_w4_wifi_link_generation    (void) { return s_link_generation; }

unsigned maya_w4_wifi_scan_results(const maya_w4_scan_entry_t **out)
{
  if (out) *out = s_scan;
  return s_scan_n;
}

/* WifiSecurity, 0..6, to wlan_security_type. */
static enum wlan_security_type maya_sec_to_wlan(int s)
{
  switch (s) {
    case 0:  return WLAN_SECURITY_NONE;             /* WIFI_SECURITY_NONE */
    case 1:  return WLAN_SECURITY_WEP_OPEN;         /* WIFI_SECURITY_WEP */
    case 2:  return WLAN_SECURITY_WPA;              /* WIFI_SECURITY_WPA */
    case 3:  return WLAN_SECURITY_WPA2;             /* WIFI_SECURITY_WPA2 */
    case 4:  return WLAN_SECURITY_WPA_WPA2_MIXED;   /* WIFI_SECURITY_WPA_WPA2 */
    case 5:  return WLAN_SECURITY_WPA3_SAE;         /* WIFI_SECURITY_WPA3_SAE */
    case 6:  return WLAN_SECURITY_WPA2_WPA3_SAE_MIXED; /* WIFI_SECURITY_WPA2_WPA3 */
    default: return WLAN_SECURITY_WPA2;
  }
}

/* {a,b,c,d} to a uint32_t in network byte order. */
static uint32_t ip4_to_netorder(const uint8_t ip[4])
{
  return (uint32_t)ip[0]
       | ((uint32_t)ip[1] << 8)
       | ((uint32_t)ip[2] << 16)
       | ((uint32_t)ip[3] << 24);
}

/* {a,b,c,d} to a uint32_t in host order. */
static uint32_t ip4_to_host(const uint8_t ip[4])
{
  return ((uint32_t)ip[0] << 24)
       | ((uint32_t)ip[1] << 16)
       | ((uint32_t)ip[2] << 8)
       |  (uint32_t)ip[3];
}

static void print_ip(const char *what, uint32_t ip_be)
{
  char dbg[144];
  snprintf(dbg, sizeof(dbg), "%s%u.%u.%u.%u.\r\n",
           what,
           (unsigned)( ip_be        & 0xFFu), (unsigned)((ip_be >>  8) & 0xFFu),
           (unsigned)((ip_be >> 16) & 0xFFu), (unsigned)((ip_be >> 24) & 0xFFu));
  NeuroN6_DbgPrint(dbg);
}

/* Events. They run in wlcmgr_task context, where a wlan_* mutator deadlocks
 * on scan_lock. */
static void sta_failed(int why)
{
  /* A joined station that fails reports LINK_LOST, whatever the driver's
   * reason, and keeps it while rejoining. */
  const bool failure = (why == MAYA_W4_REASON_NOT_FOUND || why == MAYA_W4_REASON_AUTH ||
                        why == MAYA_W4_REASON_FAILED    || why == MAYA_W4_REASON_LINK_LOST);
  if (s_connected) {
    s_connected = false;
    s_link_generation++;
    if (failure) {
      why = MAYA_W4_REASON_LINK_LOST;
      NeuroN6_DbgPrint("[MayaW476] STA link lost - rejoining.\r\n");
    }
  } else if (failure && s_last_reason == MAYA_W4_REASON_LINK_LOST) {
    why = MAYA_W4_REASON_LINK_LOST;
  }
  if (why != MAYA_W4_REASON_NONE) s_last_reason = why;
  maya_w4_set_net_status(false, 0U);
}

static unsigned ps_bit(void *data)
{
  switch ((int)(intptr_t)data) {
    case WLAN_IEEE:            return 1u;
    case WLAN_DEEP_SLEEP:      return 2u;
    case WLAN_IEEE_DEEP_SLEEP: return 3u;
    default:                   return 0u;
  }
}

/* Marks a SoftAP client as waiting for its handshake or not. The event
 * callback and the supervisor both use the table. */
static void softap_client_waiting(const uint8_t *mac, bool waiting)
{
  if (mac == NULL) return;
  const uint32_t now = now_ms();
  taskENTER_CRITICAL();
  uint32_t slot = SOFTAP_AUTH_SLOTS;
  for (uint32_t i = 0U; i < SOFTAP_AUTH_SLOTS; ++i) {
    if (s_joining_clients[i].used && memcmp(s_joining_clients[i].mac, mac, 6U) == 0) {
      slot = i;
      break;
    }
    if (!s_joining_clients[i].used && slot == SOFTAP_AUTH_SLOTS) slot = i;
  }
  if (slot < SOFTAP_AUTH_SLOTS) {
    if (waiting) {
      (void)memcpy(s_joining_clients[slot].mac, mac, 6U);
      s_joining_clients[slot].since_ms = now;
    }
    s_joining_clients[slot].used = waiting;
  }
  taskEXIT_CRITICAL();
}

static int wlan_event_callback(enum wlan_event_reason reason, void *data)
{
  switch (reason)
  {
    case WLAN_REASON_INITIALIZED:
      s_initialized_count++;
      break;

    case WLAN_REASON_AUTH_SUCCESS:
      break;  /* success is reported by WLAN_REASON_SUCCESS */

    case WLAN_REASON_SUCCESS: {
      struct wlan_ip_config addr;
      (void)memset(&addr, 0, sizeof(addr));
      uint32_t ip_be = 0U;
      if (wlan_get_address(&addr) == WM_SUCCESS) {
        ip_be = addr.ipv4.address;
      }
      s_connected   = true;
      s_last_reason = MAYA_W4_REASON_NONE;
      maya_w4_set_net_status(true, ip_be);
      print_ip("[MayaW476] STA associated + DHCP lease acquired - IP ", ip_be);
      break;
    }

    case WLAN_REASON_NETWORK_NOT_FOUND:
      sta_failed(MAYA_W4_REASON_NOT_FOUND);
      break;

    case WLAN_REASON_NETWORK_AUTH_FAILED:
      sta_failed(MAYA_W4_REASON_AUTH);
      break;

    case WLAN_REASON_INITIALIZATION_FAILED:
    case WLAN_REASON_CONNECT_FAILED:
    case WLAN_REASON_ADDRESS_FAILED:
      sta_failed(MAYA_W4_REASON_FAILED);
      break;

    case WLAN_REASON_LINK_LOST:
      sta_failed(MAYA_W4_REASON_LINK_LOST);
      break;

    case WLAN_REASON_USER_DISCONNECT:
      if (s_internal_disc) {
        s_internal_disc = false;
        sta_failed(MAYA_W4_REASON_NONE);   /* keep the reason the join last gave */
      } else {
        sta_failed(MAYA_W4_REASON_USER);
      }
      break;

    case WLAN_REASON_UAP_SUCCESS:
      s_softap_started = true;
      s_softap_failed  = false;
      NeuroN6_DbgPrint("[MayaW476] SoftAP up - beaconing.\r\n");
      break;

    case WLAN_REASON_UAP_CLIENT_ASSOC:   /* data: the station's MAC */
      s_client_count++;
      if (s_softap_wpa) softap_client_waiting((const uint8_t *)data, true);
      NeuroN6_DbgPrint("[MayaW476] SoftAP: client joined.\r\n");
      break;

    case WLAN_REASON_UAP_CLIENT_CONN:    /* handshake complete. data: the MAC */
      softap_client_waiting((const uint8_t *)data, false);
      break;

    case WLAN_REASON_UAP_CLIENT_DISSOC:  /* data: wlan_uap_client_disassoc_t */
      if (s_client_count > 0U) s_client_count--;
      if (data != NULL) {
        softap_client_waiting(((const wlan_uap_client_disassoc_t *)data)->sta_addr, false);
      }
      NeuroN6_DbgPrint("[MayaW476] SoftAP: client left.\r\n");
      break;

    case WLAN_REASON_UAP_START_FAILED:
          s_softap_failed = true;
      NeuroN6_Notice("[MayaW476] the access point did not start.\r\n");
      break;

    case WLAN_REASON_UAP_STOPPED:
      if (s_softap_started) s_link_generation++;
      s_softap_started = false;
      s_client_count   = 0U;
      break;

    case WLAN_REASON_PS_ENTER:
      s_ps_bits |= ps_bit(data);
      break;

    case WLAN_REASON_PS_EXIT:
      s_ps_bits &= ~ps_bit(data);
      break;

    default:
      break;
  }

  return 0;
}

/* Profiles */

static int add_target_network(const maya_w4_sta_creds_t *c)
{
  struct wlan_network net;
  wlan_initialize_sta_network(&net);

  (void)strncpy(net.name, k_sta_profile_name, sizeof(net.name) - 1U);
  net.name[sizeof(net.name) - 1U] = '\0';

  (void)strncpy(net.ssid, c->ssid ? c->ssid : "", sizeof(net.ssid) - 1U);
  net.ssid[sizeof(net.ssid) - 1U] = '\0';
  net.ssid_specific = 1;

  net.security.type = maya_sec_to_wlan(c->security);

  const char *psk = c->psk ? c->psk : "";
  size_t pw_len = strlen(psk);
  if (pw_len > (sizeof(net.security.password) - 1U)) {
    pw_len = sizeof(net.security.password) - 1U;
  }
  (void)memcpy(net.security.password, psk, pw_len);
  net.security.password_len = pw_len;

  size_t psk_len = strlen(psk);
  if (psk_len > (sizeof(net.security.psk) - 1U)) {
    psk_len = sizeof(net.security.psk) - 1U;
  }
  (void)memcpy(net.security.psk, psk, psk_len);
  net.security.psk_len = (uint8_t)psk_len;

  maya_w4_ip_cfg_t ipc;
  maya_w4_get_sta_ip(&ipc);
  if (ipc.ip != 0U) {
    net.ip.ipv4.addr_type = ADDR_TYPE_STATIC;
    net.ip.ipv4.address   = ipc.ip;
    net.ip.ipv4.gw        = ipc.gateway;
    net.ip.ipv4.netmask   = ipc.netmask;
    net.ip.ipv4.dns1      = ipc.dns1;
    net.ip.ipv4.dns2      = ipc.dns2;
  }

  return wlan_add_network(&net);
}

static int add_softap_network(const maya_w4_ap_creds_t *c)
{
  struct wlan_network net;
  wlan_initialize_uap_network(&net);

  /* -1: wlan_add_network picks the default for the security type. */
  net.security.mfpc = (int8_t)-1;
  net.security.mfpr = (int8_t)-1;

  (void)strncpy(net.name, k_uap_profile_name, sizeof(net.name) - 1U);
  net.name[sizeof(net.name) - 1U] = '\0';

  (void)strncpy(net.ssid, c->ssid ? c->ssid : "", sizeof(net.ssid) - 1U);
  net.ssid[sizeof(net.ssid) - 1U] = '\0';
  net.ssid_specific = 1U;

  /* Channel 0 selects one automatically. sec_channel_offset 0 is HT20. */
  net.channel = (unsigned int)c->channel;
  net.sec_channel_offset = 0;

  net.security.type = maya_sec_to_wlan(c->security);
  if (net.security.type != WLAN_SECURITY_NONE) {
    const char *psk = c->psk ? c->psk : "";
    size_t pw_len = strlen(psk);
    if (pw_len > (sizeof(net.security.password) - 1U)) {
      pw_len = sizeof(net.security.password) - 1U;
    }
    (void)memcpy(net.security.password, psk, pw_len);
    net.security.password_len = pw_len;

    size_t psk_len = strlen(psk);
    if (psk_len > (sizeof(net.security.psk) - 1U)) {
      psk_len = sizeof(net.security.psk) - 1U;
    }
    (void)memcpy(net.security.psk, psk, psk_len);
    net.security.psk_len = (uint8_t)psk_len;
  }

  /* wlan.c requires the gateway to equal the address. */
  uint32_t addr_be = ip4_to_netorder(c->ip);
  net.ip.ipv4.addr_type = ADDR_TYPE_STATIC;
  net.ip.ipv4.address   = addr_be;
  net.ip.ipv4.gw        = addr_be;
  net.ip.ipv4.netmask   = ip4_to_netorder(c->netmask);

#if CONFIG_WIFI_CAPA
  /* No 11ac or 11ax bits: this build is 2.4 GHz only. */
  net.wlan_capa = WIFI_SUPPORT_LEGACY | WIFI_SUPPORT_11N;
#endif

  return wlan_add_network(&net);
}

/* Sets the World Wide Safe Mode region, before any add_network. A region
 * enforced by the module's OTP overrides it. Access points' country elements
 * are ignored, so a join cannot change the region, its channels or its power
 * table. */
static void maya_w4_lock_region_2g4(void)
{
  uint8_t ignore = 1U;
  (void)wlan_set_country_ie_ignore(&ignore);

  int rc = wlan_set_region_code(0x00);   /* 0x00: World Wide Safe Mode */
  if (rc == WM_SUCCESS) {
    NeuroN6_DbgPrint("[MayaW476] Region locked to WWSM (2.4 GHz world-safe, "
                     "single SKU US/EU/UK).\r\n");
  } else {
    char dbg[144];
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] wlan_set_region_code(WWSM) rc=%d - keeping the "
             "driver/OTP default region (also WWSM unless OTP-enforced).\r\n",
             rc);
    NeuroN6_DbgPrint(dbg);
  }
}

/* Request handlers. Supervisor task only. */

static bool wait_flag(volatile bool *flag, bool want, uint32_t ms)
{
  for (uint32_t t = 0U; t < ms; t += 50U) {
    if (*flag == want) return true;
    MAYA_DELAY_MS(50U);
  }
  return *flag == want;
}

/* Applies the setDNS() servers over the ones DHCP gave. Callable from any
 * task. */
void maya_w4_wifi_apply_dns(void)
{
  uint32_t d1 = 0U, d2 = 0U;
  maya_w4_get_dns(&d1, &d2);
  if (d1 == 0U && d2 == 0U) return;

  const bool set1 = (d1 != 0U) && (ip_addr_get_ip4_u32(dns_getserver(0)) != d1);
  const bool set2 = (d2 != 0U) && (ip_addr_get_ip4_u32(dns_getserver(1)) != d2);
  if (!set1 && !set2) return;

  ip_addr_t a;
  LOCK_TCPIP_CORE();
  if (set1) { ip_addr_set_ip4_u32(&a, d1); dns_setserver(0, &a); }
  if (set2) { ip_addr_set_ip4_u32(&a, d2); dns_setserver(1, &a); }
  UNLOCK_TCPIP_CORE();
}

/* The driver's scan state, in wifi.c. */
extern int  wifi_event_completion(enum wifi_event event, enum wifi_event_reason result, void *data);
extern bool wifi_user_scan_pending(void);
extern void wifi_user_scan_config_cleanup(void);

/* Waits up to 6 s for the driver's scan to end, then clears it. A scan left
 * pending refuses every later scan and join. */
static void scan_settle(void)
{
  for (uint32_t i = 0U; i < 60U && wifi_user_scan_pending(); ++i) MAYA_DELAY_MS(100U);
  if (!wifi_user_scan_pending()) return;
  NeuroN6_DbgPrint("[MayaW476] a scan never finished - cleared.\r\n");
  wifi_user_scan_config_cleanup();
  (void)wifi_event_completion(WIFI_EVENT_SCAN_RESULT, WIFI_EVENT_REASON_FAILURE, NULL);
  MAYA_DELAY_MS(300U);
}

static void sta_leave(void)
{
  s_sta_wanted = false;
  enum wlan_connection_state st = WLAN_DISCONNECTED;
  if (wlan_get_connection_state(&st) == WM_SUCCESS && st != WLAN_DISCONNECTED) {
    (void)wlan_disconnect();
    for (uint32_t i = 0U; i < 100U; ++i) {
      if (wlan_get_connection_state(&st) == WM_SUCCESS && st == WLAN_DISCONNECTED) break;
      MAYA_DELAY_MS(50U);
    }
  }
  scan_settle();
  if (s_connected) {
    s_connected = false;
    s_link_generation++;
  }
  maya_w4_set_net_status(false, 0U);
}

static void stop_ap(void)
{
  if (s_dhcp_server_started) {
    dhcp_server_stop();
    s_dhcp_server_started = false;
  }
  if (s_softap_started) {
    (void)wlan_stop_network(k_uap_profile_name);
    (void)wait_flag(&s_softap_started, false, 5000U);
    if (s_softap_started) {
      s_softap_started = false;
      s_link_generation++;
    }
  }
  s_client_count = 0U;
  maya_w4_set_net_status(false, 0U);
}

static void radio_on(void)
{
  if (!s_radio_off) return;
  (void)wlan_deepsleepps_off();
  for (uint32_t i = 0U; i < 60U && (s_ps_bits & 2u) != 0u; ++i) MAYA_DELAY_MS(50U);
  s_radio_off = false;
  NeuroN6_DbgPrint("[MayaW476] WiFi radio awake.\r\n");
}

/* Settings of the station being kept joined. begin() with the same ones
 * leaves the join under way. */
static struct {
  char             ssid[65];
  char             psk[65];
  int              security;
  maya_w4_ip_cfg_t ip;
  char             hostname[33];
} s_joining;

static bool same_station(const maya_w4_sta_creds_t *c, const maya_w4_ip_cfg_t *ip)
{
  return strncmp(s_joining.ssid, c->ssid, sizeof(s_joining.ssid)) == 0 &&
         strncmp(s_joining.psk, (c->psk != NULL) ? c->psk : "", sizeof(s_joining.psk)) == 0 &&
         s_joining.security == c->security &&
         memcmp(&s_joining.ip, ip, sizeof(*ip)) == 0 &&
         strncmp(s_joining.hostname, maya_w4_get_hostname(), sizeof(s_joining.hostname)) == 0;
}

static int op_sta(void)
{
  maya_w4_sta_creds_t sta;
  maya_w4_ip_cfg_t ip;
  maya_w4_get_sta_creds(&sta);
  maya_w4_get_sta_ip(&ip);
  if (sta.ssid == NULL || sta.ssid[0] == '\0') return -1;

  if (s_mode == MAYA_W4_MODE_STA && s_sta_wanted && !s_radio_off && same_station(&sta, &ip)) {
    return 0;   /* already joined or joining */
  }

  radio_on();
  if (s_mode == MAYA_W4_MODE_AP) stop_ap();
  sta_leave();
  (void)netifapi_netif_set_default(net_get_sta_interface());
  (void)wlan_remove_network(k_sta_profile_name);   /* absent the first time */

  /* The pointer must outlive the link. */
  (void)net_dhcp_hostname_set((char *)maya_w4_get_hostname());

  int rc = add_target_network(&sta);
  if (rc != WM_SUCCESS) {
    NeuroN6_DbgPrint("[MayaW476] STA add_network FAILED - check SSID/PSK. "
                     "WiFi will not connect.\r\n");
    return (rc < 0) ? rc : -rc;
  }

  s_last_reason     = MAYA_W4_REASON_NONE;
  s_mode            = MAYA_W4_MODE_STA;
  s_backoff_ms      = 0U;
  s_busy_since_ms   = 0U;
  s_next_attempt_ms = now_ms();
  s_sta_wanted      = true;
  (void)snprintf(s_joining.ssid, sizeof(s_joining.ssid), "%s", sta.ssid);
  (void)snprintf(s_joining.psk, sizeof(s_joining.psk), "%s", (sta.psk != NULL) ? sta.psk : "");
  s_joining.security = sta.security;
  s_joining.ip       = ip;
  (void)snprintf(s_joining.hostname, sizeof(s_joining.hostname), "%s", maya_w4_get_hostname());
  NeuroN6_DbgPrint("[MayaW476] STA connecting...\r\n");
  return 0;
}

static int op_ap(void)
{
  maya_w4_ap_creds_t ap;
  maya_w4_get_ap_creds(&ap);

  radio_on();
  if (s_mode == MAYA_W4_MODE_STA) sta_leave();
  stop_ap();
  s_mode = MAYA_W4_MODE_NONE;
  (void)wlan_remove_network(k_uap_profile_name);   /* absent the first time */
  s_softap_failed = false;

  int rc = add_softap_network(&ap);
  if (rc != WM_SUCCESS) {
    char dbg[128];
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] SoftAP add_network FAILED rc=%d. WiFi will not start.\r\n", rc);
    NeuroN6_DbgPrint(dbg);
    s_softap_failed = true;
    return -1;
  }

  {
    const enum wlan_security_type sec = maya_sec_to_wlan(ap.security);
    s_softap_wpa = (sec != WLAN_SECURITY_NONE) && (sec != WLAN_SECURITY_WEP_OPEN);
  }
  taskENTER_CRITICAL();
  (void)memset(s_joining_clients, 0, sizeof(s_joining_clients));
  taskEXIT_CRITICAL();

  rc = wlan_start_network(k_uap_profile_name);
  if (rc != WM_SUCCESS) {
    char dbg[128];
    snprintf(dbg, sizeof(dbg), "[MayaW476] wlan_start_network FAILED rc=%d.\r\n", rc);
    NeuroN6_DbgPrint(dbg);
    s_softap_failed = true;
    return -2;
  }

  /* The DHCP server starts only after UAP_SUCCESS. */
  if (!wait_flag(&s_softap_started, true, 10000U)) {
    char dbg[160];
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] SoftAP UAP_SUCCESS never fired. failed=%u clients=%lu\r\n",
             (unsigned)s_softap_failed, (unsigned long)s_client_count);
    NeuroN6_DbgPrint(dbg);
    return -3;
  }

  maya_w4_report_softap_tx_power();

  if (ap.hidden) {
    (void)wlan_uap_set_hidden_ssid(1);
  }

  MAYA_DELAY_MS(200U);   /* lets the uap netif come up */

  s_mode = MAYA_W4_MODE_AP;
  /* While hosting, the access point is lwIP's default interface, so multicast
   * and broadcast leave by it. */
  (void)netifapi_netif_set_default(net_get_uap_interface());
  dhcp_server_set_pool(ip4_to_host(ap.dhcp_first), ip4_to_host(ap.dhcp_last));
  if (dhcp_server_start(net_get_uap_handle()) == WM_SUCCESS) {
    s_dhcp_server_started = true;
    maya_w4_set_net_status(true, ip4_to_netorder(ap.ip));
    NeuroN6_DbgPrint("[MayaW476] SoftAP DHCP server up - clients can join.\r\n");
  } else {
    NeuroN6_DbgPrint("[MayaW476] SoftAP DHCP server FAILED - clients "
                     "associate but get no IP.\r\n");
  }
  return 0;
}

static int op_disconnect(void)
{
  if (s_mode == MAYA_W4_MODE_STA || s_sta_wanted) sta_leave();
  if (s_mode == MAYA_W4_MODE_AP || s_softap_started) stop_ap();
  s_mode        = MAYA_W4_MODE_NONE;
  s_last_reason = MAYA_W4_REASON_USER;
  return 0;
}

/* end(): disconnects and puts the module's WiFi half in deep sleep. BLE
 * is not affected. */
static int op_off(void)
{
  (void)op_disconnect();
  if (s_radio_off) return 0;
  int rc = wlan_deepsleepps_on();
  if (rc != WM_SUCCESS) {
    char dbg[96];
    snprintf(dbg, sizeof(dbg), "[MayaW476] deep sleep refused rc=%d.\r\n", rc);
    NeuroN6_DbgPrint(dbg);
    return -1;
  }
  for (uint32_t i = 0U; i < 60U && (s_ps_bits & 2u) == 0u; ++i) MAYA_DELAY_MS(50U);
  if ((s_ps_bits & 2u) == 0u) {
    /* The connection manager can accept the request and still refuse it. */
    NeuroN6_DbgPrint("[MayaW476] the module did not enter deep sleep.\r\n");
    return -2;
  }
  s_radio_off = true;
  NeuroN6_DbgPrint("[MayaW476] WiFi radio off (deep sleep).\r\n");
  return 0;
}

/* Runs in wlcmgr context. */
static int scan_done_cb(unsigned int count)
{
  s_scan_count = count;
  (void)xSemaphoreGive(s_scan_done);
  return 0;
}

static uint8_t scan_security(const struct wlan_scan_result *r)
{
  if (r->wpa3_sae && r->wpa2) return 6u;
  if (r->wpa3_sae)            return 5u;
  if (r->wpa && r->wpa2)      return 4u;
  if (r->wpa2)                return 3u;
  if (r->wpa)                 return 2u;
  if (r->wep)                 return 1u;
  return 0u;
}

/* One scan, waited for. Returns 0 when its results are ready, else negative. */
static int scan_once(void)
{
  (void)xSemaphoreTake(s_scan_done, 0);
  int rc = -1;
  for (uint32_t tries = 0U; tries < 6U; ++tries) {
    rc = wlan_scan(scan_done_cb);
    if (rc == WM_SUCCESS) break;
    MAYA_DELAY_MS(500U);          /* a join in progress holds the scanner */
  }
  if (rc != WM_SUCCESS) return -2;
  if (xSemaphoreTake(s_scan_done, pdMS_TO_TICKS(12000U)) == pdTRUE) return 0;

  /* No answer. The scan is cleared the way the driver clears one that a
   * deauthentication ended, which releases the connection manager's scan
   * lock. */
  NeuroN6_DbgPrint("[MayaW476] scan never finished - abandoned.\r\n");
  wifi_user_scan_config_cleanup();
  (void)wifi_event_completion(WIFI_EVENT_SCAN_RESULT, WIFI_EVENT_REASON_FAILURE, NULL);
  MAYA_DELAY_MS(300U);
  (void)xSemaphoreTake(s_scan_done, 0);
  return -3;
}

static int op_scan(void)
{
  if (s_radio_off) return -1;   /* after end(): begin() first */

  /* A join in progress holds the scanner, so it is ended. The station keeper
   * rejoins after the scan. */
  if (s_mode == MAYA_W4_MODE_STA && !s_connected) {
    enum wlan_connection_state st = WLAN_DISCONNECTED;
    if (wlan_get_connection_state(&st) == WM_SUCCESS && st != WLAN_DISCONNECTED) {
      s_internal_disc = true;
      (void)wlan_disconnect();
      for (uint32_t i = 0U; i < 60U; ++i) {
        if (wlan_get_connection_state(&st) == WM_SUCCESS && st == WLAN_DISCONNECTED) break;
        MAYA_DELAY_MS(50U);
      }
    }
    s_next_attempt_ms = now_ms() + STA_BACKOFF_FIRST_MS;
  }
  scan_settle();

  int rc = scan_once();
  if (rc == -3) rc = scan_once();   /* abandoned: one retry */
  if (rc < 0) return rc;

  unsigned n = 0U;
  for (unsigned i = 0U; i < s_scan_count && n < MAYA_W4_SCAN_MAX; ++i) {
    struct wlan_scan_result r;
    if (wlan_get_scan_result(i, &r) != WM_SUCCESS) continue;
    maya_w4_scan_entry_t *e = &s_scan[n];
    size_t len = r.ssid_len < 32U ? r.ssid_len : 32U;
    memcpy(e->ssid, r.ssid, len);
    e->ssid[len] = '\0';
    memcpy(e->bssid, r.bssid, 6);
    e->rssi     = (int8_t)(-(int)r.rssi);   /* the driver reports the magnitude */
    e->channel  = (uint8_t)r.channel;
    e->security = scan_security(&r);
    /* Strongest first. */
    for (unsigned j = n; j > 0U && s_scan[j].rssi > s_scan[j - 1U].rssi; --j) {
      maya_w4_scan_entry_t t = s_scan[j];
      s_scan[j] = s_scan[j - 1U];
      s_scan[j - 1U] = t;
    }
    n++;
  }
  s_scan_n = n;
  return (int)n;
}

/* IEEE power save. Only a joined station can use it. */
static int op_ps(int on)
{
  if (s_mode == MAYA_W4_MODE_AP) return -1;
  int rc = on ? wlan_ieeeps_on(WAKE_ON_UNICAST | WAKE_ON_ALL_BROADCAST |
                               WAKE_ON_MULTICAST | WAKE_ON_ARP_BROADCAST)
              : wlan_ieeeps_off();
  return (rc == WM_SUCCESS) ? 0 : -1;
}

static int handle(int op, int arg)
{
  switch (op) {
    case MAYA_W4_OP_STA:        return op_sta();
    case MAYA_W4_OP_AP:         return op_ap();
    case MAYA_W4_OP_DISCONNECT: return op_disconnect();
    case MAYA_W4_OP_OFF:        return op_off();
    case MAYA_W4_OP_SCAN:       return op_scan();
    case MAYA_W4_OP_PS:         return op_ps(arg);
    default:                    return -1;
  }
}

/* Station keeper: rejoins whenever the station is wanted and not joined. */
static void keep_station(void)
{
  if (s_mode != MAYA_W4_MODE_STA || !s_sta_wanted || s_radio_off || s_connected) {
    s_busy_since_ms = 0U;
    if (s_connected) s_backoff_ms = 0U;
    return;
  }

  const uint32_t now = now_ms();
  enum wlan_connection_state st = WLAN_DISCONNECTED;
  if (wlan_get_connection_state(&st) != WM_SUCCESS) return;

  if (st == WLAN_DISCONNECTED) {
    s_busy_since_ms = 0U;
    if ((int32_t)(now - s_next_attempt_ms) >= 0) {
      (void)wlan_connect((char *)k_sta_profile_name);
      s_backoff_ms = (s_backoff_ms == 0U) ? STA_BACKOFF_FIRST_MS
                   : ((s_backoff_ms * 2U > STA_BACKOFF_MAX_MS) ? STA_BACKOFF_MAX_MS
                                                               : s_backoff_ms * 2U);
      s_next_attempt_ms = now + s_backoff_ms;
    }
    return;
  }

  /* Scanning, associating or waiting for an address. */
  if (s_busy_since_ms == 0U) {
    s_busy_since_ms = now;
  } else if ((now - s_busy_since_ms) > STA_STUCK_MS) {
    NeuroN6_DbgPrint("[MayaW476] STA join stuck for 60 s - retrying.\r\n");
    s_internal_disc = true;
    (void)wlan_disconnect();
    s_busy_since_ms   = 0U;
    s_next_attempt_ms = now + STA_BACKOFF_FIRST_MS;
  }
}

/* SoftAP keeper. The IW610 firmware can leave a client's WPA2 handshake
 * unfinished: such a client is disconnected so that it joins again. */
static void keep_softap_clients(void)
{
  if (s_mode != MAYA_W4_MODE_AP || !s_softap_started || !s_softap_wpa) return;

  const uint32_t now = now_ms();
  for (uint32_t i = 0U; i < SOFTAP_AUTH_SLOTS; ++i) {
    uint8_t mac[6];
    bool    due = false;
    taskENTER_CRITICAL();
    if (s_joining_clients[i].used &&
        (int32_t)(now - s_joining_clients[i].since_ms) > (int32_t)SOFTAP_AUTH_MS) {
      (void)memcpy(mac, s_joining_clients[i].mac, sizeof(mac));
      s_joining_clients[i].used = false;
      due = true;
    }
    taskEXIT_CRITICAL();
    if (due) {
      NeuroN6_DbgPrint("[MayaW476] SoftAP: a client joined but did not complete the "
                       "WPA2 handshake; disconnected it to join again.\r\n");
      (void)wlan_uap_disconnect_sta(mac);
    }
  }
}

static void maya_w4_supervisor_task(void *arg)
{
  (void)arg;

  for (uint32_t i = 0U; i < 300U; ++i) {
    if (s_initialized_count > 0U) break;
    MAYA_DELAY_MS(100U);
  }
  if (s_initialized_count == 0U) {
    NeuroN6_Notice("[MayaW476] the WiFi driver did not start within 30 s.\r\n");
  }
  /* Settling time for wlcmgr after INITIALIZED. */
  MAYA_DELAY_MS(500U);

  /* Power save starts off. lowPowerMode() and end() turn it on. */
  (void)wlan_deepsleepps_off();
  (void)wlan_ieeeps_off();
  MAYA_DELAY_MS(500U);

  maya_w4_lock_region_2g4();

  /* Runs after wlan_init applies its regional table and before add_network. */
  maya_w4_apply_tx_power_cap();
  maya_w4_report_regulatory();

  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250U));

    const uint32_t seq = s_req_seq;
    if (seq != s_done_seq) {
      const int op = s_req_op, a = s_req_arg;
      s_req_result = handle(op, a);
      s_done_seq = seq;
      (void)xSemaphoreGive(s_req_done);
    }

    keep_station();
    keep_softap_clients();

    if (maya_w4_wifi_link_ready()) {
      maya_w4_wifi_apply_dns();
    }
  }
}

/* Called once wlan_init() has succeeded. */
int maya_w4_wifi_connect_start(void)
{
  if (s_task != NULL) return 0;

  if (s_api_lock == NULL)  s_api_lock  = xSemaphoreCreateMutex();
  if (s_req_done == NULL)  s_req_done  = xSemaphoreCreateBinary();
  if (s_scan_done == NULL) s_scan_done = xSemaphoreCreateBinary();
  if (s_api_lock == NULL || s_req_done == NULL || s_scan_done == NULL) {
    NeuroN6_Notice("[MayaW476] WiFi supervisor: out of heap.\r\n");
    return -3;
  }

  int start_rc = wlan_start(wlan_event_callback);
  if (start_rc != WM_SUCCESS) {
    NeuroN6_Notice("[MayaW476] wlan_start FAILED - connection manager "
                     "did not come up.\r\n");
    return -1;
  }

  BaseType_t ok = xTaskCreate(maya_w4_supervisor_task,
                              "maya_w4_conn",
                              1536U,                      /* words: 6 kB */
                              NULL,
                              tskIDLE_PRIORITY + 2,
                              &s_task);
  if (ok != pdPASS) {
    s_task = NULL;
    NeuroN6_Notice("[MayaW476] supervisor task create FAILED (out of heap?).\r\n");
    return -2;
  }
  return 0;
}

int maya_w4_wifi_request(int op, int arg, uint32_t timeout_ms)
{
  if (s_task == NULL) return -100;
  if (xSemaphoreTake(s_api_lock, portMAX_DELAY) != pdTRUE) return -101;

  (void)xSemaphoreTake(s_req_done, 0);        /* a timed-out request's late give */
  const uint32_t my = s_req_seq + 1U;
  s_req_op  = op;
  s_req_arg = arg;
  s_req_seq = my;
  (void)xTaskNotifyGive(s_task);

  int rc = -102;                              /* timed out: result dropped */
  const uint32_t start = now_ms();
  for (;;) {
    const uint32_t spent = now_ms() - start;
    if (spent >= timeout_ms) break;
    if (xSemaphoreTake(s_req_done, pdMS_TO_TICKS(timeout_ms - spent)) != pdTRUE) break;
    if (s_done_seq == my) { rc = s_req_result; break; }
  }
  (void)xSemaphoreGive(s_api_lock);
  return rc;
}

/* Getters. Callable from any task except wlcmgr's callback. */

int32_t maya_w4_wifi_rssi(void)
{
  if (!s_connected) return 0;
  wlan_rssi_info_t info;
  (void)memset(&info, 0, sizeof(info));
  if (wlan_get_signal_info(&info) != WM_SUCCESS) return 0;
  int32_t r = (info.bcn_rssi_last != 0) ? info.bcn_rssi_last : info.bcn_rssi_avg;
  return (r > 0) ? -r : r;
}

static struct wlan_network s_net_scratch;

bool maya_w4_wifi_bssid(uint8_t out[6])
{
  if (s_mode == MAYA_W4_MODE_AP && s_softap_started) {
    return wlan_get_mac_address_uap(out) == WM_SUCCESS;
  }
  if (s_connected && wlan_get_current_network(&s_net_scratch) == WM_SUCCESS) {
    memcpy(out, s_net_scratch.bssid, 6);
    return true;
  }
  return false;
}

bool maya_w4_wifi_mac(uint8_t out[6])
{
  if (s_mode == MAYA_W4_MODE_AP) return wlan_get_mac_address_uap(out) == WM_SUCCESS;
  return wlan_get_mac_address(out) == WM_SUCCESS;
}

uint8_t maya_w4_wifi_channel(void)
{
  if (s_mode == MAYA_W4_MODE_AP && s_softap_started) {
    int ch = 0;
    return (wlan_get_uap_channel(&ch) == WM_SUCCESS) ? (uint8_t)ch : 0u;
  }
  if (s_connected && wlan_get_current_network(&s_net_scratch) == WM_SUCCESS) {
    return (uint8_t)s_net_scratch.channel;
  }
  return 0u;
}

/* Address, gateway and netmask in network byte order. */
bool maya_w4_wifi_addr(uint32_t *ip, uint32_t *gw, uint32_t *mask)
{
  struct wlan_ip_config a;
  (void)memset(&a, 0, sizeof(a));
  int rc = -1;
  if (s_mode == MAYA_W4_MODE_AP && s_softap_started) rc = wlan_get_uap_address(&a);
  else if (s_connected)                               rc = wlan_get_address(&a);
  if (rc != WM_SUCCESS) return false;
  if (ip)   *ip   = a.ipv4.address;
  if (gw)   *gw   = a.ipv4.gw;
  if (mask) *mask = a.ipv4.netmask;
  return true;
}

/* An address, network order, on the board's own subnet. The responders answer
 * nothing else (RFC 6762, 5.5). */
bool maya_w4_wifi_on_link(uint32_t addr)
{
  uint32_t ip = 0U, mask = 0U;
  if (!maya_w4_wifi_addr(&ip, NULL, &mask) || (mask == 0U)) return false;
  return ((addr ^ ip) & mask) == 0U;
}

uint32_t maya_w4_wifi_dns_server(int n)
{
  if (n < 0 || n >= DNS_MAX_SERVERS) return 0U;
  const ip_addr_t *a = dns_getserver((u8_t)n);
  return (a != NULL && IP_IS_V4(a)) ? ip4_addr_get_u32(ip_2_ip4(a)) : 0U;
}

/* Name lookup. Blocks until the answer or the resolver's timeout. */
int maya_w4_wifi_resolve(const char *host, uint32_t *ip_out)
{
  if (host == NULL || ip_out == NULL) return -1;
  ip_addr_t a;
  if (netconn_gethostbyname(host, &a) != ERR_OK || !IP_IS_V4(&a)) return -1;
  *ip_out = ip4_addr_get_u32(ip_2_ip4(&a));
  return 0;
}
