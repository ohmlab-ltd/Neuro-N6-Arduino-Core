/**
 ******************************************************************************
 * @file    MayaW476.cpp
 * @brief   Arduino WiFi library for the u-blox MAYA-W476 module, NXP IW610.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "MayaW476.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "NeuroN6.h"

#include "maya_w4_board/maya_w4_creds.h"
#include "maya_w4_board/maya_w4_sha256.h"
#include "maya_w4_board/maya_w4_sockets.h"

#include "NeuroN6_FrameQueue.h"

#include <n6_asset.h>
#include "n6_host_contract.h"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
}

WifiSoftApConfig::WifiSoftApConfig()
  : ssid     (nullptr),
    psk      (nullptr),
    security (WIFI_SECURITY_WPA2),
    channel  (6),
    hidden   (false)
{
  ip[0]         = 192; ip[1]         = 168; ip[2]         = 10; ip[3]         = 1;
  netmask[0]    = 255; netmask[1]    = 255; netmask[2]    = 255; netmask[3]    = 0;
  dhcp_start[0] = 192; dhcp_start[1] = 168; dhcp_start[2] = 10; dhcp_start[3] = 10;
  dhcp_end[0]   = 192; dhcp_end[1]   = 168; dhcp_end[2]   = 10; dhcp_end[3]   = 99;
}

MayaW476_t MayaW476;
MayaW476_t &WiFi = MayaW476;

/* Copies of the sketch's settings, and the link state. */
namespace {
  static char     s_sta_ssid[64]   = {0};
  static char     s_sta_psk[65]    = {0};   /* 64-character PSK plus terminator */
  static int      s_sta_security   = WIFI_SECURITY_WPA2;

  static char     s_ap_ssid[64]    = {0};
  static char     s_ap_psk[65]     = {0};
  static int      s_ap_security    = WIFI_SECURITY_WPA2;
  static uint8_t  s_ap_channel     = 6;
  static bool     s_ap_hidden      = false;
  static uint8_t  s_ap_ip[4]       = {192,168,10,1};
  static uint8_t  s_ap_mask[4]     = {255,255,255,0};
  static uint8_t  s_ap_dhcp_first[4] = {192,168,10,10};
  static uint8_t  s_ap_dhcp_last[4]  = {192,168,10,99};

  /* config(): the station's static address, network order. 0 is DHCP. */
  static uint32_t s_sta_ip = 0U, s_sta_gw = 0U, s_sta_mask = 0U;
  static uint32_t s_sta_dns1 = 0U, s_sta_dns2 = 0U;
  /* setDNS(), network order. 0 is none. */
  static uint32_t s_dns1 = 0U, s_dns2 = 0U;

  static char     s_hostname[33]   = {0};
  static char     s_link_key[65]   = {0};   /* setLinkKey(). "" selects the stored key. */
  static uint16_t s_tcp_port       = 5000;

  static bool     s_fw_loaded      = false;
  static bool     s_bringup_failed = false;
  static bool     s_wlcm_started   = false;   /* connection manager and supervisor */
  static bool     s_wlcm_failed    = false;   /* the last start of them failed */
  static bool     s_scanned_idle   = false;   /* WL_SCAN_COMPLETED */

  /* The one-time bring-up and the stored settings' parse run in one task at a
   * time. Recursive: the WiFi bring-up includes the firmware's. */
  static StaticSemaphore_t s_bringup_storage;
  static SemaphoreHandle_t s_bringup_lock = nullptr;

  static char     s_ipv4_str[16]   = "0.0.0.0";
  static uint32_t s_ipv4_be        = 0U;

  static uint32_t s_timeout_ms     = 7000U;   /* begin() and beginAP() wait */

  /* Request timeouts. The first request also waits up to 30 s for the
   * connection manager. */
  static const uint32_t k_start_timeout_ms = 45000U;
  static const uint32_t k_short_timeout_ms = 15000U;
  static const uint32_t k_scan_timeout_ms  = 30000U;

  /* Channels 1 to 11 are common to FCC and ETSI. Channel 0 selects
   * automatically, and anything above the maximum falls back to the default. */
  static const uint8_t MAYA_SOFTAP_MAX_CHANNEL     = 11;
  static const uint8_t MAYA_SOFTAP_DEFAULT_CHANNEL = 6;

  inline void copy_bounded(char *dst, size_t cap, const char *src)
  {
    if (!dst || cap == 0) return;
    if (!src) { dst[0] = '\0'; return; }
    size_t n = strlen(src);
    if (n >= cap) n = cap - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
  }

  /* The WiFi record a host stored, read once per boot: storing settings
   * resets the board. */
  struct StoredWifi {
    bool    read;
    uint8_t mode;             /* wl_stored_mode */
    char    ssid[33];
    char    key[65];          /* a WPA2 key of 64 hex digits, plus terminator */
    int     security;         /* WifiSecurity, or -1 to infer it from the key */
    uint8_t channel;
    char    hostname[33];
    uint8_t link_key[64];
    uint8_t link_key_len;
  };
  static StoredWifi s_stored;

  void clean_hostname(char *dst, size_t cap, const char *src, size_t n)
  {
    size_t o = 0;
    for (size_t i = 0; src != nullptr && i < n && src[i] != '\0' && o < cap - 1U; ++i) {
      const char c = src[i];
      if (isalnum((unsigned char)c) || c == '-') dst[o++] = (char)tolower((unsigned char)c);
    }
    dst[o] = '\0';
  }

  uint8_t checked_channel(uint8_t channel)
  {
    if (channel <= MAYA_SOFTAP_MAX_CHANNEL) return channel;
    char dbg[176];
    snprintf(dbg, sizeof(dbg),
             "[MayaW476] SoftAP channel %u is not a valid 2.4 GHz channel "
             "(1..%u, or 0=auto) - 5 GHz is disabled on this board; using "
             "channel %u instead.\r\n",
             (unsigned)channel,
             (unsigned)MAYA_SOFTAP_MAX_CHANNEL,
             (unsigned)MAYA_SOFTAP_DEFAULT_CHANNEL);
    NeuroN6_DbgPrint(dbg);
    return MAYA_SOFTAP_DEFAULT_CHANNEL;
  }

  /* WPA2 or WPA3 with a passphrase of 8 to 63 characters, or 64 hex digits for
   * WPA2. Anything else is refused with one line. */
  bool security_ok(const char *what, int sec, const char *psk)
  {
    const size_t n = (psk != nullptr) ? strlen(psk) : 0U;
    bool hex = (n == 64U && sec == WIFI_SECURITY_WPA2);
    for (size_t i = 0; hex && i < n; ++i) hex = (isxdigit((unsigned char)psk[i]) != 0);

    const char *why = nullptr;
    if (sec != WIFI_SECURITY_WPA2 && sec != WIFI_SECURITY_WPA3_SAE &&
        sec != WIFI_SECURITY_WPA2_WPA3) {
      why = "only WPA2 and WPA3 are supported, so open, WEP and WPA networks are refused";
    } else if (n == 0U) {
      why = "a passphrase is needed, as open networks are refused";
    } else if ((n < 8U || n > 63U) && !hex) {
      why = "the passphrase must be 8 to 63 characters";
    }
    if (why == nullptr) return true;
    char msg[176];
    snprintf(msg, sizeof(msg), "[MayaW476] %s: %s.\r\n", what, why);
    NeuroN6_Notice(msg);
    return false;
  }
} /* anonymous namespace */

/* Stored settings */

static void tlv_string(char *dst, size_t cap, const uint8_t *v, uint8_t n)
{
  const size_t len = (n < cap - 1U) ? n : cap - 1U;
  memcpy(dst, v, len);
  dst[len] = '\0';
}

/* Returns true when the lock was taken: only with the scheduler running. */
static bool bringup_lock(void)
{
  if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) return false;
  if (s_bringup_lock == nullptr) {
    vTaskSuspendAll();
    if (s_bringup_lock == nullptr) {
      s_bringup_lock = xSemaphoreCreateRecursiveMutexStatic(&s_bringup_storage);
    }
    (void)xTaskResumeAll();
  }
  return xSemaphoreTakeRecursive(s_bringup_lock, portMAX_DELAY) == pdTRUE;
}

static void bringup_unlock(bool taken)
{
  if (taken) (void)xSemaphoreGiveRecursive(s_bringup_lock);
}

static void stored_wifi_parse(void)
{
  static uint8_t rec[512];
  uint32_t len = 0;

  s_stored.security = -1;
  s_stored.channel  = MAYA_SOFTAP_DEFAULT_CHANNEL;
  if (!n6_asset_record_read(N6_ASSET_REC_WIFI, rec, sizeof(rec), &len)) return;

  for (uint32_t i = 0; i + 2U <= len && i + 2U + rec[i + 1U] <= len; i += 2U + rec[i + 1U]) {
    const uint8_t n = rec[i + 1U];
    const uint8_t *v = rec + i + 2U;
    switch (rec[i]) {
      case N6_WIFI_TLV_MODE:
        if (n == 1U && (v[0] == WL_STORED_STA || v[0] == WL_STORED_AP)) s_stored.mode = v[0];
        break;
      case N6_WIFI_TLV_SSID:     tlv_string(s_stored.ssid, sizeof(s_stored.ssid), v, n); break;
      case N6_WIFI_TLV_KEY:      tlv_string(s_stored.key, sizeof(s_stored.key), v, n);   break;
      case N6_WIFI_TLV_SECURITY:
        if (n == 1U && v[0] <= WIFI_SECURITY_WPA2_WPA3) s_stored.security = v[0];
        break;
      case N6_WIFI_TLV_CHANNEL:  if (n == 1U) s_stored.channel = v[0]; break;
      case N6_WIFI_TLV_HOSTNAME:
        clean_hostname(s_stored.hostname, sizeof(s_stored.hostname), (const char *)v, n);
        break;
      case N6_WIFI_TLV_LINK_KEY:
        if (n >= 8U && n <= sizeof(s_stored.link_key)) {
          memcpy(s_stored.link_key, v, n);
          s_stored.link_key_len = n;
        }
        break;
      default: break;
    }
  }
  /* The record holds the passphrase and the link key, kept in s_stored. */
  maya_w4_wipe(rec, sizeof(rec));
  if (s_stored.ssid[0] == '\0') s_stored.mode = WL_STORED_NONE;
}

/* The WiFi record, parsed once. */
static const StoredWifi &stored_wifi(void)
{
  const bool taken = bringup_lock();
  if (!s_stored.read) {
    stored_wifi_parse();
    s_stored.read = true;
  }
  bringup_unlock(taken);
  return s_stored;
}

/* Copies the stored network into the sketch's settings and returns the
 * supervisor op that starts it, or -1 when its security is refused. Hosting
 * with no key uses the board's own password. */
static int use_stored(const StoredWifi &st)
{
  const int sec = (st.security >= 0) ? st.security : WIFI_SECURITY_WPA2;
  const char *key = (st.mode == WL_STORED_AP && st.key[0] == '\0') ? MayaW476.devicePassword()
                                                                   : st.key;
  if (!security_ok("the WiFi settings stored on this board", sec, key)) return -1;
  char msg[160];
  snprintf(msg, sizeof(msg), "[MayaW476] %s \"%s\", from the WiFi settings stored on this board.\r\n",
           st.mode == WL_STORED_AP ? "hosting" : "joining", st.ssid);
  NeuroN6_DbgPrint(msg);
  if (st.mode == WL_STORED_AP) {
    copy_bounded(s_ap_ssid, sizeof(s_ap_ssid), st.ssid);
    copy_bounded(s_ap_psk,  sizeof(s_ap_psk),  key);
    s_ap_security = sec;
    s_ap_channel  = checked_channel(st.channel);
    s_ap_hidden   = false;
    return MAYA_W4_OP_AP;
  }
  copy_bounded(s_sta_ssid, sizeof(s_sta_ssid), st.ssid);
  copy_bounded(s_sta_psk,  sizeof(s_sta_psk),  st.key);
  s_sta_security = sec;
  return MAYA_W4_OP_STA;
}

/* Bring-up */

/* GPIO, SDMMC1, SDIO probe and IW610 firmware download. */
extern "C" int maya_w4_sdio_bringup(void);
/* Starts the host server task. */
extern "C" int maya_w4_video_wifi_start(uint16_t tcp_port);

/* Powers the module and downloads its firmware, once. BLE needs no more
 * than this. */
static int ensure_firmware(void)
{
  const bool taken = bringup_lock();
  int rc = 0;

  if (s_fw_loaded) {
    rc = 0;
  } else if (s_bringup_failed) {
    rc = -1;
  } else {
    /* Keeps the USB console running through the bring-up. */
    NeuroN6_StreamUsbOnlyStart();
    NeuroN6_DbgPrint("[MayaW476] bring-up: powering the module and loading its firmware.\r\n");

    rc = maya_w4_sdio_bringup();
    if (rc != 0) {
      s_bringup_failed = true;
    } else {
      s_fw_loaded = true;
    }
  }
  bringup_unlock(taken);
  return rc;
}

/* Firmware, then the connection manager and the supervisor. */
static int ensure_wlan(void)
{
  const bool taken = bringup_lock();
  (void)stored_wifi();   /* read in the sketch's task, before the WiFi task starts */
  int rc = ensure_firmware();
  if (rc == 0 && !s_wlcm_started) {
    rc = maya_w4_wifi_connect_start();
    s_wlcm_failed = (rc != 0);
    if (rc != 0) {
      NeuroN6_Notice("[MayaW476] wlan_start / supervisor bring-up failed - "
                       "falling back to USB transport.\r\n");
    } else {
      s_wlcm_started = true;
    }
  }
  bringup_unlock(taken);
  return rc;
}

/* Settings */

void MayaW476_t::setSoftAP(const WifiSoftApConfig &cfg)
{
  copy_bounded(s_ap_ssid, sizeof(s_ap_ssid), cfg.ssid);
  copy_bounded(s_ap_psk,  sizeof(s_ap_psk),  cfg.psk);
  s_ap_security = (int)cfg.security;
  s_ap_channel  = checked_channel(cfg.channel);
  s_ap_hidden   = cfg.hidden;
  memcpy(s_ap_ip,   cfg.ip,      4);
  memcpy(s_ap_mask, cfg.netmask, 4);
  memcpy(s_ap_dhcp_first, cfg.dhcp_start, 4);
  memcpy(s_ap_dhcp_last,  cfg.dhcp_end,   4);
}

void MayaW476_t::setTcpPort(uint16_t port)
{
  s_tcp_port = port;
}

void MayaW476_t::setTimeout(uint32_t ms) { s_timeout_ms = ms; }

void MayaW476_t::config(IPAddress local_ip)
{
  config(local_ip, IPAddress(local_ip[0], local_ip[1], local_ip[2], 1));
}

void MayaW476_t::config(IPAddress local_ip, IPAddress dns_server)
{
  config(local_ip, dns_server, IPAddress(local_ip[0], local_ip[1], local_ip[2], 1));
}

void MayaW476_t::config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway)
{
  config(local_ip, dns_server, gateway, IPAddress(255, 255, 255, 0));
}

void MayaW476_t::config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway,
                        IPAddress subnet)
{
  if (!local_ip.isSet()) {
    s_sta_ip = s_sta_gw = s_sta_mask = s_sta_dns1 = s_sta_dns2 = 0U;
    const uint8_t ap[4] = {192, 168, 10, 1}, mask[4] = {255, 255, 255, 0};
    memcpy(s_ap_ip, ap, 4);
    memcpy(s_ap_mask, mask, 4);
    return;
  }
  s_sta_ip   = local_ip.raw();
  s_sta_dns1 = dns_server.raw();
  s_sta_dns2 = 0U;
  s_sta_gw   = gateway.raw();
  s_sta_mask = subnet.raw();
  for (int i = 0; i < 4; i++) {
    s_ap_ip[i]   = local_ip[i];
    s_ap_mask[i] = subnet[i];
  }
}

void MayaW476_t::setDNS(IPAddress dns_server1)
{
  setDNS(dns_server1, IPAddress());
}

void MayaW476_t::setDNS(IPAddress dns_server1, IPAddress dns_server2)
{
  s_dns1 = dns_server1.raw();
  s_dns2 = dns_server2.raw();
  if (maya_w4_wifi_link_ready()) maya_w4_wifi_apply_dns();
}

void MayaW476_t::setHostname(const char *name)
{
  /* An empty name falls back to the stored name or the default. */
  clean_hostname(s_hostname, sizeof(s_hostname), name, name != nullptr ? strlen(name) : 0U);
}

bool MayaW476_t::setLinkKey(const char *key)
{
  const size_t n = (key != nullptr) ? strlen(key) : 0U;
  if (n != 0U && (n < 8U || n >= sizeof(s_link_key))) return false;
  copy_bounded(s_link_key, sizeof(s_link_key), key);
  return true;
}

const char *MayaW476_t::getHostname(void) const
{
  return maya_w4_get_hostname();
}

/* Joining and hosting */

wl_status_t MayaW476_t::status(void) const
{
  if (s_bringup_failed || s_wlcm_failed) return WL_NO_MODULE;
  if (!s_wlcm_started)  return s_scanned_idle ? WL_SCAN_COMPLETED : WL_IDLE_STATUS;

  switch (maya_w4_wifi_mode()) {
    case MAYA_W4_MODE_STA:
      if (maya_w4_wifi_sta_connected()) return WL_CONNECTED;
      switch (maya_w4_wifi_last_reason()) {
        case MAYA_W4_REASON_NOT_FOUND: return WL_NO_SSID_AVAIL;
        case MAYA_W4_REASON_AUTH:
        case MAYA_W4_REASON_FAILED:    return WL_CONNECT_FAILED;
        case MAYA_W4_REASON_LINK_LOST: return WL_CONNECTION_LOST;
        default:                       return WL_IDLE_STATUS;   /* still joining */
      }
    case MAYA_W4_MODE_AP:
      if (maya_w4_wifi_softap_failed()) return WL_AP_FAILED;
      if (maya_w4_wifi_softap_started()) {
        return maya_w4_wifi_softap_client_count() > 0U ? WL_AP_CONNECTED : WL_AP_LISTENING;
      }
      return WL_IDLE_STATUS;
    default:
      if (maya_w4_wifi_softap_failed())                        return WL_AP_FAILED;
      if (maya_w4_wifi_last_reason() == MAYA_W4_REASON_USER)   return WL_DISCONNECTED;
      return s_scanned_idle ? WL_SCAN_COMPLETED : WL_IDLE_STATUS;
  }
}

wl_status_t MayaW476_t::bringup(int transport_mode)
{
  if (ensure_wlan() != 0) return WL_NO_MODULE;

  s_scanned_idle = false;
  const bool sta = (transport_mode == TRANSPORT_WIFI_STA);
  const int rc = maya_w4_wifi_request(sta ? MAYA_W4_OP_STA : MAYA_W4_OP_AP, 0, k_start_timeout_ms);
  if (rc < 0) return sta ? WL_CONNECT_FAILED : WL_AP_FAILED;

  const uint32_t start = HAL_GetTick();
  for (;;) {
    const wl_status_t s = status();
    if (sta) {
      if (s == WL_CONNECTED || s == WL_CONNECT_FAILED ||
          s == WL_NO_SSID_AVAIL || s == WL_NO_MODULE) return s;
    } else {
      if (s == WL_AP_LISTENING || s == WL_AP_CONNECTED ||
          s == WL_AP_FAILED    || s == WL_NO_MODULE)  return s;
    }
    if ((HAL_GetTick() - start) >= s_timeout_ms) return s;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

wl_status_t MayaW476_t::begin(const char *ssid, const char *psk, WifiSecurity sec)
{
  if (!security_ok("begin()", (int)sec, psk)) return WL_CONNECT_FAILED;
  copy_bounded(s_sta_ssid, sizeof(s_sta_ssid), ssid);
  copy_bounded(s_sta_psk,  sizeof(s_sta_psk),  psk);
  s_sta_security = (int)sec;
  return bringup(TRANSPORT_WIFI_STA);
}

wl_status_t MayaW476_t::begin(void)
{
  const StoredWifi &st = stored_wifi();
  if (st.mode == WL_STORED_NONE) {
    NeuroN6_Notice("[MayaW476] begin(): no WiFi settings are stored on this board. Store "
                     "them from Neuro Studio, or with tools/bench/n6bench.py wifiset.\r\n");
    return WL_NO_SSID_AVAIL;
  }
  const int op = use_stored(st);
  if (op < 0) return (st.mode == WL_STORED_AP) ? WL_AP_FAILED : WL_CONNECT_FAILED;
  return bringup(op == MAYA_W4_OP_AP ? TRANSPORT_WIFI_SOFTAP : TRANSPORT_WIFI_STA);
}

wl_stored_mode MayaW476_t::storedMode(void)
{
  return (wl_stored_mode)stored_wifi().mode;
}

const char *MayaW476_t::storedSSID(void)
{
  return stored_wifi().ssid;
}

wl_status_t MayaW476_t::beginAP(const char *ssid, const char *psk,
                                uint8_t channel, WifiSecurity sec)
{
  if (psk == nullptr) psk = devicePassword();
  if (!security_ok("beginAP()", (int)sec, psk)) return WL_AP_FAILED;
  copy_bounded(s_ap_ssid, sizeof(s_ap_ssid), ssid);
  copy_bounded(s_ap_psk,  sizeof(s_ap_psk),  psk);
  s_ap_security = (int)sec;
  s_ap_channel  = checked_channel(channel);
  s_ap_hidden   = false;
  return bringup(TRANSPORT_WIFI_SOFTAP);
}

void MayaW476_t::disconnect(void)
{
  s_scanned_idle = false;
  if (s_wlcm_started) (void)maya_w4_wifi_request(MAYA_W4_OP_DISCONNECT, 0, k_short_timeout_ms);
}

void MayaW476_t::end(void)
{
  s_scanned_idle = false;
  if (s_wlcm_started) (void)maya_w4_wifi_request(MAYA_W4_OP_OFF, 0, k_short_timeout_ms);
}

bool MayaW476_t::lowPowerMode(void)
{
  return s_wlcm_started && maya_w4_wifi_request(MAYA_W4_OP_PS, 1, k_short_timeout_ms) == 0;
}

bool MayaW476_t::noLowPowerMode(void)
{
  return s_wlcm_started && maya_w4_wifi_request(MAYA_W4_OP_PS, 0, k_short_timeout_ms) == 0;
}

/* Link state */

IPAddress MayaW476_t::localIP(void) const
{
  return IPAddress(s_ipv4_be);
}

IPAddress MayaW476_t::subnetMask(void) const
{
  uint32_t mask = 0U;
  return maya_w4_wifi_addr(nullptr, nullptr, &mask) ? IPAddress(mask) : IPAddress();
}

IPAddress MayaW476_t::gatewayIP(void) const
{
  uint32_t gw = 0U;
  return maya_w4_wifi_addr(nullptr, &gw, nullptr) ? IPAddress(gw) : IPAddress();
}

IPAddress MayaW476_t::dnsIP(int n) const
{
  return s_wlcm_started ? IPAddress(maya_w4_wifi_dns_server(n)) : IPAddress();
}

const char *MayaW476_t::SSID(void) const
{
  switch (maya_w4_wifi_mode()) {
    case MAYA_W4_MODE_STA: return s_sta_ssid;
    case MAYA_W4_MODE_AP:  return s_ap_ssid;
    default:               return "";
  }
}

int32_t MayaW476_t::RSSI(void) const
{
  return s_wlcm_started ? maya_w4_wifi_rssi() : 0;
}

uint8_t *MayaW476_t::BSSID(uint8_t *bssid) const
{
  if (bssid == nullptr) return bssid;
  if (!s_wlcm_started || !maya_w4_wifi_bssid(bssid)) memset(bssid, 0, 6);
  return bssid;
}

uint8_t MayaW476_t::channel(void) const
{
  return s_wlcm_started ? maya_w4_wifi_channel() : 0u;
}

uint8_t *MayaW476_t::macAddress(uint8_t *mac) const
{
  if (mac == nullptr) return mac;
  if (!s_fw_loaded || !maya_w4_wifi_mac(mac)) memset(mac, 0, 6);
  return mac;
}

const char *MayaW476_t::ipv4String(void) const { return s_ipv4_str; }

int MayaW476_t::hostByName(const char *aHostname, IPAddress &aResult)
{
  if (aHostname == nullptr) return 0;
  IPAddress direct;
  if (direct.fromString(aHostname)) { aResult = direct; return 1; }
  if (!s_wlcm_started) return 0;
  uint32_t ip = 0U;
  if (maya_w4_wifi_resolve(aHostname, &ip) != 0) return 0;
  aResult = IPAddress(ip);
  return 1;
}

/* Network time: SNTP, RFC 4330 */

namespace {
  static char     s_time_server[64] = "pool.ntp.org";
  static uint32_t s_time_epoch      = 0U;   /* seconds at s_time_ms */
  static uint32_t s_time_ms         = 0U;   /* HAL tick of the last answer, 0 = none */
  static const uint32_t k_ntp_to_unix       = 2208988800UL;
  static const uint32_t k_time_refresh_ms   = 3600000UL;
}

void MayaW476_t::setTimeServer(const char *server)
{
  copy_bounded(s_time_server, sizeof(s_time_server), server ? server : "pool.ntp.org");
  s_time_ms = 0U;
}

static unsigned long time_now(void)
{
  return (s_time_ms != 0U) ? s_time_epoch + (HAL_GetTick() - s_time_ms) / 1000U : 0UL;
}

/* A transmit timestamp nobody else can predict, which the server's answer
 * returns as its originate timestamp. */
static void ntp_nonce(uint8_t out[8])
{
  static uint32_t count;
  const uint32_t mix[5] = {
    HAL_GetUIDw0() ^ HAL_GetUIDw2(), HAL_GetUIDw1(), ++count, DWT->CYCCNT, HAL_GetTick(),
  };
  maya_w4_sha256_t c;
  uint8_t d[32];
  maya_w4_sha256_init(&c);
  maya_w4_sha256_update(&c, mix, sizeof(mix));
  maya_w4_sha256_final(&c, d);
  memcpy(out, d, 8U);
}

/* One request on a socket connected to the server, and an answer carrying its
 * nonce within 2 s. */
static bool ntp_ask(int s)
{
  uint8_t pkt[48], nonce[8];
  memset(pkt, 0, sizeof(pkt));
  pkt[0] = 0x23;                            /* LI 0, version 4, mode 3 client */
  ntp_nonce(nonce);
  memcpy(&pkt[40], nonce, sizeof(nonce));   /* transmit timestamp */
  if (lwip_send(s, pkt, sizeof(pkt), 0) != (int)sizeof(pkt)) return false;

  const uint32_t sent = HAL_GetTick();
  while ((HAL_GetTick() - sent) < 2000U) {
    const int n = lwip_recv(s, pkt, sizeof(pkt), MSG_DONTWAIT);
    if (n >= 48 && (pkt[0] & 0x07u) == 4u && pkt[1] != 0u &&   /* server reply, synchronised */
        memcmp(&pkt[24], nonce, sizeof(nonce)) == 0) {          /* originate: this request */
      const uint32_t secs = ((uint32_t)pkt[40] << 24) | ((uint32_t)pkt[41] << 16) |
                            ((uint32_t)pkt[42] << 8)  |  (uint32_t)pkt[43];
      s_time_epoch = secs - k_ntp_to_unix;
      s_time_ms    = HAL_GetTick();
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return false;
}

unsigned long MayaW476_t::getTime(void)
{
  if (s_time_ms != 0U && (HAL_GetTick() - s_time_ms) < k_time_refresh_ms) return time_now();
  if (!maya_w4_wifi_link_ready()) return time_now();

  IPAddress server;
  if (!hostByName(s_time_server, server)) return time_now();

  const int s = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s < 0) return time_now();
  struct sockaddr_in to;
  memset(&to, 0, sizeof(to));
  to.sin_family      = AF_INET;
  to.sin_port        = htons(123);
  to.sin_addr.s_addr = server.raw();
  if (lwip_connect(s, (const struct sockaddr *)&to, sizeof(to)) == 0) {
    for (int attempt = 0; attempt < 3 && !ntp_ask(s); attempt++) { }
  }
  (void)lwip_close(s);
  return time_now();
}

/* Scanning */

int8_t MayaW476_t::scanNetworks(void)
{
  if (ensure_wlan() != 0) return -1;
  const int n = maya_w4_wifi_request(MAYA_W4_OP_SCAN, 0, k_scan_timeout_ms);
  if (n < 0) return -1;
  if (maya_w4_wifi_mode() == MAYA_W4_MODE_NONE) s_scanned_idle = true;
  return (int8_t)n;
}

static const maya_w4_scan_entry_t *scan_entry(uint8_t i)
{
  const maya_w4_scan_entry_t *all = nullptr;
  const unsigned n = maya_w4_wifi_scan_results(&all);
  return (all != nullptr && i < n) ? &all[i] : nullptr;
}

const char *MayaW476_t::SSID(uint8_t networkItem) const
{
  const maya_w4_scan_entry_t *e = scan_entry(networkItem);
  return e ? e->ssid : "";
}

int32_t MayaW476_t::RSSI(uint8_t networkItem) const
{
  const maya_w4_scan_entry_t *e = scan_entry(networkItem);
  return e ? e->rssi : 0;
}

uint8_t MayaW476_t::encryptionType(uint8_t networkItem) const
{
  const maya_w4_scan_entry_t *e = scan_entry(networkItem);
  if (e == nullptr) return ENC_TYPE_UNKNOWN;
  switch (e->security) {
    case 0:  return ENC_TYPE_NONE;
    case 1:  return ENC_TYPE_WEP;
    case 2:  return ENC_TYPE_TKIP;
    case 3:  return ENC_TYPE_CCMP;
    case 5:  return ENC_TYPE_WPA3;
    case 4:
    case 6:  return ENC_TYPE_AUTO;
    default: return ENC_TYPE_UNKNOWN;
  }
}

uint8_t *MayaW476_t::BSSID(uint8_t networkItem, uint8_t *bssid) const
{
  if (bssid == nullptr) return bssid;
  const maya_w4_scan_entry_t *e = scan_entry(networkItem);
  if (e) memcpy(bssid, e->bssid, 6); else memset(bssid, 0, 6);
  return bssid;
}

uint8_t MayaW476_t::channel(uint8_t networkItem) const
{
  const maya_w4_scan_entry_t *e = scan_entry(networkItem);
  return e ? e->channel : 0u;
}

/* Settings and status bridge for the C code, maya_w4_creds.h */

extern "C" void maya_w4_get_sta_creds(maya_w4_sta_creds_t *out)
{
  if (!out) return;
  out->ssid     = s_sta_ssid;
  out->psk      = s_sta_psk;
  out->security = s_sta_security;
}

extern "C" void maya_w4_get_ap_creds(maya_w4_ap_creds_t *out)
{
  if (!out) return;
  out->ssid     = s_ap_ssid;
  out->psk      = s_ap_psk;
  out->security = s_ap_security;
  out->channel  = s_ap_channel;
  out->hidden   = s_ap_hidden;
  memcpy(out->ip,      s_ap_ip,   4);
  memcpy(out->netmask, s_ap_mask, 4);
  memcpy(out->dhcp_first, s_ap_dhcp_first, 4);
  memcpy(out->dhcp_last,  s_ap_dhcp_last,  4);
}

extern "C" void maya_w4_get_sta_ip(maya_w4_ip_cfg_t *out)
{
  if (!out) return;
  out->ip      = s_sta_ip;
  out->gateway = s_sta_gw;
  out->netmask = s_sta_mask;
  out->dns1    = s_sta_dns1;
  out->dns2    = s_sta_dns2;
}

extern "C" void maya_w4_get_dns(uint32_t *dns1, uint32_t *dns2)
{
  if (dns1) *dns1 = s_dns1;
  if (dns2) *dns2 = s_dns2;
}

extern "C" const char *maya_w4_get_hostname(void)
{
  static char s_default[16] = {0};
  if (s_hostname[0] != '\0') return s_hostname;
  if (stored_wifi().hostname[0] != '\0') return s_stored.hostname;
  if (s_default[0] == '\0') {
    const uint32_t uid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
    snprintf(s_default, sizeof(s_default), "neuron6-%06lx", (unsigned long)(uid & 0xFFFFFFu));
  }
  return s_default;
}

/* Fills out from the hardware random number generator, started as ST's HAL
 * starts it. False on a seed or clock error. */
static bool hw_random(uint8_t *out, size_t n)
{
  __HAL_RCC_RNG_CLK_ENABLE();
  RNG->CR = (RNG->CR & ~(RNG_CR_RNGEN | RNG_CR_CED)) | RNG_CR_CONDRST;   /* clock check on */
  RNG->CR &= ~RNG_CR_CONDRST;
  uint32_t t0 = HAL_GetTick();
  while ((RNG->CR & RNG_CR_CONDRST) != 0U) {
    if ((HAL_GetTick() - t0) > 10U) return false;
  }
  RNG->SR = 0U;   /* clears the seed and clock error flags */
  RNG->CR |= RNG_CR_RNGEN;

  bool ok = true;
  for (size_t i = 0; ok && i < n; ) {
    t0 = HAL_GetTick();
    while (ok && (RNG->SR & RNG_SR_DRDY) == 0U) {
      ok = ((RNG->SR & (RNG_SR_SECS | RNG_SR_CECS)) == 0U) && ((HAL_GetTick() - t0) <= 10U);
    }
    if (!ok) break;
    const uint32_t w = RNG->DR;
    /* A word read during a seed or clock error is not used. */
    ok = ((RNG->SR & (RNG_SR_SEIS | RNG_SR_CEIS)) == 0U);
    for (unsigned b = 0; ok && b < 4U && i < n; ++b) out[i++] = (uint8_t)(w >> (8U * b));
  }
  RNG->CR &= ~RNG_CR_RNGEN;
  __HAL_RCC_RNG_CLK_DISABLE();
  return ok;
}

/* xxxx-xxxx-xxxx-xxxx from 32 letters and digits, 80 random bits. l, o, 0
 * and 1 look alike and are left out. */
static bool new_password(char out[20])
{
  static const char k_chars[] = "abcdefghijkmnpqrstuvwxyz23456789";
  uint8_t r[16];
  if (!hw_random(r, sizeof(r)) && !hw_random(r, sizeof(r))) return false;
  size_t o = 0;
  for (size_t i = 0; i < sizeof(r); ++i) {
    if (i > 0U && (i % 4U) == 0U) out[o++] = '-';
    out[o++] = k_chars[r[i] & 31U];
  }
  out[o] = '\0';
  maya_w4_wipe(r, sizeof(r));
  return true;
}

static bool printable(const char *s, uint32_t n)
{
  for (uint32_t i = 0; i < n; ++i) {
    if ((unsigned char)s[i] < 0x20U || (unsigned char)s[i] > 0x7EU) return false;
  }
  return true;
}

const char *MayaW476_t::devicePassword(void)
{
  static char s_pass[64] = {0};
  if (s_pass[0] != '\0') return s_pass;

  uint32_t n = 0U;
  if (n6_asset_record_read(N6_ASSET_REC_AP_PASSWORD, s_pass, sizeof(s_pass) - 1U, &n) &&
      n >= 8U && printable(s_pass, n)) {
    s_pass[n] = '\0';
    return s_pass;
  }
  if (!new_password(s_pass)) {
    s_pass[0] = '\0';
    NeuroN6_Notice("[MayaW476] the hardware random number generator failed, so the board "
                   "has no SoftAP password.\r\n");
    return s_pass;
  }

  /* A stage the bootloader left, or its error, means a restart would not
   * store it either. */
  const uint32_t last = n6_asset_last_result();
  if (n6_asset_pending() || (last != 0U && last != N6_ASSET_RESULT_OK) ||
      !n6_asset_record_stage(N6_ASSET_REC_AP_PASSWORD, s_pass, (uint32_t)strlen(s_pass))) {
    NeuroN6_Notice("[MayaW476] the board's SoftAP password could not be stored, so it "
                   "changes at the next restart.\r\n");
    return s_pass;
  }
  NeuroN6_Notice("[MayaW476] storing the board's new SoftAP password: the board restarts "
                 "once.\r\n");
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) vTaskDelay(pdMS_TO_TICKS(200));
  n6_asset_apply();
  return s_pass;
}

extern "C" size_t maya_w4_get_link_key(uint8_t *out, size_t cap)
{
  size_t n = strlen(s_link_key);
  const uint8_t *k = (const uint8_t *)s_link_key;
  if (n == 0U) {
    n = stored_wifi().link_key_len;
    k = s_stored.link_key;
  }
  if (n > cap) return 0U;
  memcpy(out, k, n);
  return n;
}

extern "C" uint16_t maya_w4_get_tcp_port(void)
{
  return s_tcp_port;
}

extern "C" void maya_w4_set_net_status(bool net_ready, uint32_t ipv4_be)
{
  s_ipv4_be = net_ready ? ipv4_be : 0U;
  if (ipv4_be != 0u) {
    const unsigned a = (ipv4_be      ) & 0xFFu;
    const unsigned b = (ipv4_be >>  8) & 0xFFu;
    const unsigned c = (ipv4_be >> 16) & 0xFFu;
    const unsigned d = (ipv4_be >> 24) & 0xFFu;
    snprintf(s_ipv4_str, sizeof(s_ipv4_str), "%u.%u.%u.%u", a, b, c, d);
  } else if (!net_ready) {
    copy_bounded(s_ipv4_str, sizeof(s_ipv4_str), "0.0.0.0");
  }
}

/* Override the core's weak accessors, which the USB task reads for the
 * META_BOARD_IPV4 announce. The address is zero until the board's own access
 * point is up, as the host server runs on it only. */
extern "C" uint32_t MayaW476_GetIPv4_BE(void)
{
  return (maya_w4_wifi_mode() == MAYA_W4_MODE_AP) ? s_ipv4_be : 0U;
}
extern "C" uint16_t MayaW476_GetIPv4_Port(void) { return s_tcp_port; }

/* Overrides the core's weak hook for the USB-only META_BOARD_SOFTAP announce:
 * { u8 ssid_len, ssid, u8 pass_len, passphrase } while the access point is up. */
extern "C" uint32_t MayaW476_SoftApAnnounce(uint8_t *out, uint32_t cap)
{
  if (maya_w4_wifi_mode() != MAYA_W4_MODE_AP || !maya_w4_wifi_softap_started()) return 0U;
  const size_t sn = strlen(s_ap_ssid);
  const size_t pn = strlen(s_ap_psk);
  if (sn == 0U || sn > 32U || pn == 0U || (2U + sn + pn) > cap) return 0U;
  out[0] = (uint8_t)sn;
  memcpy(&out[1], s_ap_ssid, sn);
  out[1U + sn] = (uint8_t)pn;
  memcpy(&out[2U + sn], s_ap_psk, pn);
  return (uint32_t)(2U + sn + pn);
}

/* C hooks */

extern "C" {

int MayaW476_RadioStart(void)
{
  return ensure_firmware();
}

int MayaW476_Start(int transport_mode)
{
  if (transport_mode != TRANSPORT_WIFI_SOFTAP) {
    NeuroN6_Notice("[MayaW476] the WiFi stream runs on the board's own access point only: "
                   "use TRANSPORT_WIFI_SOFTAP.\r\n");
    return -3;
  }
  if (ensure_wlan() != 0) return -1;

  /* An access point the sketch started with beginAP() is kept. */
  const int mode = maya_w4_wifi_mode();
  if (mode == MAYA_W4_MODE_STA) {
    NeuroN6_Notice("[MayaW476] the board joined a network with begin(), and the WiFi stream "
                   "runs on its own access point only.\r\n");
    return -3;
  }
  if (mode != MAYA_W4_MODE_AP) {
    /* The sketch's settings, else the stored ones. */
    const StoredWifi &st = stored_wifi();
    if (s_ap_ssid[0] == '\0' && st.mode == WL_STORED_AP && use_stored(st) < 0) return -2;
    if (s_ap_ssid[0] == '\0') {
      NeuroN6_Notice("[MayaW476] cfg.transport asks for an access point, and none is set: call "
                     "MayaW476.beginAP(ssid), or store WiFi settings on the board from Neuro "
                     "Studio.\r\n");
      return -2;
    }
    if (s_ap_psk[0] == '\0') copy_bounded(s_ap_psk, sizeof(s_ap_psk), MayaW476.devicePassword());
    if (!security_ok("setSoftAP()", s_ap_security, s_ap_psk)) return -2;

    const int rc = maya_w4_wifi_request(MAYA_W4_OP_AP, 0, k_start_timeout_ms);
    if (rc < 0) {
      NeuroN6_Notice("[MayaW476] could not start the access point - "
                       "falling back to USB transport.\r\n");
      return rc;
    }
  }
  if (maya_w4_video_wifi_start(s_tcp_port) != 0) return -1;
  NeuroN6_DbgPrint("[MayaW476] WiFi transport active. Point Neuro "
                   "Studio at the board's IP on the configured TCP port.\r\n");
  return 0;
}

} /* extern "C" */
