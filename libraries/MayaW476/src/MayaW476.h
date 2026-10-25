/**
 ******************************************************************************
 * @file    MayaW476.h
 * @brief   Arduino WiFi library for the Neuro N6: u-blox MAYA-W476, NXP IW610.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#ifndef MAYA_W476_H
#define MAYA_W476_H

/* Makes the Arduino build compile and link the N6lwIP library. */
#include <N6lwip.h>

#ifdef __cplusplus

#include <stdint.h>
#include <stdbool.h>

#include "IPAddress.h"

/* begin() and beginAP() take WPA2 and WPA3 only. The others describe scan
 * results. */
enum WifiSecurity {
  WIFI_SECURITY_NONE        = 0,   /* open */
  WIFI_SECURITY_WEP         = 1,
  WIFI_SECURITY_WPA         = 2,
  WIFI_SECURITY_WPA2        = 3,   /* the default */
  WIFI_SECURITY_WPA_WPA2    = 4,
  WIFI_SECURITY_WPA3_SAE    = 5,   /* WPA3 personal */
  WIFI_SECURITY_WPA2_WPA3   = 6,
};

/* Values match Arduino's WiFi libraries. */
enum wl_status_t {
  WL_IDLE_STATUS      = 0,   /* before begin() or beginAP(), or still starting */
  WL_NO_SSID_AVAIL    = 1,   /* STA: no network with that name answered */
  WL_SCAN_COMPLETED   = 2,   /* idle, and scanNetworks() has results */
  WL_CONNECTED        = 3,   /* STA associated, DHCP lease acquired */
  WL_CONNECT_FAILED   = 4,   /* STA connect attempt failed */
  WL_CONNECTION_LOST  = 5,   /* STA lost the link and is rejoining */
  WL_DISCONNECTED     = 6,   /* disconnect() or end() */
  WL_AP_LISTENING     = 7,   /* SoftAP up, no client */
  WL_AP_CONNECTED     = 8,   /* SoftAP up, at least one client */
  WL_AP_FAILED        = 9,
  WL_NO_MODULE        = 255  /* IW610 SDIO bring-up or wlan_init failed */
};

/* Returned by encryptionType(). Values match WiFiNINA and WiFi101. */
enum wl_enc_type {
  ENC_TYPE_TKIP    = 2,     /* WPA */
  ENC_TYPE_WPA     = 2,
  ENC_TYPE_CCMP    = 4,     /* WPA2 */
  ENC_TYPE_WPA2    = 4,
  ENC_TYPE_WEP     = 5,
  ENC_TYPE_WPA3    = 6,
  ENC_TYPE_NONE    = 7,     /* open */
  ENC_TYPE_AUTO    = 8,     /* mixed WPA/WPA2 or WPA2/WPA3 */
  ENC_TYPE_UNKNOWN = 255
};

/* Returned by storedMode(). */
enum wl_stored_mode {
  WL_STORED_NONE = 0,
  WL_STORED_STA  = 1,   /* join the stored network */
  WL_STORED_AP   = 2    /* host the stored network */
};

/* Settings for setSoftAP(). */
struct WifiSoftApConfig {
  const char  *ssid;
  const char  *psk;            /* nullptr: the board's own password */
  WifiSecurity security;
  uint8_t      channel;        /* 2.4 GHz, 1 to 11 or 0 for automatic, default 6 */
  bool         hidden;         /* keeps the SSID out of the beacons */

  uint8_t      ip[4];          /* board IP, default {192,168,10,1} */
  uint8_t      netmask[4];     /* default {255,255,255,0} */
  uint8_t      dhcp_start[4];  /* first leasable IP, default {192,168,10,10} */
  uint8_t      dhcp_end[4];    /* last leasable IP, default {192,168,10,99}.
                                  Outside the subnet, the whole subnet is leased */

  WifiSoftApConfig();
};

class MayaW476_t
{
public:
  MayaW476_t() = default;

  /* Joins a WPA2 or WPA3 network with a passphrase of 8 to 63 characters, and
   * returns WL_CONNECT_FAILED for anything else. Waits up to setTimeout() for
   * the link, then keeps trying in the background. */
  wl_status_t begin(const char *ssid, const char *psk,
                    WifiSecurity sec = WIFI_SECURITY_WPA2);
  wl_status_t begin(const char *ssid)   /* open networks are refused */
  { return begin(ssid, nullptr); }

  /* Hosts a WPA2 or WPA3 access point at 192.168.10.1/24 unless config() set
   * another, on channel 1 to 11 or 0 for automatic (above 11 becomes 6). psk
   * nullptr uses devicePassword(), and one under 8 characters returns
   * WL_AP_FAILED. */
  wl_status_t beginAP(const char *ssid, const char *psk,
                      uint8_t channel = 6,
                      WifiSecurity sec = WIFI_SECURITY_WPA2);
  wl_status_t beginAP(const char *ssid)  /* with devicePassword() */
  { return beginAP(ssid, nullptr); }

  /* The board's own SoftAP password, random from the hardware generator. The
   * first call stores it and restarts the board once. */
  const char *devicePassword(void);

  /* Joins or hosts the network a host stored on the board over USB. Returns
   * WL_NO_SSID_AVAIL when nothing is stored. */
  wl_status_t    begin(void);
  wl_stored_mode storedMode(void);
  const char    *storedSSID(void);      /* "" when nothing is stored */

  /* Leaves the network, or stops hosting one. The radio stays on. */
  void disconnect(void);

  /* disconnect(), then turns the WiFi radio off. BLE keeps working. */
  void end(void);

  wl_status_t status(void) const;

  /* All zero until the link is up. */
  IPAddress localIP(void) const;
  IPAddress subnetMask(void) const;
  IPAddress gatewayIP(void) const;
  IPAddress dnsIP(int n = 0) const;      /* n = 0 or 1 */

  /* Call before begin() for a static address, or before beginAP() to set the
   * access point's address and subnet. config(IPAddress()) restores DHCP. */
  void config(IPAddress local_ip);
  void config(IPAddress local_ip, IPAddress dns_server);
  void config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway);
  void config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway,
              IPAddress subnet);

  void setDNS(IPAddress dns_server1);
  void setDNS(IPAddress dns_server1, IPAddress dns_server2);

  /* The DHCP and mDNS name: up to 32 letters, digits and hyphens, default
   * neuron6-xxxxxx. Call before begin(). */
  void setHostname(const char *name);
  const char *getHostname(void) const;

  /* "" when idle. Valid until the next begin() or beginAP(). */
  const char *SSID(void) const;

  /* dBm, or 0 when not joined. */
  int32_t RSSI(void) const;

  /* Writes the access point's MAC, 6 bytes, to bssid. Zeroes if none. */
  uint8_t *BSSID(uint8_t *bssid) const;

  /* 0 if none. */
  uint8_t channel(void) const;

  /* Writes this board's MAC, 6 bytes, to mac. */
  uint8_t *macAddress(uint8_t *mac) const;

  /* Returns the number of networks in range, up to 32, or -1 on failure. They
   * are listed strongest first and stay until the next scan. */
  int8_t      scanNetworks(void);
  const char *SSID(uint8_t networkItem) const;
  int32_t     RSSI(uint8_t networkItem) const;
  uint8_t     encryptionType(uint8_t networkItem) const;   /* wl_enc_type */
  uint8_t    *BSSID(uint8_t networkItem, uint8_t *bssid) const;
  uint8_t     channel(uint8_t networkItem) const;

  /* Returns 1 on success. */
  int hostByName(const char *aHostname, IPAddress &aResult);

  /* Seconds since 1970-01-01 UTC from an NTP server, pool.ntp.org by default,
   * or 0 if none answered. After an answer the server is asked at most once
   * an hour. Only the server's answer to the board's own request is taken,
   * by its address and an echoed nonce. SNTP has no authentication. */
  unsigned long getTime(void);
  void setTimeServer(const char *server);

  /* IEEE power save for a joined station. True if the module accepts it. */
  bool lowPowerMode(void);
  bool noLowPowerMode(void);

  /* A key of 8 to 64 characters that a WiFi host must prove before it gets the
   * stream or sends a command, with no encryption of the stream. "" selects
   * the stored key, and another length returns false. */
  bool setLinkKey(const char *key);

  /* Timeout for begin() and beginAP() in ms, default 7000. */
  void setTimeout(uint32_t ms);

  /* The host server's TCP port and the discovery UDP port, default 5000. Call
   * before Vision.begin(). */
  void setTcpPort(uint16_t port);

  /* Dotted-quad form of localIP(). */
  const char *ipv4String(void) const;

  /* The access point Vision.begin() hosts for TRANSPORT_WIFI_SOFTAP when the
   * sketch has not called beginAP(). */
  void setSoftAP(const WifiSoftApConfig &cfg);

private:
  wl_status_t bringup(int transport_mode);
};

extern MayaW476_t MayaW476;

/* The same object under Arduino's name. */
extern MayaW476_t &WiFi;

#include "WiFiClient.h"
#include "WiFiServer.h"
#include "WiFiUdp.h"

#endif /* __cplusplus */

#ifdef __cplusplus
extern "C" {
#endif

/* Called by Vision.begin() in place of the weak default in NeuroN6.c. Starts
 * the host server on the board's own access point. Returns 0 on success. */
int  MayaW476_Start(int transport_mode);

/* Powers the module and loads its firmware without starting a network, for
 * ArduinoBLE's BLE.begin(). Returns 0 once it runs. */
int  MayaW476_RadioStart(void);

#ifdef __cplusplus
}
#endif

#endif /* MAYA_W476_H */
