/*
  WiFiApi

  A command shell on the USB serial console for checking the WiFi API on a
  board, driven by tools/bench/n6wifiapi.py. One command per line, every
  answer line starts "[api]", and each command ends with "[api] ok <cmd>" or
  "[api] fail <cmd> <why>".

    ap SSID KEY [CH [SEC]]    beginAP (KEY "-" for the board's own password; SEC
                              is a WifiSecurity, 5 WPA3, 6 WPA2/WPA3, default WPA2)
    sta SSID KEY [MS [SEC]]   begin, waiting MS (default 20000; KEY "-" for none)
    cfg IP [DNS [GW [MASK]]]  config
    dns IP [IP2]              setDNS
    host NAME                 setHostname ("-" for the default)
    stat                      status, addressing, signal, MACs
    scan                      scanNetworks
    disc                      disconnect
    end                       end
    lp 1|0                    lowPowerMode / noLowPowerMode
    resolve NAME              hostByName
    tcp HOST PORT N           connect, send N bytes, read the N echoed back
    get HOST PORT PATH        HTTP/1.0 GET: status line and body length
    srv PORT                  echo server, serviced from loop()
    udp PORT                  echo every datagram back to its sender
    usend HOST PORT TEXT      send one datagram
    mcast GROUP PORT          report datagrams sent to a multicast group
    time SERVER               getTime from that server
    stored                    storedMode, storedSSID and the hostname in use
    sbegin                    begin() from the stored settings
    asset                     what the asset store holds: the user asset, the WiFi record
    awrite TEXT               store TEXT as the user asset (the board restarts)
    key TEXT                  setLinkKey ("-" for none)
    aclear                    remove the user asset (the board restarts)
    pass                      devicePassword (storing it restarts the board)
    pclear                    remove the board's own password (the board restarts)
    wstart MODE               MayaW476_Start: 1 the board's access point, 2 station
    reboot                    restart the board

  Hardware: Neuro N6 with the MAYA-W476 wireless module.

  This example code is in the public domain.
*/

#include <NeuroN6.h>
#include <MayaW476.h>
#include <n6_asset.h>
#include <stdarg.h>

/* The module's power-save state: 1 IEEE power save, 2 deep sleep. */
extern "C" unsigned maya_w4_wifi_ps_bits(void);

static WiFiServer *g_srv = nullptr;
static WiFiUDP     g_udp;
static bool        g_udp_on = false;
static WiFiUDP     g_mc;
static bool        g_mc_on = false;

static char line[256];
static size_t line_len = 0;

static const char *mac_str(const uint8_t *m, char *out)
{
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
  return out;
}

static void say(const char *fmt, ...)
{
  char buf[240];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print("[api] ");
  Serial.println(buf);
}

static bool ip_arg(const char *s, IPAddress &ip)
{
  return s != nullptr && ip.fromString(s);
}

static void stat(void)
{
  uint8_t m[6];
  char a[18], b[18];
  WiFi.BSSID(m);
  mac_str(m, a);
  WiFi.macAddress(m);
  mac_str(m, b);
  say("stat status=%d ip=%s mask=%s gw=%s dns=%s ssid=%s rssi=%ld ch=%u bssid=%s mac=%s host=%s ps=%u",
      (int)WiFi.status(), WiFi.localIP().toString().c_str(), WiFi.subnetMask().toString().c_str(),
      WiFi.gatewayIP().toString().c_str(), WiFi.dnsIP().toString().c_str(), WiFi.SSID(),
      (long)WiFi.RSSI(), (unsigned)WiFi.channel(), a, b, WiFi.getHostname(),
      maya_w4_wifi_ps_bits());
}

static void tcp_echo(const char *host, uint16_t port, int n)
{
  WiFiClient c;
  const uint32_t t0 = millis();
  if (!c.connect(host, port)) { say("fail tcp connect"); return; }
  const uint32_t t1 = millis();
  static uint8_t buf[4096];
  int sent = 0, got = 0, bad = 0;
  while (sent < n) {
    int k = (n - sent) < (int)sizeof(buf) ? (n - sent) : (int)sizeof(buf);
    for (int i = 0; i < k; i++) buf[i] = (uint8_t)((sent + i) * 7 + 3);
    if ((int)c.write(buf, k) != k) break;
    sent += k;
    while (c.available() > 0) {   /* drained here, or the peer stalls */
      int r = c.read(buf, sizeof(buf));
      for (int i = 0; i < r; i++) if (buf[i] != (uint8_t)((got + i) * 7 + 3)) bad++;
      got += (r > 0) ? r : 0;
    }
  }
  const uint32_t t_read = millis();
  while (got < n && (millis() - t_read) < 8000u) {
    int r = c.read(buf, sizeof(buf));
    if (r <= 0) { delay(1); continue; }
    for (int i = 0; i < r; i++) if (buf[i] != (uint8_t)((got + i) * 7 + 3)) bad++;
    got += r;
  }
  const uint32_t t2 = millis();
  c.stop();
  say("tcp sent=%d got=%d bad=%d connect_ms=%lu total_ms=%lu remote=%s", sent, got, bad,
      (unsigned long)(t1 - t0), (unsigned long)(t2 - t0), host);
  if (sent == n && got == n && bad == 0) say("ok tcp"); else say("fail tcp mismatch");
}

static void http_get(const char *host, uint16_t port, const char *path)
{
  WiFiClient c;
  if (!c.connect(host, port)) { say("fail get connect"); return; }
  c.print("GET "); c.print(path); c.print(" HTTP/1.0\r\nHost: "); c.print(host);
  c.print("\r\nConnection: close\r\n\r\n");
  String status;
  bool in_body = false;
  long body = 0;
  String hdr;
  const uint32_t t_get = millis();
  while ((c.connected() || c.available()) && (millis() - t_get) < 10000u) {
    int ch = c.read();
    if (ch < 0) { delay(1); continue; }
    if (in_body) { body++; continue; }
    if (ch == '\n') {
      if (status.length() == 0) status = hdr;
      if (hdr.length() == 0 || hdr == "\r") in_body = true;
      hdr = "";
    } else {
      hdr += (char)ch;
    }
  }
  c.stop();
  status.trim();
  say("get status=\"%s\" body=%ld", status.c_str(), body);
  if (status.startsWith("HTTP/1.")) say("ok get"); else say("fail get");
}

static void handle(char *cmd)
{
  char *argv[8];
  int argc = 0;
  for (char *t = strtok(cmd, " "); t && argc < 8; t = strtok(nullptr, " ")) argv[argc++] = t;
  if (argc == 0) return;
  const char *c = argv[0];

  if (!strcmp(c, "ap") && argc >= 3) {
    const char *key = strcmp(argv[2], "-") ? argv[2] : nullptr;
    uint8_t ch = (argc >= 4) ? (uint8_t)atoi(argv[3]) : 6;
    const WifiSecurity sec = (argc >= 5) ? (WifiSecurity)atoi(argv[4]) : WIFI_SECURITY_WPA2;
    wl_status_t s = WiFi.beginAP(argv[1], key, ch, sec);
    say("ap status=%d ip=%s", (int)s, WiFi.localIP().toString().c_str());
    say((s == WL_AP_LISTENING || s == WL_AP_CONNECTED) ? "ok ap" : "fail ap");
  } else if (!strcmp(c, "sta") && argc >= 3) {
    const char *key = strcmp(argv[2], "-") ? argv[2] : nullptr;
    WiFi.setTimeout(argc >= 4 ? (uint32_t)atol(argv[3]) : 20000u);
    const WifiSecurity sec = (argc >= 5) ? (WifiSecurity)atoi(argv[4]) : WIFI_SECURITY_WPA2;
    wl_status_t s = WiFi.begin(argv[1], key, sec);
    say("sta status=%d ip=%s", (int)s, WiFi.localIP().toString().c_str());
    say((s == WL_CONNECTED) ? "ok sta" : "fail sta");
  } else if (!strcmp(c, "cfg") && argc >= 2) {
    IPAddress ip, dns, gw, mask(255, 255, 255, 0);
    if (!ip_arg(argv[1], ip)) { say("fail cfg ip"); return; }
    if (argc >= 5 && ip_arg(argv[2], dns) && ip_arg(argv[3], gw) && ip_arg(argv[4], mask)) WiFi.config(ip, dns, gw, mask);
    else if (argc >= 4 && ip_arg(argv[2], dns) && ip_arg(argv[3], gw)) WiFi.config(ip, dns, gw);
    else if (argc >= 3 && ip_arg(argv[2], dns)) WiFi.config(ip, dns);
    else WiFi.config(ip);
    say("ok cfg");
  } else if (!strcmp(c, "dns") && argc >= 2) {
    IPAddress a, b;
    if (!ip_arg(argv[1], a)) { say("fail dns"); return; }
    if (argc >= 3 && ip_arg(argv[2], b)) WiFi.setDNS(a, b); else WiFi.setDNS(a);
    say("ok dns");
  } else if (!strcmp(c, "host") && argc >= 2) {
    WiFi.setHostname(strcmp(argv[1], "-") ? argv[1] : "");
    say("host name=%s", WiFi.getHostname());
    say("ok host");
  } else if (!strcmp(c, "stat")) {
    stat();
    say("ok stat");
  } else if (!strcmp(c, "scan")) {
    const uint32_t t0 = millis();
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++) {
      uint8_t m[6];
      char b[18];
      WiFi.BSSID((uint8_t)i, m);
      say("net %d ssid=\"%s\" rssi=%ld enc=%u ch=%u bssid=%s", i, WiFi.SSID((uint8_t)i),
          (long)WiFi.RSSI((uint8_t)i), (unsigned)WiFi.encryptionType((uint8_t)i),
          (unsigned)WiFi.channel((uint8_t)i), mac_str(m, b));
    }
    say("scan n=%d ms=%lu status=%d", n, (unsigned long)(millis() - t0), (int)WiFi.status());
    say(n >= 0 ? "ok scan" : "fail scan");
  } else if (!strcmp(c, "disc")) {
    WiFi.disconnect();
    say("disc status=%d", (int)WiFi.status());
    say("ok disc");
  } else if (!strcmp(c, "end")) {
    WiFi.end();
    say("end status=%d", (int)WiFi.status());
    say("ok end");
  } else if (!strcmp(c, "lp") && argc >= 2) {
    bool r = atoi(argv[1]) ? WiFi.lowPowerMode() : WiFi.noLowPowerMode();
    say(r ? "ok lp" : "fail lp");
  } else if (!strcmp(c, "resolve") && argc >= 2) {
    IPAddress ip;
    const uint32_t t0 = millis();
    int r = WiFi.hostByName(argv[1], ip);
    say("resolve %s -> %s (%lu ms)", argv[1], ip.toString().c_str(), (unsigned long)(millis() - t0));
    say(r ? "ok resolve" : "fail resolve");
  } else if (!strcmp(c, "tcp") && argc >= 4) {
    tcp_echo(argv[1], (uint16_t)atoi(argv[2]), atoi(argv[3]));
  } else if (!strcmp(c, "get") && argc >= 4) {
    http_get(argv[1], (uint16_t)atoi(argv[2]), argv[3]);
  } else if (!strcmp(c, "srv") && argc >= 2) {
    delete g_srv;
    g_srv = new WiFiServer((uint16_t)atoi(argv[1]));
    g_srv->begin();
    say((g_srv->status() != 0) ? "ok srv" : "fail srv");
  } else if (!strcmp(c, "udp") && argc >= 2) {
    g_udp_on = g_udp.begin((uint16_t)atoi(argv[1])) != 0;
    say(g_udp_on ? "ok udp" : "fail udp");
  } else if (!strcmp(c, "usend") && argc >= 4) {
    WiFiUDP u;
    bool r = u.beginPacket(argv[1], (uint16_t)atoi(argv[2])) &&
             u.write((const uint8_t *)argv[3], strlen(argv[3])) == strlen(argv[3]) &&
             u.endPacket();
    say(r ? "ok usend" : "fail usend");
  } else if (!strcmp(c, "mcast") && argc >= 3) {
    IPAddress g;
    g_mc_on = ip_arg(argv[1], g) && g_mc.beginMulticast(g, (uint16_t)atoi(argv[2]));
    say(g_mc_on ? "ok mcast" : "fail mcast");
  } else if (!strcmp(c, "time") && argc >= 2) {
    WiFi.setTimeServer(argv[1]);
    const uint32_t t0 = millis();
    unsigned long t = WiFi.getTime();
    say("time epoch=%lu (%lu ms)", t, (unsigned long)(millis() - t0));
    say(t != 0 ? "ok time" : "fail time");
  } else if (!strcmp(c, "key") && argc >= 2) {
    say(WiFi.setLinkKey(strcmp(argv[1], "-") ? argv[1] : "") ? "ok key" : "fail key length");
  } else if (!strcmp(c, "stored")) {
    say("stored mode=%d ssid=\"%s\" hostname=%s", (int)WiFi.storedMode(), WiFi.storedSSID(),
        WiFi.getHostname());
    say("ok stored");
  } else if (!strcmp(c, "sbegin")) {
    WiFi.setTimeout(20000);
    wl_status_t s = WiFi.begin();
    say("sbegin status=%d ip=%s ssid=\"%s\"", (int)s, WiFi.localIP().toString().c_str(), WiFi.SSID());
    say((s == WL_CONNECTED || s == WL_AP_LISTENING || s == WL_AP_CONNECTED) ? "ok sbegin" : "fail sbegin");
  } else if (!strcmp(c, "asset")) {
    static char text[65];
    uint32_t len = 0, ver = 0, wlen = 0;
    bool user = n6_asset_present(&ver, &len);
    if (user) {
      uint8_t buf[N6_ASSET_MAX_BYTES];
      user = n6_asset_read(buf, sizeof(buf), &len, &ver);
      uint32_t i = 0;
      for (; user && i < len && i < sizeof(text) - 1; i++) text[i] = (buf[i] >= 32 && buf[i] < 127) ? (char)buf[i] : '.';
      text[i] = '\0';
    }
    const bool wifi = n6_asset_record_read(N6_ASSET_REC_WIFI, nullptr, 0, &wlen);
    uint32_t plen = 0;
    const bool pass = n6_asset_record_read(N6_ASSET_REC_AP_PASSWORD, nullptr, 0, &plen);
    say("asset user=%s len=%lu ver=%lu text=\"%s\" wifi=%s wlen=%lu appass=%s fsbl=\"%s\"",
        user ? "yes" : "no", (unsigned long)(user ? len : 0), (unsigned long)(user ? ver : 0),
        user ? text : "", wifi ? "yes" : "no", (unsigned long)wlen, pass ? "yes" : "no",
        n6_asset_result_str(n6_asset_last_result()));
    say("ok asset");
  } else if (!strcmp(c, "pass")) {
    uint32_t n = 0;
    if (!n6_asset_record_read(N6_ASSET_REC_AP_PASSWORD, nullptr, 0, &n)) {
      say("ok pass storing");   /* devicePassword() stores it and restarts the board */
      Serial.flush();
      delay(200);
    }
    const char *k = WiFi.devicePassword();
    say("pass key=%s len=%u", k, (unsigned)strlen(k));
    say(strlen(k) >= 8 ? "ok pass" : "fail pass");
  } else if (!strcmp(c, "wstart") && argc >= 2) {
    const int rc = MayaW476_Start(atoi(argv[1]));
    say("wstart rc=%d status=%d", rc, (int)WiFi.status());
    say(rc == 0 ? "ok wstart" : "fail wstart");
  } else if (!strcmp(c, "reboot")) {
    say("ok reboot");
    Serial.flush();
    delay(200);
    NVIC_SystemReset();
  } else if ((!strcmp(c, "awrite") && argc >= 2) || !strcmp(c, "aclear") || !strcmp(c, "pclear")) {
    const bool ok = (c[1] == 'w') ? n6_asset_stage(argv[1], strlen(argv[1]), 0)
                  : (c[0] == 'a') ? n6_asset_record_stage(N6_ASSET_REC_USER, nullptr, 0)
                                  : n6_asset_record_stage(N6_ASSET_REC_AP_PASSWORD, nullptr, 0);
    if (!ok) { say("fail %s staging refused", c); return; }
    say("ok %s restarting", c);
    Serial.flush();
    delay(200);
    n6_asset_apply();   /* does not return */
  } else {
    say("fail %s unknown", c);
  }
}

void setup()
{
  NeuroN6_StreamUsbOnlyStart();   // the USB serial console, with no camera
  Serial.begin(115200);
}

void loop()
{
  while (Serial.available() > 0) {
    int ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[line_len] = '\0';
      handle(line);
      line_len = 0;
    } else if (line_len < sizeof(line) - 1) {
      line[line_len++] = (char)ch;
    }
  }

  if (g_srv != nullptr) {
    WiFiClient c = g_srv->available();
    if (c) {
      uint8_t buf[1024];
      int n = c.read(buf, sizeof(buf));
      if (n > 0) c.write(buf, n);
    }
  }
  if (g_udp_on && g_udp.parsePacket() > 0) {
    uint8_t buf[WiFiUDP::kMaxDatagram];
    int n = g_udp.read(buf, sizeof(buf));
    if (n > 0 && g_udp.beginPacket(g_udp.remoteIP(), g_udp.remotePort())) {
      g_udp.write(buf, n);
      g_udp.endPacket();
    }
  }
  if (g_mc_on && g_mc.parsePacket() > 0) {
    char buf[128];
    int n = g_mc.read(buf, sizeof(buf) - 1);
    buf[n > 0 ? n : 0] = '\0';
    say("mcast from=%s len=%d text=\"%s\"", g_mc.remoteIP().toString().c_str(), n, buf);
  }

  static uint32_t beat = 0;
  if (millis() - beat > 5000) {
    beat = millis();
    say("beat status=%d ip=%s", (int)WiFi.status(), WiFi.localIP().toString().c_str());
  }
  delay(1);
}
