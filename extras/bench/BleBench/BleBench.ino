/*
  BleBench

  A command shell on the USB serial console for checking BLE, driven by
  tools/bench/n6bleapi.py. One command per line, and each ends with
  "[ble] ok <cmd>" or "[ble] fail <cmd> <why>".

    adv NAME              advertise the bench service as NAME, pairable
    stopadv               stop advertising
    disc                  drop every connection
    busy SECONDS          block loop() that long
    notify 1|0            count up the notify value every 500 ms, or stop
    scan SECONDS          scan as a central and list what it hears
    central UUID          find a device advertising UUID, connect, read and
                          write its bench value, disconnect
    stat                  address and connection
    end                   BLE.end()
    wifiap SSID KEY       start a WiFi access point
    wifiend               WiFi.end()
    debug 1|0             every HCI packet on the console, or not

  Hardware: Neuro N6 with the MAYA-W476 wireless module.

  This example code is in the public domain.
*/

#include <NeuroN6.h>
#include <ArduinoBLE.h>
#include <MayaW476.h>
#include <stdarg.h>

BLEService                   svc       ("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEByteCharacteristic        ctrlChar  ("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEUnsignedIntCharacteristic countChar ("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEUnsignedIntCharacteristic secretChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLEEncryption);

// The bench value on the host's GATT server, from tools/bench/n6radio.
static const char *kPcValue = "6e360002-b5a3-f393-e0a9-e50e24dcca9e";

static bool     g_begun = false;
static bool     g_svc_added = false;   // BLE.end() empties the GATT database
static bool     g_notify = false;
static uint32_t g_count = 0;

// One bond, kept in RAM.
static uint8_t bond_mac[6], bond_irk[16], bond_ltk[16];
static bool    has_irk = false, has_ltk = false;

static char line[160];
static size_t line_len = 0;

static void say(const char *fmt, ...)
{
  char buf[200];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print("[ble] ");
  Serial.println(buf);
}

static bool begin_ble()
{
  if (g_begun) return true;
  g_begun = BLE.begin() != 0;
  if (!g_begun) return false;

  BLE.setDisplayCode([](uint32_t code) { say("pair code %06lu", (unsigned long)code); });
  BLE.setBinaryConfirmPairing([]() { say("pair confirm: yes"); return true; });
  BLE.setStoreIRK([](uint8_t *addr, uint8_t *irk) {
    memcpy(bond_mac, addr, 6); memcpy(bond_irk, irk, 16); has_irk = true;
    say("bond irk stored"); return 1;
  });
  BLE.setStoreLTK([](uint8_t *addr, uint8_t *ltk) {
    memcpy(bond_mac, addr, 6); memcpy(bond_ltk, ltk, 16); has_ltk = true;
    say("bond ltk stored"); return 1;
  });
  BLE.setGetIRKs([](uint8_t *n, uint8_t **types, uint8_t ***addrs, uint8_t ***irks) {
    *n = has_irk ? 1 : 0;
    *addrs = new uint8_t*[1];
    *irks  = new uint8_t*[1];
    *types = new uint8_t[1];
    (*types)[0] = 0;
    (*addrs)[0] = new uint8_t[6];
    (*irks)[0]  = new uint8_t[16];
    memcpy((*addrs)[0], bond_mac, 6);
    memcpy((*irks)[0], bond_irk, 16);
    return 1;
  });
  BLE.setGetLTK([](uint8_t *addr, uint8_t *ltk) {
    if (!has_ltk || memcmp(addr, bond_mac, 6) != 0) return 0;
    memcpy(ltk, bond_ltk, 16);
    return 1;
  });
  BLE.setEventHandler(BLEConnected, [](BLEDevice c) { say("connected %s", c.address().c_str()); });
  BLE.setEventHandler(BLEDisconnected, [](BLEDevice c) { say("disconnected %s", c.address().c_str()); });
  ctrlChar.setEventHandler(BLEWritten, [](BLEDevice, BLECharacteristic) {
    say("ctrl written 0x%02X", (unsigned)ctrlChar.value());
  });
  return true;
}

static void cmd_adv(const char *name)
{
  if (!begin_ble()) { say("fail adv begin"); return; }
  if (!g_svc_added) {
    svc.addCharacteristic(ctrlChar);
    svc.addCharacteristic(countChar);
    svc.addCharacteristic(secretChar);
    BLE.addService(svc);
    ctrlChar.writeValue(0);
    countChar.writeValue(0);
    secretChar.writeValue(0x4E365345);    // any constant
    g_svc_added = true;
  }
  BLE.setLocalName(name);
  BLE.setDeviceName(name);
  BLE.setAdvertisedService(svc);
  BLE.setConnectable(true);
  BLE.setPairable(Pairable::YES);
  say("adv address=%s", BLE.address().c_str());
  say(BLE.advertise() ? "ok adv" : "fail adv advertise");
}

static void cmd_scan(int seconds)
{
  if (!begin_ble()) { say("fail scan begin"); return; }
  BLE.stopAdvertise();
  BLE.scan(true);
  const uint32_t end = millis() + (uint32_t)seconds * 1000u;
  int heard = 0;
  while ((int32_t)(end - millis()) > 0) {
    BLEDevice d = BLE.available();
    if (!d) { delay(5); continue; }
    char mfg[40] = "";
    if (d.hasManufacturerData()) {
      uint8_t m[16];
      int n = d.manufacturerData(m, sizeof(m));
      for (int i = 0; i < n && i < 16; i++) snprintf(mfg + 2 * i, 3, "%02x", m[i]);
    }
    say("heard %s rssi=%d name=\"%s\" uuid=%s mfg=%s", d.address().c_str(), d.rssi(),
        d.hasLocalName() ? d.localName().c_str() : "",
        d.hasAdvertisedServiceUuid() ? d.advertisedServiceUuid().c_str() : "-", mfg[0] ? mfg : "-");
    heard++;
  }
  BLE.stopScan();
  say("scan heard=%d", heard);
  say("ok scan");
}

static void cmd_central(const char *uuid)
{
  if (!begin_ble()) { say("fail central begin"); return; }
  BLE.stopAdvertise();
  BLE.scanForUuid(uuid);
  BLEDevice p;
  const uint32_t end = millis() + 15000u;
  while (!p && (int32_t)(end - millis()) > 0) { p = BLE.available(); if (!p) delay(10); }
  BLE.stopScan();
  if (!p) { say("fail central not found"); return; }
  say("central found %s", p.address().c_str());
  if (!p.connect()) { say("fail central connect"); return; }
  if (!p.discoverAttributes()) { say("fail central discover"); p.disconnect(); return; }
  BLECharacteristic c = p.characteristic(kPcValue);
  if (!c) { say("fail central no bench value"); p.disconnect(); return; }
  uint8_t buf[32] = {0};
  bool rd = c.readValue(buf, sizeof(buf) - 1) > 0;
  say("central read \"%s\"", (const char *)buf);
  const char *msg = "board-was-here";
  bool wr = c.writeValue((const uint8_t *)msg, strlen(msg), true) != 0;
  p.disconnect();
  say((rd && wr) ? "ok central" : "fail central readwrite");
}

static void handle(char *cmd)
{
  char *argv[4];
  int argc = 0;
  for (char *t = strtok(cmd, " "); t && argc < 4; t = strtok(nullptr, " ")) argv[argc++] = t;
  if (argc == 0) return;
  const char *c = argv[0];

  if (!strcmp(c, "adv") && argc >= 2) {
    cmd_adv(argv[1]);
  } else if (!strcmp(c, "stopadv")) {
    BLE.stopAdvertise();
    say("ok stopadv");
  } else if (!strcmp(c, "disc")) {
    BLE.disconnect();
    say("ok disc");
  } else if (!strcmp(c, "busy") && argc >= 2) {
    delay((uint32_t)atoi(argv[1]) * 1000u);      // no BLE.poll() here
    say("ok busy");
  } else if (!strcmp(c, "notify") && argc >= 2) {
    g_notify = atoi(argv[1]) != 0;
    say("ok notify");
  } else if (!strcmp(c, "scan") && argc >= 2) {
    cmd_scan(atoi(argv[1]));
  } else if (!strcmp(c, "central") && argc >= 2) {
    cmd_central(argv[1]);
  } else if (!strcmp(c, "stat")) {
    say("stat begun=%d address=%s connected=%d paired=%d wifi=%d", (int)g_begun,
        g_begun ? BLE.address().c_str() : "-", g_begun ? (int)BLE.connected() : 0,
        g_begun ? (int)BLE.paired() : 0, (int)WiFi.status());
    say("ok stat");
  } else if (!strcmp(c, "end")) {
    BLE.end();
    g_begun = false;
    g_svc_added = false;
    say("ok end");
  } else if (!strcmp(c, "debug") && argc >= 2) {
    if (atoi(argv[1])) BLE.debug(Serial); else BLE.noDebug();
    say("ok debug");
  } else if (!strcmp(c, "wifiap") && argc >= 3) {
    wl_status_t s = WiFi.beginAP(argv[1], argv[2]);
    say("wifiap status=%d ip=%s", (int)s, WiFi.localIP().toString().c_str());
    say((s == WL_AP_LISTENING || s == WL_AP_CONNECTED) ? "ok wifiap" : "fail wifiap");
  } else if (!strcmp(c, "wifiend")) {
    WiFi.end();
    say("wifiend status=%d", (int)WiFi.status());
    say(WiFi.status() == WL_DISCONNECTED ? "ok wifiend" : "fail wifiend");
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
  static uint32_t last = 0;
  if (g_notify && millis() - last >= 500) {
    last = millis();
    countChar.writeValue(++g_count);
  }
  delay(2);
}
