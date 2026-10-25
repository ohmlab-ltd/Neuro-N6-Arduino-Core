/*
  LTE thermal upload

  Uploads a camera picture, a thermal image and a status record to a web
  server every thirty seconds over an LTE modem.

  Hardware: Neuro N6 with a VD66GY camera on the Thermal carrier, a FLIR
  Lepton thermal camera and an MMC5603NJ magnetometer. SIMCom A7683E modem:
  TXD to D0, RXD to D1, and PWRKEY, RESET and NETLIGHT to the pins chosen
  below. A 5 V supply for the modem, 2 A in bursts, sharing ground with the
  board.

  This example code is in the public domain.
*/

#include <VD66GY_Arduino.h>
#include <Wire.h>
#include <MMC5603NJ.h>
#include <NeuroVisionThermal.h>
#include <A7683E.h>
#include <A7683EVideo.h>

// Settings

// Modem control pins. Any free header pin: D5, D6, D9 to D13, A0 to A5, or
// MOSI, MISO and SCK when SPI is unused. A7683E_NO_PIN for one not wired.
#define MODEM_PWRKEY_PIN    MISO
#define MODEM_RESET_PIN     D9
#define MODEM_NETLIGHT_PIN  D10

// The SIM's APN, user name and password, for example:
//   "internet"            O2 and Tesco   no user or password
//   "everywhere"          EE             eesecure and secure
//   "pp.vodafone.co.uk"   Vodafone       wap and wap
//   "uk.lebara.mobi"      Lebara         wap and wap
//   "TM"                  Three
#define MODEM_APN           "YOUR_APN"
#define MODEM_APN_USER      ""
#define MODEM_APN_PASS      ""

// The server, a name for this board and a token the server accepts. The
// upload is plain http and the token is in the address.
#define SERVER_BASE         "http://YOUR_SERVER_HOST"
#define DEVICE_ID           "neuro-n6-thermal-01"
#define SERVER_TOKEN        "CHANGE_ME"

#define UPLOAD_PERIOD_MS    30000
#define MODEM_BAUD          921600

#define CAMERA_MODE         CAMERA_VGA
#define AT_TRACE            false     // true prints every modem command

VD66GY             camera(VD66GY_CARRIER_THERMAL);
MMC5603NJ          mag;
NeuroVisionThermal thermal;

// One 160x120 thermal frame.
static uint16_t tframe[NVT_VOSPI_PIXELS];

// The same frame as a greyscale BMP: header, colour table, one byte per
// pixel.
#define BMP_HEADER_BYTES  (14 + 40 + 1024)
#define BMP_TOTAL_BYTES   (BMP_HEADER_BYTES + NVT_VOSPI_PIXELS)
static uint8_t tbmp[BMP_TOTAL_BYTES];

static bool magOK        = false;
static bool thermalOK    = false;
static bool cameraOK     = false;

// Modem bring-up, one step per pass of loop().
enum Stage { ST_POWER = 0, ST_SIM, ST_BAUD, ST_NETWORK, ST_DATA, ST_DRAIN, ST_RUN };
static Stage    g_stage       = ST_POWER;
static uint32_t g_lastNote    = 0;
static uint32_t g_lastUpload  = 0;
static uint32_t g_sent        = 0;
static uint32_t g_failed      = 0;
static uint32_t g_bytes       = 0;
static int      g_resets      = 0;

static float g_tMinC = 0, g_tMaxC = 0, g_tCentreC = 0;

static const char *stageName(Stage s) {
  switch (s) {
    case ST_POWER:   return "power  ";
    case ST_SIM:     return "sim    ";
    case ST_BAUD:    return "baud   ";
    case ST_NETWORK: return "network";
    case ST_DATA:    return "data   ";
    case ST_DRAIN:   return "drain  ";
    default:         return "run    ";
  }
}

static void note(const char *msg) {
  Serial.print('[');
  Serial.print(stageName(g_stage));
  Serial.print("] ");
  Serial.println(msg);
}

static bool shouldNote() {
  uint32_t now = millis();
  if ((now - g_lastNote) < 5000u) return false;
  g_lastNote = now;
  return true;
}

static void enter(Stage s) { g_stage = s; g_lastNote = 0; }

// Writes a thermal frame as a greyscale BMP, black to white from its coldest
// to its hottest pixel. Returns the file size in bytes.
static size_t thermalToBmp(const uint16_t *src, uint8_t *out) {
  uint16_t lo = 0xFFFF, hi = 0;
  for (int i = 0; i < NVT_VOSPI_PIXELS; i++) {
    if (src[i] < lo) lo = src[i];
    if (src[i] > hi) hi = src[i];
  }
  if (hi <= lo) hi = (uint16_t)(lo + 1);

  // Raw values are in 0.01 K.
  g_tMinC    = (lo / 100.0f) - 273.15f;
  g_tMaxC    = (hi / 100.0f) - 273.15f;
  g_tCentreC = (src[(NVT_VOSPI_HEIGHT / 2) * NVT_VOSPI_WIDTH + (NVT_VOSPI_WIDTH / 2)]
                / 100.0f) - 273.15f;

  memset(out, 0, BMP_HEADER_BYTES);
  const uint32_t total = BMP_TOTAL_BYTES;

  out[0] = 'B'; out[1] = 'M';
  out[2]  = (uint8_t)(total);       out[3]  = (uint8_t)(total >> 8);
  out[4]  = (uint8_t)(total >> 16); out[5]  = (uint8_t)(total >> 24);
  out[10] = (uint8_t)(BMP_HEADER_BYTES);
  out[11] = (uint8_t)(BMP_HEADER_BYTES >> 8);

  out[14] = 40;                                   // BITMAPINFOHEADER size
  out[18] = (uint8_t)(NVT_VOSPI_WIDTH);
  out[19] = (uint8_t)(NVT_VOSPI_WIDTH >> 8);
  // A negative height means the rows run top to bottom.
  int32_t negH = -(int32_t)NVT_VOSPI_HEIGHT;
  out[22] = (uint8_t)(negH);       out[23] = (uint8_t)(negH >> 8);
  out[24] = (uint8_t)(negH >> 16); out[25] = (uint8_t)(negH >> 24);
  out[26] = 1;                                    // planes
  out[28] = 8;                                    // bits per pixel
  out[34] = (uint8_t)(NVT_VOSPI_PIXELS);
  out[35] = (uint8_t)(NVT_VOSPI_PIXELS >> 8);
  out[36] = (uint8_t)(NVT_VOSPI_PIXELS >> 16);
  out[46] = 0; out[47] = 1;                       // 256 colours used

  uint8_t *pal = out + 54;
  for (int i = 0; i < 256; i++) {                 // grey colour table
    pal[i * 4 + 0] = (uint8_t)i;
    pal[i * 4 + 1] = (uint8_t)i;
    pal[i * 4 + 2] = (uint8_t)i;
    pal[i * 4 + 3] = 0;
  }

  uint8_t  *px    = out + BMP_HEADER_BYTES;
  uint32_t  range = (uint32_t)(hi - lo);
  for (int i = 0; i < NVT_VOSPI_PIXELS; i++) {
    px[i] = (uint8_t)(((uint32_t)(src[i] - lo) * 255u) / range);
  }
  return total;
}

// Posts one part. The board's name and token go in the address.
static bool postPart(const char *path, const char *contentType,
                     const uint8_t *body, size_t len) {
  char url[220];
  snprintf(url, sizeof(url), "%s%s?device=%s&token=%s",
           SERVER_BASE, path, DEVICE_ID, SERVER_TOKEN);

  if (!A7683E.httpBegin(url, contentType)) return false;
  bool ok = A7683E.httpPost(body, len);
  A7683E.httpEnd();
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\r\nA7683E thermal + RGB uploader");

  if (AT_TRACE) A7683E.setDebug(&Serial);

  // The thermal camera starts first. It fails if anything else talks on the
  // shared I2C bus while it starts.
  thermal.powerOn();
  thermal.setFlip180(true);
  thermalOK = thermal.videoBegin();
  Serial.print("thermal ...       ");
  Serial.println(thermalOK ? "OK" : "FAILED (VoSPI)");

  magOK = mag.begin();
  Serial.print("magnetometer ...  ");
  Serial.println(magOK ? "OK (temperature available)" : "NOT FOUND");

  // Starting the camera also keeps the serial console alive, so it comes
  // before the modem work. TRANSPORT_ETHERNET is the slot the modem library
  // uses, with no Ethernet hardware involved.
  VisionConfig cfg;
  cfg.camera      = CAMERA_MODE;
  cfg.mirror_flip = CAMERA_AUTO_FLIP;
  cfg.transport   = TRANSPORT_ETHERNET;
  cfg.lcd         = false;

  Serial.print("camera ...        ");
  Vision.begin(cfg);
  cameraOK = A7683EVideo.active();
  Serial.println(cameraOK ? "OK (frames routed to A7683EVideo)"
                          : "WARN - transport fell back, no frames will arrive");

  Serial.print("modem uart ...    ");
  bool uartOK = A7683E.begin(MODEM_PWRKEY_PIN, MODEM_RESET_PIN,
                             MODEM_NETLIGHT_PIN, 115200);
  if (!uartOK) {
    Serial.println("FAILED (UART4 init)");
  } else {
    Serial.print("OK  PWRKEY=");
    Serial.print(MODEM_PWRKEY_PIN);
    Serial.print(" RESET=");
    Serial.print(MODEM_RESET_PIN);
    Serial.print(" NETLIGHT=");
    Serial.print(MODEM_NETLIGHT_PIN);
    Serial.print("  clk=");
    Serial.println(A7683ESerial.kernelHz());
  }

  enter(ST_POWER);
}

static void runUpload() {
  const uint8_t *jpeg = NULL;
  uint32_t       jlen = 0;

  bool haveRgb = A7683EVideo.grab(&jpeg, &jlen, 50);

  // Every picture grabbed must be released, or the camera stops handing out
  // new ones.
  uint32_t now = millis();
  if ((now - g_lastUpload) < UPLOAD_PERIOD_MS) {
    if (haveRgb) A7683EVideo.release();
    return;
  }
  g_lastUpload = now;

  uint32_t t0 = millis();
  bool rgbOk = false, thmOk = false, telOk = false;

  if (haveRgb) {
    rgbOk = postPart("/api/v1/ingest/rgb", "image/jpeg", jpeg, jlen);
    g_bytes += jlen;
    A7683EVideo.release();
  }

  size_t blen = 0;
  if (thermalOK && thermal.readFrame(tframe, 900)) {
    blen  = thermalToBmp(tframe, tbmp);
    thmOk = postPart("/api/v1/ingest/thermal", "image/bmp", tbmp, blen);
    g_bytes += blen;
  }

  A7683ENetlight nl = A7683E.netlightState(1200);
  float magTempC = magOK ? mag.getTemp() : 0.0f;

  char body[400];
  int n = snprintf(body, sizeof(body),
    "{\"upload_ms\":%lu,\"lte_dbm\":%d,\"lte_csq\":%d,"
    "\"netlight\":\"%s\",\"netlight_period_ms\":%u,"
    "\"mag_temp_c\":%.1f,"
    "\"thermal_min_c\":%.1f,\"thermal_max_c\":%.1f,\"thermal_centre_c\":%.1f,"
    "\"rgb_bytes\":%lu,\"thermal_bytes\":%lu,"
    "\"resets\":%d,\"health\":\"%s\"}",
    (unsigned long)(millis() - t0),
    A7683E.signalDbm(), A7683E.signalQuality(),
    A7683EClass::netlightName(nl), (unsigned)A7683E.netlightPeriodMs(),
    (double)magTempC,
    (double)g_tMinC, (double)g_tMaxC, (double)g_tCentreC,
    (unsigned long)jlen, (unsigned long)blen,
    g_resets, (rgbOk && thmOk) ? "ok" : "partial");

  if (n > 0) {
    telOk = postPart("/api/v1/ingest/telemetry", "application/json",
                     (const uint8_t *)body, (size_t)n);
  }

  uint32_t elapsed = millis() - t0;
  if (rgbOk || thmOk) g_sent++; else g_failed++;

  Serial.print('[');
  Serial.print(g_sent + g_failed);
  Serial.print("] rgb=");
  Serial.print(rgbOk ? "ok" : (haveRgb ? "FAIL" : "none"));
  Serial.print(" thermal=");
  Serial.print(thmOk ? "ok" : (thermalOK ? "FAIL" : "none"));
  Serial.print(" telem=");
  Serial.print(telOk ? "ok" : "FAIL");
  Serial.print("  magT=");
  Serial.print(magTempC, 1);
  Serial.print("C  net=");
  Serial.print(A7683EClass::netlightName(nl));
  Serial.print("  ");
  Serial.print(elapsed);
  Serial.print(" ms  total ");
  Serial.print(g_bytes / 1048576.0f, 2);
  Serial.println(" MB");

  if (!rgbOk && !thmOk && !telOk) {
    if ((g_failed % 4) == 0 && A7683E.resetPin() >= 0) {
      Serial.println("       repeated failures - hardware RESET");
      g_resets++;
      A7683E.reset();
      enter(ST_SIM);
    } else if ((g_failed % 2) == 0) {
      Serial.println("       rebuilding PDP context");
      enter(ST_DATA);
    }
  }
}

void loop() {
  char buf[110];

  switch (g_stage) {

    // PWRKEY is a toggle: a pulse turns a running modem off.
    case ST_POWER: {
      if (A7683E.powerOn()) {
        snprintf(buf, sizeof(buf), "modem answering AT at %lu baud",
                 (unsigned long)A7683E.baud());
        note(buf);
        enter(ST_SIM);
      } else if (shouldNote()) {
        snprintf(buf, sizeof(buf), "no AT reply - rx=%lu drop=%lu netlight=%s",
                 (unsigned long)A7683ESerial.rxTotal(),
                 (unsigned long)A7683ESerial.rxDropped(),
                 A7683EClass::netlightName(A7683E.netlightState(800)));
        note(buf);
      }
      break;
    }

    case ST_SIM:
      if (!A7683E.isAlive(1000)) { enter(ST_POWER); break; }
      A7683E.sendAT("ATE0", 2000);
      if (A7683E.simReady(2000)) {
        note("READY");
        enter(ST_BAUD);
      } else if (shouldNote()) {
        note("no SIM - missing, upside down, or PIN-locked");
      }
      break;

    case ST_BAUD:
      if (A7683E.syncBaud(MODEM_BAUD)) {
        snprintf(buf, sizeof(buf), "%d OK", MODEM_BAUD);
        note(buf);
      } else {
        note("could not switch - staying slow, uploads will be sluggish");
      }
      enter(ST_NETWORK);
      break;

    case ST_NETWORK:
      if (A7683E.isRegistered()) {
        char op[40];
        if (A7683E.getOperator(op, sizeof(op))) {
          snprintf(buf, sizeof(buf), "registered - %s, %d dBm, netlight %s",
                   op, A7683E.signalDbm(),
                   A7683EClass::netlightName(A7683E.netlightState(1200)));
        } else {
          snprintf(buf, sizeof(buf), "registered - %d dBm", A7683E.signalDbm());
        }
        note(buf);
        enter(ST_DATA);
      } else if (shouldNote()) {
        snprintf(buf, sizeof(buf), "searching - CSQ %d, CREG %d, netlight %s",
                 A7683E.signalQuality(), A7683E.registrationStatus(),
                 A7683EClass::netlightName(A7683E.netlightState(1200)));
        note(buf);
      }
      break;

    case ST_DATA:
      if (A7683E.attachData(MODEM_APN, MODEM_APN_USER, MODEM_APN_PASS, 8000)) {
        char ip[24];
        if (A7683E.getLocalIP(ip, sizeof(ip))) {
          snprintf(buf, sizeof(buf), "PDP up, IP %s", ip);
          note(buf);
        } else {
          note("PDP up");
        }
        enter(ST_DRAIN);
      } else if (shouldNote()) {
        note("no address yet - APN " MODEM_APN " / " MODEM_APN_USER " correct?");
      }
      break;

    case ST_DRAIN: {
      const uint8_t *j; uint32_t l;
      int binned = 0;
      while (A7683EVideo.grab(&j, &l, 0)) { A7683EVideo.release(); binned++; }
      snprintf(buf, sizeof(buf), "discarded %d stale frame(s) - uploading", binned);
      note(buf);
      g_lastUpload = millis() - UPLOAD_PERIOD_MS;   // upload immediately
      enter(ST_RUN);
      break;
    }

    case ST_RUN:
      runUpload();
      break;
  }
}
