/*
 * SdCardWriteBenchmark
 *
 * Times card bring-up and the write of an image, a Lepton radiometric frame
 * and a telemetry packet into their own folder, at each SPI clock of a sweep.
 * Every file is then read back and byte-compared, outside the timed region,
 * and its verify line prints OK or FAIL.
 *
 * Hardware: Neuro N6 with a VD66GY camera on the thermal carrier and an SD
 * module on the header SPI, SPI2:
 *   MOSI -> D22 (PD7)      MISO -> D23 (PD11)
 *   SCK  -> D24 (PF2)      CS   -> A5  (PG15)
 *   VCC 3V3, GND common.
 *
 * Dependency: SdFat by Bill Greiman (MIT), not vendored in this core:
 *   arduino-cli lib install SdFat
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

/* 1 = write a VGA JPEG captured from the VD66GY. 0 = write a synthetic
 * 921,600 B raw frame, with no camera. */
#define USE_REAL_JPEG    1

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#if USE_REAL_JPEG
/* The core has no single-still call. EdgeViewFrames claims the Ethernet
 * transport slot and drains its queue of encoded frames. */
#include <VD66GY_Arduino.h>
#include "EdgeViewFrames.h"
#include "NeuroN6_FrameQueue.h"
VD66GY camera(VD66GY_CARRIER_THERMAL);
#endif

/* Settings */
#define SD_CS_PIN        A5                      /* PG15 */
#define RGB_W            640
#define RGB_H            480
#define RGB_BPP          3                       /* RGB888 */
#define THERMAL_PIXELS   19200                   /* Lepton 160x120 */
#define TELEM_BYTES      512
#define RUNS             3                       /* repeats per clock */
static const uint32_t kClocksMHz[] = { 32, 16, 8, 4 };   /* HSI kernel /2../16 */

#define RGB_BYTES     ((uint32_t)RGB_W * RGB_H * RGB_BPP)
#define THERM_BYTES   ((uint32_t)THERMAL_PIXELS * 2)

static SdFs   sd;
static FsFile file;

static uint8_t *rgbBuf, *thermBuf, telemBuf[TELEM_BYTES];
/* Set when a JPEG was captured. The image file is then the JPEG. */
static uint8_t *jpegBuf = nullptr;
static uint32_t jpegLen = 0;
/* A private copy: the core's buffer is valid only between grab() and
 * release(). */
static uint8_t *jpegStore = nullptr;
/* Camera and JPEG outcome, printed on every pass. */
static const char *s_camMsg = "not attempted";
static uint32_t    s_camMs  = 0;
/* Hantro encoder diagnostics from the core:
 *  -3 = INVALID_ARG, -6 = OUTBUF_OVERFLOW, -15 = INSTANCE_ERR/HW_RESET. */
extern "C" volatile int32_t g_dbg_lb_enc_rv;     /* line-buffer JpegEncEncode return */
extern "C" volatile int32_t g_dbg_ff_init_ret;   /* JpegEncInit return */
extern "C" volatile int32_t g_dbg_ff_jesps;      /* SetPictureSize return */
static int32_t s_encRv = 0, s_encIni = 0, s_encSps = 0;
static uint8_t  vbuf[8192];                 /* read-back compare window */
static uint32_t s_folder = 0;

/* malloc with 32-byte alignment. The block is never freed. */
static uint8_t *allocAligned(uint32_t bytes)
{
  void *raw = malloc(bytes + 32u);
  if (raw == nullptr) return nullptr;
  return (uint8_t *)(((uintptr_t)raw + 31u) & ~(uintptr_t)31u);
}

/* Returns the microseconds taken to create, preallocate, write and close. */
static uint32_t writeFile(const char *path, const uint8_t *buf, uint32_t len, bool &ok)
{
  const uint32_t t0 = micros();
  ok = false;
  if (!file.open(path, O_WRONLY | O_CREAT | O_TRUNC)) return micros() - t0;
  file.preAllocate(len);
  const int32_t n = file.write(buf, len);
  file.close();                       /* flushes the directory entry */
  const uint32_t us = micros() - t0;
  ok = (n == (int32_t)len);
  return us;
}

/* Returns 0 = verified, 1 = open failed, 2 = size differs, 3 = data differs,
 * 4 = short read. detail carries the size or the offset. */
static int verifyFile(const char *path, const uint8_t *expect, uint32_t len, uint32_t &detail)
{
  FsFile f;
  detail = 0;
  if (!f.open(path, O_RDONLY)) return 1;
  if (f.fileSize() != (uint64_t)len) { detail = (uint32_t)f.fileSize(); f.close(); return 2; }

  uint32_t off = 0;
  while (off < len) {
    uint32_t want = len - off;
    if (want > sizeof(vbuf)) want = sizeof(vbuf);
    const int32_t got = f.read(vbuf, want);
    if (got != (int32_t)want) { detail = off; f.close(); return 4; }
    for (uint32_t i = 0; i < want; i++) {
      if (vbuf[i] != expect[off + i]) { detail = off + i; f.close(); return 3; }
    }
    off += want;
  }
  f.close();
  return 0;
}

static void reportVerify(const char *what, int rc, uint32_t detail, uint32_t len)
{
  Serial.print("      verify ");
  Serial.print(what);
  switch (rc) {
    case 0: Serial.print("  OK, ");   Serial.print(len); Serial.println(" B match"); break;
    case 1: Serial.println("  FAIL - file not found on the card");                    break;
    case 2: Serial.print("  FAIL - size is ");  Serial.print(detail);
            Serial.print(", expected ");        Serial.println(len);                  break;
    case 3: Serial.print("  FAIL - data differs at byte "); Serial.println(detail);    break;
    default:Serial.print("  FAIL - short read at byte ");   Serial.println(detail);    break;
  }
}

static void reportTime(const char *what, uint32_t us, uint32_t bytes, bool ok)
{
  Serial.print("      ");
  Serial.print(what);
  Serial.print("  ");
  Serial.print(us / 1000.0f, 1);
  Serial.print(" ms");
  if (bytes >= 4096u && us > 0u) {
    Serial.print("   ");
    Serial.print((float)bytes / (1.048576f * (float)us), 2);
    Serial.print(" MB/s   ");
    Serial.print((float)bytes * 1000.0f / (1.024f * (float)us), 0);
    Serial.print(" kB/s");
  }
  if (!ok) Serial.print("   <-- WRITE REPORTED FAILURE");
  Serial.println();
}

/* Drives the card by hand and prints the raw bytes. */
static void busProbe()
{
  Serial.println("\n[sd] ---- bus probe ----");

  /* The prescaler divides the kernel clock by 2 to 256. */
  const uint32_t kern = (uint32_t)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2);
  Serial.print("    SPI2 kernel (RCC says) ");
  Serial.print(kern / 1000000.0f, 2);
  Serial.print(" MHz");
  if (kern == 0u) {
    Serial.println("   <-- RCC returned 0; library falls back to PCLK1");
  } else {
    Serial.print(" -> SCK range ");  Serial.print(kern / 256u / 1000.0f, 1);
    Serial.print(" kHz .. ");        Serial.print(kern / 2u / 1000000.0f, 1);
    Serial.println(" MHz");
  }
  Serial.print("    PCLK1 ");
  Serial.print(HAL_RCC_GetPCLK1Freq() / 1000000.0f, 1);
  Serial.println(" MHz");

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, LOW);
  const int csLow = digitalRead(SD_CS_PIN);
  digitalWrite(SD_CS_PIN, HIGH);
  const int csHigh = digitalRead(SD_CS_PIN);
  Serial.print("    CS pin drives: low->");   Serial.print(csLow);
  Serial.print("  high->");                   Serial.println(csHigh);
  if (csLow != 0 || csHigh != 1) {
    Serial.println("    -> CS IS NOT MOVING. The pin is stuck; nothing can work.");
  }

  /* The adapter's pull-ups of about 10k beat the internal pulldown of about
   * 40k, so PD11 reads high when the MISO wire is attached. */
  {
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOD_CLK_ENABLE();
    g.Pin = GPIO_PIN_11; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOD, &g);
    delayMicroseconds(50);
    const int misoWire = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_SET);
    Serial.print("    MISO wire attached (pulldown test): ");
    Serial.println(misoWire ? "YES (external pull-up wins)" : "NO - wire loose/absent");
    g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH; g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOD, &g);
  }

  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));

  /* CS is high, so no card drives MISO: 0xFF back is the pull-up alone, and
   * the same byte back means MOSI and MISO are joined. */
  const uint8_t echo1 = SPI.transfer(0x5A);
  const uint8_t echo2 = SPI.transfer(0xA5);
  Serial.print("    loopback probe: sent 5A A5, got ");
  Serial.print(echo1, HEX); Serial.print(" "); Serial.println(echo2, HEX);
  if (echo1 == 0x5Au && echo2 == 0xA5u) {
    Serial.println("    -> MOSI is looped to MISO. Bus works; remove the jumper.");
  }

  /* The card's power-up sequence is CS high and at least 74 clocks. */
  uint8_t idle[10];
  for (uint32_t i = 0; i < 10u; i++) idle[i] = SPI.transfer(0xFF);
  Serial.print("    MISO idle bytes:");
  for (uint32_t i = 0; i < 10u; i++) { Serial.print(" 0x"); Serial.print(idle[i], HEX); }
  Serial.println();
  if (idle[0] == 0x00u && idle[9] == 0x00u) {
    Serial.println("    -> STUCK LOW. MISO is not reaching D23/PD11, or the");
    Serial.println("       module is unpowered. Nothing else can work.");
  } else if (idle[0] == 0xFFu && idle[9] == 0xFFu) {
    Serial.println("    -> idles high, as it should (card may still be mute)");
  }

  SPI.endTransaction();

  /* CMD0 in all four SPI modes. A mode that returns 0x01 identifies the
   * framing. */
  uint8_t r1 = 0xFFu;
  for (uint32_t m = 0; m < 4u; m++) {
    SPI.beginTransaction(SPISettings(400000, MSBFIRST, m));
    uint8_t rr = 0xFFu;
    for (uint32_t attempt = 0; attempt < 3u && rr != 0x01u; attempt++) {
      digitalWrite(SD_CS_PIN, HIGH);
      for (uint32_t i = 0; i < 12u; i++) SPI.transfer(0xFF);
      digitalWrite(SD_CS_PIN, LOW);
      static const uint8_t cmd0[] = { 0x40, 0, 0, 0, 0, 0x95 };
      for (uint32_t i = 0; i < sizeof(cmd0); i++) SPI.transfer(cmd0[i]);
      rr = 0xFFu;
      for (uint32_t i = 0; i < 16u && rr == 0xFFu; i++) rr = SPI.transfer(0xFF);
      digitalWrite(SD_CS_PIN, HIGH);
      SPI.transfer(0xFF);
    }
    SPI.endTransaction();
    Serial.print("    CMD0 via SPI_MODE"); Serial.print(m);
    Serial.print(" -> 0x");                Serial.println(rr, HEX);
    if (rr == 0x01u && r1 != 0x01u) r1 = rr;
  }

  if (r1 == 0x01u) {
    Serial.println("    -> CARD IS ALIVE (idle) over hardware SPI.");
  } else if (r1 == 0xFFu) {
    Serial.println("    -> silent. MISO idles high (so it IS connected and");
    Serial.println("       pulled up) but the card never sees a command, which");
    Serial.println("       points at MOSI (D22/PD7), SCK (D24/PF2) or the CS");
    Serial.println("       wire not reaching the card - not at MISO or power.");
    Serial.println("       Jumper MOSI to MISO and re-run: the loopback probe");
    Serial.println("       above will then echo 5A A5 and prove the bus.");
  } else {
    Serial.println("    -> answered, but not 0x01. Card present and clocking;");
    Serial.println("       a timing or level problem rather than wiring.");
  }
}

/* Bit-banged CMD0 on the same pins at about 50 kHz, with no SPI peripheral. */
#define BB_MOSI 22
#define BB_MISO 23
#define BB_SCK  24

static uint8_t bbXfer(uint8_t v)
{
  uint8_t r = 0;
  for (int b = 7; b >= 0; b--) {
    digitalWrite(BB_MOSI, (v >> b) & 1u);
    delayMicroseconds(10);
    digitalWrite(BB_SCK, HIGH);            /* card samples on the rise */
    delayMicroseconds(10);
    r = (uint8_t)((r << 1) | (digitalRead(BB_MISO) ? 1u : 0u));
    digitalWrite(BB_SCK, LOW);
  }
  return r;
}

static void bitbangProbe()
{
  Serial.println("    -- bit-bang probe (GPIO only, ~50 kHz) --");

  pinMode(BB_MOSI, OUTPUT);  digitalWrite(BB_MOSI, HIGH);
  pinMode(BB_SCK,  OUTPUT);  digitalWrite(BB_SCK,  LOW);   /* mode 0 idle */
  pinMode(BB_MISO, INPUT_PULLUP);
  pinMode(SD_CS_PIN, OUTPUT); digitalWrite(SD_CS_PIN, HIGH);
  delay(2);

  for (uint32_t i = 0; i < 10u; i++) (void)bbXfer(0xFF);   /* 80 init clocks */

  uint8_t r1 = 0xFF;
  Serial.print("    bit-bang CMD0:");
  for (uint32_t a = 0; a < 4u && r1 != 0x01u; a++) {
    digitalWrite(SD_CS_PIN, LOW);
    static const uint8_t c0[] = { 0x40, 0, 0, 0, 0, 0x95 };
    for (uint32_t i = 0; i < sizeof(c0); i++) (void)bbXfer(c0[i]);
    r1 = 0xFF;
    for (uint32_t i = 0; i < 16u && r1 == 0xFFu; i++) r1 = bbXfer(0xFF);
    digitalWrite(SD_CS_PIN, HIGH);
    (void)bbXfer(0xFF);
    Serial.print(" 0x"); Serial.print(r1, HEX);
  }
  Serial.println();

  if (r1 == 0x01u) {
    Serial.println("    -> ALIVE VIA BIT-BANG. Wiring is fully good; the SPI2");
    Serial.println("       path (781 kHz init floor or pin mux) is the fault.");
  } else {
    Serial.println("    -> silent even on hand-driven 50 kHz clocks. The MCU");
    Serial.println("       side is exonerated: SCK wire, CS at the card, or");
    Serial.println("       the card itself. Try another card.");
  }

  /* The pins go back to SPI2 for the sweep. */
  GPIO_InitTypeDef g = {0};
  g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH; g.Alternate = GPIO_AF5_SPI2;
  g.Pin = GPIO_PIN_7;  HAL_GPIO_Init(GPIOD, &g);   /* MOSI PD7  */
  g.Pin = GPIO_PIN_11; HAL_GPIO_Init(GPIOD, &g);   /* MISO PD11 */
  g.Pin = GPIO_PIN_2;  HAL_GPIO_Init(GPIOF, &g);   /* SCK  PF2  */
}

/* Wiring tester. With a jumper from MISO to another wire at the adapter end,
 * MISO follows both levels driven on that wire. */
static bool followsMiso(uint32_t drivePin)
{
  pinMode(drivePin, OUTPUT);
  pinMode(BB_MISO, INPUT_PULLUP);
  digitalWrite(drivePin, LOW);
  delayMicroseconds(200);
  const int lo = digitalRead(BB_MISO);
  digitalWrite(drivePin, HIGH);
  delayMicroseconds(200);
  const int hi = digitalRead(BB_MISO);
  return (lo == 0 && hi == 1);
}

static void wiringTester()
{
  Serial.println("    -- wiring tester (jumper MISO to one wire AT THE ADAPTER) --");
  const bool mosi = followsMiso(BB_MOSI);
  const bool sck  = followsMiso(BB_SCK);
  const bool cs   = followsMiso(SD_CS_PIN);
  Serial.print("    MISO follows:  MOSI=");  Serial.print(mosi ? "YES" : "no");
  Serial.print("   SCK=");                   Serial.print(sck  ? "YES" : "no");
  Serial.print("   CS=");                    Serial.println(cs ? "YES" : "no");
  if (!mosi && !sck && !cs) {
    Serial.println("    (no jumper fitted - fit one at the ADAPTER end to test");
    Serial.println("     that wire end-to-end; a good wire shows YES)");
  }
  pinMode(SD_CS_PIN, OUTPUT); digitalWrite(SD_CS_PIN, HIGH);
}

static bool runAtClock(uint32_t mhz)
{
  Serial.print("\n[sd] ================ SPI ");
  Serial.print(mhz);
  Serial.println(" MHz ================");

  /* DEDICATED_SPI: nothing else shares this bus. */
  SdSpiConfig cfg(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(mhz), &SPI);

  const uint32_t tUp = micros();
  const bool up = sd.begin(cfg);
  const uint32_t upUs = micros() - tUp;

  Serial.print("    BRING-UP  ");
  Serial.print(upUs / 1000.0f, 1);
  Serial.println(up ? " ms" : " ms   <-- card init FAILED at this clock");
  if (!up) return false;

  for (uint32_t run = 0; run < RUNS; run++) {
    char dir[24];
    do {
      snprintf(dir, sizeof(dir), "/CAP%05lu", (unsigned long)(++s_folder));
    } while (sd.exists(dir) && s_folder < 99999u);

    const uint32_t tMk = micros();
    const bool mkOk = sd.mkdir(dir);
    const uint32_t mkUs = micros() - tMk;

    /* With USE_REAL_JPEG, rgbBuf is a 4 KB stub, so the fallback length is
     * 4096 rather than RGB_BYTES. */
    const uint8_t *imgBuf = jpegBuf ? jpegBuf : rgbBuf;
    const uint32_t imgLen = jpegBuf ? jpegLen
                                    : (USE_REAL_JPEG ? 4096u : RGB_BYTES);

    char pRgb[40], pTh[40], pTe[40];
    snprintf(pRgb, sizeof(pRgb), jpegBuf ? "%s/image.jpg" : "%s/rgb.raw", dir);
    snprintf(pTh,  sizeof(pTh),  "%s/thermal.raw", dir);
    snprintf(pTe,  sizeof(pTe),  "%s/telem.bin", dir);

    bool ok1, ok2, ok3;
    const uint32_t us1 = writeFile(pRgb, imgBuf,   imgLen,      ok1);
    const uint32_t us2 = writeFile(pTh,  thermBuf, THERM_BYTES, ok2);
    const uint32_t us3 = writeFile(pTe,  telemBuf, TELEM_BYTES, ok3);

    Serial.print("    run ");
    Serial.print(run + 1);
    Serial.print("  ");
    Serial.println(dir);
    reportTime("mkdir  ", mkUs, 0, mkOk);
    reportTime(jpegBuf ? "jpeg   " : "rgb    ", us1, imgLen, ok1);
    reportTime("thermal", us2, THERM_BYTES, ok2);
    reportTime("telem  ", us3, TELEM_BYTES, ok3);
    reportTime("TOTAL  ", mkUs + us1 + us2 + us3,
               imgLen + THERM_BYTES + TELEM_BYTES, mkOk && ok1 && ok2 && ok3);

    uint32_t d;
    int rc;
    rc = verifyFile(pRgb, imgBuf,   imgLen,      d);
    reportVerify(jpegBuf ? "jpeg   " : "rgb    ", rc, d, imgLen);
    rc = verifyFile(pTh,  thermBuf, THERM_BYTES, d); reportVerify("thermal", rc, d, THERM_BYTES);
    rc = verifyFile(pTe,  telemBuf, TELEM_BYTES, d); reportVerify("telem  ", rc, d, TELEM_BYTES);
  }
  sd.end();
  return true;
}

void setup()
{
  Serial.begin(115200);
  uint32_t tw = millis();
  while (!Serial && (millis() - tw) < 3000u) { delay(10); }

  Serial.println();
  Serial.println("[sd] ===== SD write benchmark + read-back verify =====");
  Serial.print("[sd] per capture: rgb ");  Serial.print(RGB_BYTES);
  Serial.print(" B + thermal ");           Serial.print(THERM_BYTES);
  Serial.print(" B + telem ");             Serial.print(TELEM_BYTES);
  Serial.print(" B = ");
  Serial.print(RGB_BYTES + THERM_BYTES + TELEM_BYTES);
  Serial.println(" B");

  /* The raw buffer is not allocated on the JPEG path: the camera and the VENC
   * encoder allocate from the same malloc heap. */
#if USE_REAL_JPEG
  rgbBuf   = allocAligned(4096u);            /* fallback only */
#else
  rgbBuf   = allocAligned(RGB_BYTES);
#endif
  thermBuf = allocAligned(THERM_BYTES);
  if (rgbBuf == nullptr || thermBuf == nullptr) {
    Serial.println("[sd] FATAL: could not allocate frame buffers");
    for (;;) { delay(1000); }
  }
#if USE_REAL_JPEG
  Serial.print("[sd] Vision.begin (JPEG via the Ethernet transport slot)... ");
  NeuroN6_StreamUsbOnlyStart();          /* n6_usb first */
  {
    VisionConfig cfg;
    cfg.camera      = CAMERA_VGA;
    cfg.fps         = FPS_15;
    cfg.mirror_flip = CAMERA_AUTO_FLIP;
    cfg.transport   = TRANSPORT_ETHERNET;   /* -> EdgeViewFrames queue */
    cfg.lcd         = false;
    cfg.af          = AF_DISABLED;
    cfg.isp_enabled = VD66_isp_tuning_auto;
    Vision.begin(cfg);
  }
  if (!EdgeViewFrames.active()) {
    s_camMsg = "EdgeViewFrames not active - transport slot not claimed";
    Serial.println("FAILED - frames are not routed here");
  } else {
    Serial.println("ok, waiting for an encoded frame");
    const uint32_t t0 = millis();
    const uint8_t *jp = nullptr;
    uint32_t n = 0;
    /* The first two frames are discarded: the auto-exposure is still
     * settling. The loop allows up to 15 s. */
    int frames = 0;
    for (int i = 0; i < 30 && n == 0; i++) {
      const uint8_t *p = nullptr; uint32_t l = 0;
      if (EdgeViewFrames.grab(&p, &l, 500)) {
        if (++frames > 2 && l >= 512u) {
          jp = p; n = l;
          jpegStore = (uint8_t *)allocAligned(l);
          if (jpegStore) memcpy(jpegStore, p, l);
        }
        EdgeViewFrames.release();          /* always, or the gate stalls */
      }
    }
    (void)jp;
    Serial.print("grabbed JPEG -> ");
    Serial.print(n);
    Serial.print(" B in ");
    Serial.print(millis() - t0);
    Serial.println(" ms");
    s_camMs = millis() - t0;
    if (n >= 512u && jpegStore != nullptr) {
      jpegBuf = jpegStore;
      jpegLen = n;
      s_camMsg = "JPEG captured";
      Serial.print("[sd] JPEG is ");
      Serial.print((float)RGB_BYTES / (float)n, 1);
      Serial.println("x smaller than the raw frame");
    } else {
      s_camMsg = "no encoded frame arrived from the transport queue";
      s_encRv  = g_dbg_lb_enc_rv;
      s_encIni = g_dbg_ff_init_ret;
      s_encSps = g_dbg_ff_jesps;
      Serial.println("[sd] JPEG too small/failed - using the raw frame");
    }
  }
#endif

#if !USE_REAL_JPEG
  for (uint32_t i = 0; i < RGB_BYTES; i++)   rgbBuf[i]   = (uint8_t)(i * 31u);
#endif
  for (uint32_t i = 0; i < THERM_BYTES; i++) thermBuf[i] = (uint8_t)(i * 17u);
  for (uint32_t i = 0; i < TELEM_BYTES; i++) telemBuf[i] = (uint8_t)(i * 7u + 1u);

  SPI.begin();

  /* With PCLK1 at 200 MHz the slowest SCK is 781 kHz, above the 400 kHz an SD
   * card is guaranteed to initialise at. HSI at 64 MHz reaches 250 kHz. */
  Serial.print("[sd] SPI2 kernel before: ");
  Serial.print(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2) / 1000000.0f, 2);
  Serial.println(" MHz");
  SPI.setKernelSource(RCC_SPI2CLKSOURCE_HSI);
  Serial.print("[sd] SPI2 kernel after HSI switch: ");
  Serial.print(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2) / 1000000.0f, 2);
  Serial.println(" MHz  (want 64.00)");

  /* SdFat's native SPI_MODE0 is used. The line below pins mode 3 for a card
   * that needs it. */
  /* SPI.forceDataMode(SPI_MODE3); */
  Serial.println("[sd] SPE held across transactions; mode NOT pinned");
}

void loop()
{
#if USE_REAL_JPEG
  Serial.println();
  Serial.print("[sd] camera/JPEG: ");
  Serial.print(s_camMsg);
  Serial.print("   bytes=");  Serial.print(jpegLen);
  Serial.print("  encRv=");   Serial.print(s_encRv);
  Serial.print(" init=");     Serial.print(s_encIni);
  Serial.print(" setSize=");  Serial.print(s_encSps);
  Serial.print("  took ");    Serial.print(s_camMs);
  Serial.println(" ms");
#endif
  busProbe();
  wiringTester();
  bitbangProbe();

  bool any = false;
  for (uint32_t i = 0; i < sizeof(kClocksMHz) / sizeof(kClocksMHz[0]); i++) {
    if (runAtClock(kClocksMHz[i])) any = true;
  }

  Serial.println();
  if (!any) {
    Serial.println("[sd] card never initialised at any clock. Check CS on A5,");
    Serial.println("[sd] 3V3 power, a common ground, and that the module is a");
    Serial.println("[sd] 3.3 V type (5 V level-shifter boards need 5 V on VCC).");
  } else {
    Serial.println("[sd] ===== done =====");
    Serial.println("[sd] Take the fastest clock whose runs are all 'verify OK',");
    Serial.println("[sd] then derate one step for margin. A clock that writes");
    Serial.println("[sd] fast but fails verification is silently corrupting.");
  }
  Serial.println("[sd] (repeating in 5 s)");
  Serial.flush();
  delay(5000);
}
