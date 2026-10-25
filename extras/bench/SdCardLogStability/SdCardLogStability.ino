/*
 * SdCardLogStability
 *
 * Soak test for the SD path, with no camera. Every cycle re-initialises the
 * card, writes three payloads, byte-compares them and prints failure totals.
 *
 * Hardware: Neuro N6 with an SD card on the header SPI, SPI2:
 *   MOSI D22/PD7   MISO D23/PD11   SCK D24/PF2   CS A5/PG15
 *   3V3, a common ground, 10 uF and 100 nF at the adapter's VCC pin.
 *
 * Dependency: SdFat (MIT), arduino-cli lib install SdFat
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

/* Settings */
#define SD_CS_PIN       A5
#define SD_CLOCK_MHZ    32
#define IMAGE_BYTES     20000UL     /* stands in for a VGA JPEG */
#define THERM_BYTES     38400UL     /* Lepton 160x120 x 2 */
#define TELEM_BYTES     512UL
#define CYCLE_MS        2000UL
#define USE_FOLDERS     0           /* 0 = flat files, skips the mkdir */
/* 1 = one .ohmlab container per capture instead of three files. */
#define USE_CONTAINER   1

/* .ohmlab container: a 64-byte header, then the payloads back to back.
 * Little-endian. */
#define OHMLAB_MAGIC    "OHMLAB1"
#define OHMLAB_VERSION  1u
#define OHMLAB_FMT_JPEG 1u
#define OHMLAB_FMT_RAW  0u

typedef struct __attribute__((packed)) {
  char     magic[8];        /* "OHMLAB1", NUL-padded */
  uint32_t version;
  uint32_t seq;             /* capture index, monotonic within a power-up */
  uint32_t timestampMs;     /* device uptime */
  uint32_t headerBytes;     /* sizeof this struct; payloads start after */
  uint32_t imageOff,   imageLen;
  uint32_t thermalOff, thermalLen;
  uint32_t telemOff,   telemLen;
  uint32_t imageFormat;     /* OHMLAB_FMT_* */
  uint32_t thermalW, thermalH;
  uint32_t reserved;        /* pads to 64 */
} OhmlabHeader;

static SdFs   sd;
static FsFile file;

static uint8_t *imgBuf, *thermBuf;
static uint8_t  telemBuf[TELEM_BYTES];
static uint8_t  vbuf[4096];

static uint32_t s_cycle = 0, s_seq = 0;
static uint32_t s_failInit = 0, s_failWrite = 0, s_failVerify = 0;
static uint32_t s_upMinUs = 0xFFFFFFFFu, s_upMaxUs = 0;
static uint32_t s_wrMinUs = 0xFFFFFFFFu, s_wrMaxUs = 0;

static uint8_t *allocAligned(uint32_t n)
{
  void *raw = malloc(n + 32u);
  if (!raw) return nullptr;
  return (uint8_t *)(((uintptr_t)raw + 31u) & ~(uintptr_t)31u);
}

#if !USE_CONTAINER
static bool writeFile(const char *path, const uint8_t *buf, uint32_t len)
{
  if (!file.open(path, O_WRONLY | O_CREAT | O_TRUNC)) return false;
  file.preAllocate(len);
  const int32_t n = file.write(buf, len);
  file.close();
  return (n == (int32_t)len);
}

/* Returns 0 on success, else a code, with detail carrying the offending offset
 * or size. */
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
    if (f.read(vbuf, want) != (int32_t)want) { detail = off; f.close(); return 4; }
    for (uint32_t i = 0; i < want; i++) {
      if (vbuf[i] != expect[off + i]) { detail = off + i; f.close(); return 3; }
    }
    off += want;
  }
  f.close();
  return 0;
}
#endif

void setup()
{
  Serial.begin(115200);
  uint32_t tw = millis();
  while (!Serial && (millis() - tw) < 2500u) { delay(10); }

  imgBuf   = allocAligned(IMAGE_BYTES);
  thermBuf = allocAligned(THERM_BYTES);
  if (!imgBuf || !thermBuf) {
    Serial.println("[log] FATAL: buffer allocation failed");
    for (;;) delay(1000);
  }
  for (uint32_t i = 0; i < IMAGE_BYTES; i++) imgBuf[i]   = (uint8_t)(i * 31u + 7u);
  for (uint32_t i = 0; i < THERM_BYTES; i++) thermBuf[i] = (uint8_t)(i * 17u);
  for (uint32_t i = 0; i < TELEM_BYTES; i++) telemBuf[i] = (uint8_t)(i * 7u + 1u);

  SPI.begin();
  /* SPI2 on PCLK1 cannot divide below 781 kHz, above the SD spec's 400 kHz
   * init limit. On HSI the same prescaler reaches 250 kHz. */
  SPI.setKernelSource(RCC_SPI2CLKSOURCE_HSI);

  Serial.println();
  Serial.println("[log] ===== SD logging stability soak =====");
  Serial.print("[log] per cycle: image ");   Serial.print(IMAGE_BYTES);
  Serial.print(" B + thermal ");             Serial.print(THERM_BYTES);
  Serial.print(" B + telem ");               Serial.print(TELEM_BYTES);
  Serial.print(" B, every ");                Serial.print(CYCLE_MS);
  Serial.println(" ms, card re-initialised each cycle");
}

void loop()
{
  const uint32_t tCycle = millis();
  s_cycle++;

  bool ok = true;
  uint32_t upUs = 0, wrUs = 0;

  SdSpiConfig cfg(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(SD_CLOCK_MHZ), &SPI);
  const uint32_t t0 = micros();
  const bool up = sd.begin(cfg);
  upUs = micros() - t0;

  if (!up) {
    s_failInit++; ok = false;
    Serial.print("[log] #");   Serial.print(s_cycle);
    Serial.print("  INIT FAILED after ");
    Serial.print(upUs / 1000.0f, 1);
    Serial.println(" ms");
  } else {
    if (upUs < s_upMinUs) s_upMinUs = upUs;
    if (upUs > s_upMaxUs) s_upMaxUs = upUs;

#if USE_CONTAINER
    ++s_seq;
    char pCap[40];
    snprintf(pCap, sizeof(pCap), "/cap%05lu.ohmlab", (unsigned long)s_seq);

    OhmlabHeader hdr;
    memset(&hdr, 0, sizeof(hdr));
    memcpy(hdr.magic, OHMLAB_MAGIC, 7);
    hdr.version     = OHMLAB_VERSION;
    hdr.seq         = s_seq;
    hdr.timestampMs = millis();
    hdr.headerBytes = (uint32_t)sizeof(OhmlabHeader);
    hdr.imageOff    = hdr.headerBytes;
    hdr.imageLen    = IMAGE_BYTES;
    hdr.thermalOff  = hdr.imageOff + hdr.imageLen;
    hdr.thermalLen  = THERM_BYTES;
    hdr.telemOff    = hdr.thermalOff + hdr.thermalLen;
    hdr.telemLen    = TELEM_BYTES;
    hdr.imageFormat = OHMLAB_FMT_JPEG;
    hdr.thermalW    = 160; hdr.thermalH = 120;
    const uint32_t totalLen = hdr.telemOff + hdr.telemLen;

    const uint32_t tw0 = micros();
    bool w1 = false, w2 = false, w3 = false;
    if (file.open(pCap, O_WRONLY | O_CREAT | O_TRUNC)) {
      file.preAllocate(totalLen);
      const bool h  = (file.write(&hdr, sizeof(hdr)) == (int32_t)sizeof(hdr));
      w1 = (file.write(imgBuf,   IMAGE_BYTES) == (int32_t)IMAGE_BYTES);
      w2 = (file.write(thermBuf, THERM_BYTES) == (int32_t)THERM_BYTES);
      w3 = (file.write(telemBuf, TELEM_BYTES) == (int32_t)TELEM_BYTES);
      file.close();
      w1 = w1 && h;
    }
    wrUs = micros() - tw0;

    if (!(w1 && w2 && w3)) { s_failWrite++; ok = false; }
    if (wrUs < s_wrMinUs) s_wrMinUs = wrUs;
    if (wrUs > s_wrMaxUs) s_wrMaxUs = wrUs;

    int bad = 0; uint32_t d = 0;
    {
      FsFile f;
      if (!f.open(pCap, O_RDONLY)) { bad = 1; }
      else if (f.fileSize() != (uint64_t)totalLen) { bad = 2; d = (uint32_t)f.fileSize(); f.close(); }
      else {
        OhmlabHeader rh;
        if (f.read(&rh, sizeof(rh)) != (int32_t)sizeof(rh)) { bad = 4; }
        else if (memcmp(rh.magic, OHMLAB_MAGIC, 7) != 0 ||
                 rh.version != OHMLAB_VERSION || rh.seq != s_seq ||
                 rh.imageLen != IMAGE_BYTES || rh.thermalLen != THERM_BYTES ||
                 rh.telemLen != TELEM_BYTES) { bad = 5; }
        if (!bad) {
          struct { uint32_t off, len; const uint8_t *exp; } sec[3] = {
            { rh.imageOff,   rh.imageLen,   imgBuf   },
            { rh.thermalOff, rh.thermalLen, thermBuf },
            { rh.telemOff,   rh.telemLen,   telemBuf },
          };
          for (int s = 0; s < 3 && !bad; s++) {
            if (!f.seekSet(sec[s].off)) { bad = 6; d = sec[s].off; break; }
            uint32_t off = 0;
            while (off < sec[s].len && !bad) {
              uint32_t want = sec[s].len - off;
              if (want > sizeof(vbuf)) want = sizeof(vbuf);
              if (f.read(vbuf, want) != (int32_t)want) { bad = 4; d = sec[s].off + off; break; }
              for (uint32_t i = 0; i < want; i++) {
                if (vbuf[i] != sec[s].exp[off + i]) { bad = 3; d = sec[s].off + off + i; break; }
              }
              off += want;
            }
          }
        }
        f.close();
      }
    }
    if (bad) { s_failVerify++; ok = false; }
#else
    char pImg[40], pTh[40], pTe[40];
#if USE_FOLDERS
    char dir[24];
    do {
      snprintf(dir, sizeof(dir), "/LOG%05lu", (unsigned long)(++s_seq));
    } while (sd.exists(dir) && s_seq < 99999u);
    const uint32_t tw0 = micros();
    const bool mkOk = sd.mkdir(dir);
    snprintf(pImg, sizeof(pImg), "%s/image.jpg",   dir);
    snprintf(pTh,  sizeof(pTh),  "%s/thermal.raw", dir);
    snprintf(pTe,  sizeof(pTe),  "%s/telem.bin",   dir);
#else
    ++s_seq;
    const uint32_t tw0 = micros();
    const bool mkOk = true;
    snprintf(pImg, sizeof(pImg), "/img%05lu.jpg", (unsigned long)s_seq);
    snprintf(pTh,  sizeof(pTh),  "/thm%05lu.raw", (unsigned long)s_seq);
    snprintf(pTe,  sizeof(pTe),  "/tlm%05lu.bin", (unsigned long)s_seq);
#endif
    const bool w1 = writeFile(pImg, imgBuf,   IMAGE_BYTES);
    const bool w2 = writeFile(pTh,  thermBuf, THERM_BYTES);
    const bool w3 = writeFile(pTe,  telemBuf, TELEM_BYTES);
    wrUs = micros() - tw0;

    if (!(mkOk && w1 && w2 && w3)) { s_failWrite++; ok = false; }
    if (wrUs < s_wrMinUs) s_wrMinUs = wrUs;
    if (wrUs > s_wrMaxUs) s_wrMaxUs = wrUs;

    uint32_t d; int rc;
    int bad = 0;
    rc = verifyFile(pImg, imgBuf,   IMAGE_BYTES, d); if (rc) { bad = rc; }
    if (!bad) { rc = verifyFile(pTh, thermBuf, THERM_BYTES, d); if (rc) bad = rc; }
    if (!bad) { rc = verifyFile(pTe, telemBuf, TELEM_BYTES, d); if (rc) bad = rc; }
    if (bad) { s_failVerify++; ok = false; }
#endif  /* USE_CONTAINER */

    Serial.print("[log] #");        Serial.print(s_cycle);
    Serial.print("  up ");          Serial.print(upUs / 1000.0f, 1);
    Serial.print(" ms  write ");    Serial.print(wrUs / 1000.0f, 1);
    Serial.print(" ms  ");          Serial.print(ok ? "OK" : "FAIL");
    if (bad) { Serial.print(" (verify rc="); Serial.print(bad);
               Serial.print(" @");           Serial.print(d); Serial.print(")"); }
    Serial.print("   fails: init "); Serial.print(s_failInit);
    Serial.print(" write ");         Serial.print(s_failWrite);
    Serial.print(" verify ");        Serial.print(s_failVerify);
    Serial.print("   up[");          Serial.print(s_upMinUs / 1000.0f, 1);
    Serial.print("-");               Serial.print(s_upMaxUs / 1000.0f, 1);
    Serial.print("] wr[");           Serial.print(s_wrMinUs / 1000.0f, 1);
    Serial.print("-");               Serial.print(s_wrMaxUs / 1000.0f, 1);
    Serial.println("] ms");
  }

  sd.end();

  const uint32_t el = millis() - tCycle;
  if (el < CYCLE_MS) delay(CYCLE_MS - el);
}
