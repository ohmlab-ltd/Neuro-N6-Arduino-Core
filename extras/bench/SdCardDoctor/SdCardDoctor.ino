/*
 * SdCardDoctor
 *
 * Works through the combinations of wiring, chip select and clock that could
 * carry an SD card on the header SPI, and reports which of them the card
 * answers. Every transfer is bit-banged.
 *
 * Hardware: Neuro N6 with an SD card on SPI2: MOSI D22/PD7, MISO D23/PD11,
 * SCK D24/PF2.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

#include <Arduino.h>

/* Settings */
#define PIN_SCK        24        /* PF2, the header SCK */
#define PIN_A          22        /* PD7, MOSI in the normal orientation */
#define PIN_B          23        /* PD11, MISO in the normal orientation */
static const uint8_t kCsCandidates[] = { 13, 19, 12, 11, 10, 9, 6, 5, 18, 17, 16, 15, 14 };
static const uint32_t kHalfUs[] = { 2, 10, 50 };   /* ~250 / 50 / 10 kHz */

struct Bb {
  uint8_t sck, mosi, miso, cs;
  uint32_t halfUs;
};

static const char *pinName(uint8_t p)
{
  switch (p) {
    case 22: return "D22/PD7";   case 23: return "D23/PD11";
    case 24: return "D24/PF2";   case 13: return "D13/PG14";
    case 19: return "A5/PG15";   case 12: return "D12/PA3";
    case 11: return "D11/PF15";  case 10: return "D10/PF10";
    case  9: return "D9/PD12";   case  6: return "D6/PF5";
    case  5: return "D5/PE9";    case 18: return "A4/PF14";
    case 17: return "A3/PF3";    case 16: return "A2/PF12";
    case 15: return "A1/PF11";   case 14: return "A0/PF13";
    default: return "?";
  }
}

static bool gpioSelfTest(uint8_t p)
{
  pinMode(p, OUTPUT);
  digitalWrite(p, LOW);  delayMicroseconds(50);
  const int lo = digitalRead(p);
  digitalWrite(p, HIGH); delayMicroseconds(50);
  const int hi = digitalRead(p);
  return (lo == 0 && hi == 1);
}

static bool hasExternalPullup(uint8_t p)
{
  pinMode(p, INPUT_PULLDOWN);
  delayMicroseconds(300);
  const int withDown = digitalRead(p);
  pinMode(p, INPUT);
  return (withDown == 1);          /* external pull-up beats ~40k internal */
}

/* Bit-banged SPI, mode 0. */
static uint8_t bbXfer(const Bb &b, uint8_t v)
{
  uint8_t r = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(b.mosi, (v >> i) & 1u);
    delayMicroseconds(b.halfUs);
    digitalWrite(b.sck, HIGH);
    delayMicroseconds(b.halfUs);
    r = (uint8_t)((r << 1) | (digitalRead(b.miso) ? 1u : 0u));
    digitalWrite(b.sck, LOW);
  }
  return r;
}

static void bbSetup(const Bb &b)
{
  pinMode(b.sck,  OUTPUT); digitalWrite(b.sck,  LOW);
  pinMode(b.mosi, OUTPUT); digitalWrite(b.mosi, HIGH);
  pinMode(b.miso, INPUT_PULLUP);
  pinMode(b.cs,   OUTPUT); digitalWrite(b.cs,   HIGH);
}

static uint8_t bbCmd(const Bb &b, uint8_t cmd, uint32_t arg, uint8_t crc)
{
  (void)bbXfer(b, 0xFF);
  (void)bbXfer(b, (uint8_t)(0x40u | cmd));
  (void)bbXfer(b, (uint8_t)(arg >> 24));
  (void)bbXfer(b, (uint8_t)(arg >> 16));
  (void)bbXfer(b, (uint8_t)(arg >> 8));
  (void)bbXfer(b, (uint8_t)arg);
  (void)bbXfer(b, crc);
  uint8_t r = 0xFF;
  for (uint32_t i = 0; i < 16u && r == 0xFFu; i++) r = bbXfer(b, 0xFF);
  return r;
}

static uint8_t tryCmd0(const Bb &b)
{
  bbSetup(b);
  delay(2);
  digitalWrite(b.cs, HIGH);
  for (uint32_t i = 0; i < 12u; i++) (void)bbXfer(b, 0xFF);   /* >=74 clocks */
  digitalWrite(b.cs, LOW);
  const uint8_t r = bbCmd(b, 0, 0, 0x95);
  digitalWrite(b.cs, HIGH);
  (void)bbXfer(b, 0xFF);
  return r;
}

static void fullInit(Bb b)
{
  Serial.println("\n[doc] === full init on the working combination ===");
  digitalWrite(b.cs, LOW);
  uint8_t r = bbCmd(b, 8, 0x000001AAu, 0x87);          /* CMD8 SEND_IF_COND */
  Serial.print("    CMD8 R1 = 0x"); Serial.print(r, HEX);
  if (r <= 1u) {
    uint8_t e[4];
    for (uint32_t i = 0; i < 4u; i++) e[i] = bbXfer(b, 0xFF);
    Serial.print("  echo ");
    for (uint32_t i = 0; i < 4u; i++) { Serial.print(e[i], HEX); Serial.print(" "); }
    Serial.println(e[3] == 0xAAu ? " (v2 card, voltage OK)" : " (unexpected echo)");
  } else {
    Serial.println("  (v1 card or no response)");
  }
  digitalWrite(b.cs, HIGH); (void)bbXfer(b, 0xFF);

  /* ACMD41 until the card leaves idle. */
  uint8_t r41 = 0xFF;
  uint32_t tries = 0;
  for (; tries < 200u && r41 != 0x00u; tries++) {
    digitalWrite(b.cs, LOW);
    (void)bbCmd(b, 55, 0, 0x65);
    r41 = bbCmd(b, 41, 0x40000000u, 0x77);
    digitalWrite(b.cs, HIGH); (void)bbXfer(b, 0xFF);
    if (r41 != 0x00u) delay(10);
  }
  Serial.print("    ACMD41 -> 0x"); Serial.print(r41, HEX);
  Serial.print(" after "); Serial.print(tries); Serial.println(" tries");

  if (r41 == 0x00u) {
    digitalWrite(b.cs, LOW);
    r = bbCmd(b, 58, 0, 0xFD);                          /* CMD58 READ_OCR */
    uint8_t ocr[4] = {0,0,0,0};
    if (r <= 1u) for (uint32_t i = 0; i < 4u; i++) ocr[i] = bbXfer(b, 0xFF);
    digitalWrite(b.cs, HIGH); (void)bbXfer(b, 0xFF);
    Serial.print("    CMD58 OCR = ");
    for (uint32_t i = 0; i < 4u; i++) { Serial.print(ocr[i], HEX); Serial.print(" "); }
    Serial.println((ocr[0] & 0x40u) ? " -> SDHC/SDXC (block addressing)"
                                    : " -> SDSC (byte addressing)");
    Serial.println("    *** CARD IS FULLY INITIALISED ***");
  } else {
    Serial.println("    card answered CMD0 but never left idle.");
  }
}

void setup()
{
  Serial.begin(115200);
  uint32_t tw = millis();
  while (!Serial && (millis() - tw) < 3000u) { delay(10); }
}

void loop()
{
  Serial.println("\n\n[doc] ================ SD CARD DOCTOR ================");
  Serial.println("[doc] bus = SPI2 header pins (NOT the Lepton's SPI1)");

  Serial.println("\n[doc] --- 1. GPIO self-test (pin drives + reads back) ---");
  const uint8_t core[] = { PIN_SCK, PIN_A, PIN_B };
  for (uint32_t i = 0; i < 3u; i++) {
    Serial.print("    ");  Serial.print(pinName(core[i]));
    Serial.println(gpioSelfTest(core[i]) ? "  OK" : "  *** STUCK ***");
  }

  Serial.println("\n[doc] --- 2. external pull-up census ---");
  Serial.println("    (a bare SD adapter pulls CMD/DAT up, so a pin listed");
  Serial.println("     here is physically wired to the adapter)");
  uint32_t found = 0;
  for (uint8_t p = 0; p < 29u; p++) {
    if (p == 25u || p == 26u || p == 27u) continue;    /* LEDs */
    if (hasExternalPullup(p)) {
      Serial.print("    pin ");  Serial.print(p);
      Serial.print("  ");        Serial.print(pinName(p));
      Serial.println("  <-- externally pulled UP");
      found++;
    }
  }
  if (found == 0u) {
    Serial.println("    NONE. No pin on this header sees an external pull-up,");
    Serial.println("    which means the adapter is not electrically connected");
    Serial.println("    to any of them - check power and grounds first.");
  }

  Serial.println("\n[doc] --- 3. connectivity between candidate pins ---");
  const uint8_t probe[] = { PIN_SCK, PIN_A, PIN_B, 13, 19 };
  for (uint32_t d = 0; d < 5u; d++) {
    for (uint32_t s = 0; s < 5u; s++) {
      if (s == d) continue;
      pinMode(probe[s], INPUT_PULLUP);
      pinMode(probe[d], OUTPUT);
      digitalWrite(probe[d], LOW);  delayMicroseconds(200);
      const int lo = digitalRead(probe[s]);
      digitalWrite(probe[d], HIGH); delayMicroseconds(200);
      const int hi = digitalRead(probe[s]);
      pinMode(probe[d], INPUT);
      if (lo == 0 && hi == 1) {
        Serial.print("    ");   Serial.print(pinName(probe[d]));
        Serial.print("  ==>  "); Serial.print(pinName(probe[s]));
        Serial.println("   CONNECTED");
      }
    }
  }

  Serial.println("\n[doc] --- 4. brute-force CMD0 (CS x orientation x clock) ---");
  bool solved = false;
  Bb win = {0,0,0,0,0};
  for (uint32_t c = 0; c < sizeof(kCsCandidates) && !solved; c++) {
    const uint8_t cs = kCsCandidates[c];
    if (cs == PIN_A || cs == PIN_B || cs == PIN_SCK) continue;
    for (uint32_t orient = 0; orient < 2u && !solved; orient++) {
      for (uint32_t k = 0; k < 3u && !solved; k++) {
        Bb b;
        b.sck    = PIN_SCK;
        b.mosi   = orient ? PIN_B : PIN_A;
        b.miso   = orient ? PIN_A : PIN_B;
        b.cs     = cs;
        b.halfUs = kHalfUs[k];
        const uint8_t r = tryCmd0(b);
        if (r == 0x01u) {
          Serial.println("\n    *** CMD0 ANSWERED 0x01 ***");
          Serial.print("      CS   = "); Serial.println(pinName(b.cs));
          Serial.print("      MOSI = "); Serial.println(pinName(b.mosi));
          Serial.print("      MISO = "); Serial.println(pinName(b.miso));
          Serial.print("      SCK  = "); Serial.println(pinName(b.sck));
          Serial.print("      clock ~"); Serial.print(500u / b.halfUs);
          Serial.println(" kHz");
          solved = true; win = b;
        } else if (r != 0xFFu) {
          Serial.print("    CS ");     Serial.print(pinName(cs));
          Serial.print(orient ? " swapped" : " normal");
          Serial.print(" -> R1 0x");   Serial.println(r, HEX);
        }
      }
    }
  }

  if (solved) {
    fullInit(win);
  } else {
    Serial.println("    no combination answered. Every CS pin, both");
    Serial.println("    orientations, three clocks down to 10 kHz: silent.");
    Serial.println("    With the GPIOs proven above, that leaves SCK not");
    Serial.println("    reaching the card, no common ground, the card");
    Serial.println("    unpowered, or the card not seated.");
    Serial.println("    Check section 2: if NOTHING is externally pulled up,");
    Serial.println("    the adapter is not connected at all.");
  }

  Serial.println("\n[doc] (repeating in 8 s)");
  Serial.flush();
  delay(8000);
}
