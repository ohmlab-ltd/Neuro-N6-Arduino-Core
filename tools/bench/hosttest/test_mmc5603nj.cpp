/* Host test of the magnetometer driver, libraries/MMC5603NJ/src/MMC5603NJ.cpp,
 * against a simulated part on a simulated Wire bus. From this folder:
 *
 *   g++ -std=c++17 -Wall -Ifake_mag -I../../../cores/NeuroN6 -I../../../libraries/MMC5603NJ/src \
 *       test_mmc5603nj.cpp ../../../libraries/MMC5603NJ/src/MMC5603NJ.cpp -o /tmp/test_mmc5603nj
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#include <stdio.h>
#include <string.h>
#include "MMC5603NJ.h"
#include "../../../libraries/PostProcess/src/vnd_meta.h"

/* Clock */
static uint32_t now;
static void part_update(void);
uint32_t millis(void) { return now; }
void delay(uint32_t ms) { now += ms; part_update(); }

extern "C" void NeuroN6_StreamUsbOnlyStart(void) {}
extern "C" bool vnd_meta_publish_tlv(vnd_meta_slot_t, uint8_t, const void *, uint16_t) { return true; }

/* The part at 0x30 */
static uint8_t  reg[256];
static bool     running;         /* CTRL2 CMM_EN */
static uint32_t odr, next_at;
static uint32_t samples, triggers;
static int      fail_reg = -1, fail_val = -1;

static void new_sample(void)
{
  const uint32_t x = 524288u + 1000u * (++samples);
  reg[0] = (uint8_t)(x >> 12); reg[1] = (uint8_t)(x >> 4); reg[6] = (uint8_t)((x & 0xFu) << 4);
  reg[2] = 0x80; reg[3] = 0; reg[7] = 0; reg[4] = 0x80; reg[5] = 0; reg[8] = 0;
  reg[0x18] |= 0x40;
}

static void part_update(void)
{
  if (running && odr) {
    while ((int32_t)(now - next_at) >= 0) { new_sample(); next_at += 1000u / odr; }
  }
}

static bool part_write(uint8_t r, uint8_t v)
{
  if ((int)r == fail_reg && (int)v == fail_val) { fail_reg = -1; return false; }
  switch (r) {
    case 0x1B:
      if ((v & 0x01) && !running) { triggers++; new_sample(); }
      if ((v & 0x02) && !running) { reg[0x09] = 125; reg[0x18] |= 0x80; }
      break;
    case 0x1C: break;
    case 0x1D:
      running = (v & 0x10) != 0;
      if (running) next_at = now + 1000u / (odr ? odr : 1u);
      break;
    case 0x1A: odr = v; break;
    default: reg[r] = v; break;
  }
  return true;
}

/* Wire */
TwoWire Wire;
static uint8_t tx[8], txn, rx_at, rx_left;

void TwoWire::beginTransmission(uint8_t) { txn = 0; }
size_t TwoWire::write(uint8_t b) { if (txn < sizeof tx) tx[txn++] = b; return 1; }
size_t TwoWire::write(const uint8_t *p, size_t n) { for (size_t i = 0; i < n; i++) write(p[i]); return n; }
uint8_t TwoWire::endTransmission(bool)
{
  if (txn == 1) { rx_at = tx[0]; return 0; }                 /* a register address for a read */
  for (uint8_t i = 1; i < txn; i++) if (!part_write((uint8_t)(tx[0] + i - 1), tx[i])) return 2;
  return 0;
}
uint8_t TwoWire::requestFrom(uint8_t, uint8_t n) { rx_left = n; return n; }
int TwoWire::read(void)
{
  if (rx_left == 0) return -1;
  rx_left--;
  const uint8_t r = rx_at++;
  const uint8_t v = reg[r];
  if (r == 0x00) reg[0x18] &= (uint8_t)~0x40;
  if (r == 0x09) reg[0x18] &= (uint8_t)~0x80;
  return v;
}

static int fails, checks;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

int main(void)
{
  MMC5603NJ mag;
  reg[0x39] = 0x10;

  /* 1. free-running at 50 Hz: each read is a later sample */
  CHECK(mag.begin());
  mag.setContinuousMode(50);
  CHECK(running);
  delay(25);
  int32_t a = mag.getRawMag().x;
  delay(25);
  CHECK(mag.getRawMag().x != a);

  /* 2. the restart after a temperature reading not acknowledged: the part
   *    stays stopped, and each read still measures */
  fail_reg = 0x1D; fail_val = 0x10;
  CHECK(mag.getRawTemp() == 125);
  CHECK(!running);
  {
    int same = 0;
    a = mag.getRawMag().x;
    for (int i = 0; i < 20; i++) {
      delay(25);
      const int32_t b = mag.getRawMag().x;
      if (b == a) same++;
      a = b;
    }
    CHECK(same == 0);
  }

  /* 3. the next temperature reading starts free-running again */
  CHECK(mag.getRawTemp() == 125);
  CHECK(running);
  {
    const uint32_t t0 = triggers;
    delay(25);
    a = mag.getRawMag().x;
    delay(25);
    CHECK(mag.getRawMag().x != a && triggers == t0);
  }

  /* 4. single-shot: each read measures */
  mag.setContinuousMode(0);
  CHECK(!running);
  {
    const uint32_t t0 = triggers;
    a = mag.getRawMag().x;
    CHECK(mag.getRawMag().x != a && triggers == t0 + 2u);
  }

  printf("%d checks, %d failed\n", checks, fails);
  return fails != 0;
}
