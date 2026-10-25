/* Host test of the asset store, libraries/N6Assets/src/n6_asset.c. From this folder:
 *
 *   MSVC, x64 Native Tools prompt:
 *     cl /nologo /W3 /std:c11 /Ifake /I..\..\..\libraries\N6Assets\src test_asset.c /Fe:%TEMP%\test_asset.exe
 *   gcc:
 *     gcc -std=c11 -Wall -Ifake -I../../../libraries/N6Assets/src test_asset.c -o /tmp/test_asset
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */
#define __attribute__(x)
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32n6xx.h"
#include "n6_ota_layout.h"
#undef N6_OTA_FLASH_MMAP_BASE
#undef N6_ASSET_PSRAM_BASE
static uint8_t g_flash[0x2000];
static uint8_t g_psram[0x1100];
#define N6_OTA_FLASH_MMAP_BASE ((uintptr_t)g_flash - 0x00282000u)
#define N6_ASSET_PSRAM_BASE    ((uintptr_t)g_psram)
TAMP_TypeDef g_tamp;
PWR_TypeDef g_pwr;
int g_susp;
#include "n6_asset.c"

#include <setjmp.h>

static int fails, checks;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

/* The D-cache: a spoilt byte reads wrong until its line is dropped. */
static uint8_t *g_spoilt;
static uint8_t  g_true;
static int      g_drops;

void SCB_InvalidateDCache_by_Addr(volatile void *addr, int32_t dsize)
{
  uint8_t *a = (uint8_t *)addr;
  if (g_spoilt != NULL && g_spoilt >= a && g_spoilt < a + dsize) {
    *g_spoilt = g_true;
    g_spoilt = NULL;
    g_drops++;
  }
}

static void spoil(uint8_t *p, uint8_t bad)
{
  g_true = *p;
  *p = bad;
  g_spoilt = p;
}

static jmp_buf g_reset;
static int g_resets;

void fake_system_reset(void)
{
  g_resets++;
  longjmp(g_reset, 1);
}

static uint8_t *copy_at(int c) { return g_flash + (c ? 0x1000 : 0); }

static int copy_ok(int c, uint32_t *ver)
{
  n6_asset_hdr_t h;
  memcpy(&h, copy_at(c), sizeof(h));
  if (h.magic != N6_ASSET_MAGIC || h.hdr_crc != n6_ota_crc32(0, (uint8_t *)&h, 16)) return 0;
  if (h.len == 0 || h.len > N6_ASSET_MAX_BYTES) return 0;
  if (n6_ota_crc32(0, copy_at(c) + 20, h.len) != h.crc32) return 0;
  *ver = h.version;
  return 1;
}

static void write_copy(int c, uint32_t ver, const void *p, uint32_t len)
{
  n6_asset_hdr_t h = { N6_ASSET_MAGIC, ver, len, n6_ota_crc32(0, p, len), 0 };
  h.hdr_crc = n6_ota_crc32(0, (uint8_t *)&h, 16);
  memset(copy_at(c), 0xFF, 0x1000);
  memcpy(copy_at(c), &h, sizeof(h));
  memcpy(copy_at(c) + 20, p, len);
}

/* The FSBL: programs a staged asset into the copy that is not the current best. */
static int fsbl(void)
{
  n6_asset_hdr_t *h = (n6_asset_hdr_t *)g_psram;
  uint32_t v0 = 0, v1 = 0;
  int ok0, ok1, target;
  if ((&TAMP->BKP0R)[N6_ASSET_TRIGGER_BKPR] != N6_ASSET_TRIGGER_MAGIC) return 0;
  (&TAMP->BKP0R)[N6_ASSET_TRIGGER_BKPR] = 0;
  if (h->magic != N6_ASSET_PSRAM_MAGIC || h->hdr_crc != n6_ota_crc32(0, g_psram, 16)) return -1;
  if (h->len == 0 || h->len > N6_ASSET_MAX_BYTES) return -2;
  if (n6_ota_crc32(0, g_psram + 20, h->len) != h->crc32) return -3;
  ok0 = copy_ok(0, &v0); ok1 = copy_ok(1, &v1);
  target = (ok0 && (!ok1 || v0 >= v1)) ? 1 : 0;
  write_copy(target, h->version, g_psram + 20, h->len);
  (&TAMP->BKP0R)[N6_ASSET_RESULT_BKPR] = N6_ASSET_RESULT_OK;
  return 1;
}

/* The newest valid copy's payload, as stored. */
static uint32_t raw_payload(uint8_t *out, uint32_t *ver)
{
  uint32_t v0 = 0, v1 = 0;
  int ok0 = copy_ok(0, &v0), ok1 = copy_ok(1, &v1), c;
  n6_asset_hdr_t h;
  if (!ok0 && !ok1) return 0;
  c = (ok0 && (!ok1 || v0 > v1)) ? 0 : 1;
  memcpy(&h, copy_at(c), sizeof(h));
  memcpy(out, copy_at(c) + 20, h.len);
  *ver = h.version;
  return h.len;
}

static int user_is(const char *want)
{
  uint8_t buf[N6_ASSET_MAX_BYTES];
  uint32_t len = 0, ver = 0;
  if (!n6_asset_read(buf, sizeof(buf), &len, &ver)) return want == NULL;
  return want != NULL && len == strlen(want) && memcmp(buf, want, len) == 0;
}

static int rec_is(uint8_t type, const void *want, uint32_t wlen)
{
  uint8_t buf[N6_ASSET_MAX_BYTES];
  uint32_t len = 0;
  if (!n6_asset_record_read(type, buf, sizeof(buf), &len)) return want == NULL;
  return want != NULL && len == wlen && memcmp(buf, want, len) == 0;
}

int main(void)
{
  uint8_t raw[N6_ASSET_MAX_BYTES], big[N6_ASSET_MAX_BYTES];
  uint32_t ver = 0, len = 0, n;

  memset(g_flash, 0xFF, sizeof(g_flash));

  /* 1. nothing stored */
  CHECK(user_is(NULL));
  CHECK(rec_is(2, NULL, 0));
  CHECK(!n6_asset_present(&ver, &len));
  CHECK(g_susp == 0);

  /* 2. a user asset alone is stored plain */
  CHECK(n6_asset_stage("hello", 5, 0));
  CHECK(g_susp == 0);
  CHECK(fsbl() == 1);
  CHECK(user_is("hello"));
  n = raw_payload(raw, &ver);
  CHECK(n == 5 && memcmp(raw, "hello", 5) == 0 && ver == 1);
  CHECK(n6_asset_present(&ver, &len) && ver == 1 && len == 5);

  /* 3. a WiFi record beside it: a container, the user asset kept */
  CHECK(n6_asset_record_stage(2, "wifi-tlv", 8));
  CHECK(fsbl() == 1);
  CHECK(user_is("hello"));
  CHECK(rec_is(2, "wifi-tlv", 8));
  n = raw_payload(raw, &ver);
  CHECK(n == 4 + 4 + 5 + 4 + 8 && memcmp(raw, "N6RC", 4) == 0 && ver == 2);

  /* 4. replace the user asset: the WiFi record kept */
  CHECK(n6_asset_stage("hello2", 6, 0));
  CHECK(fsbl() == 1);
  CHECK(user_is("hello2"));
  CHECK(rec_is(2, "wifi-tlv", 8));
  CHECK(n6_asset_present(&ver, &len) && ver == 3 && len == 6);

  /* 5. replace the WiFi record */
  CHECK(n6_asset_record_stage(2, "wifi-2", 6));
  CHECK(fsbl() == 1);
  CHECK(user_is("hello2") && rec_is(2, "wifi-2", 6));

  /* 6. remove the WiFi record: back to plain */
  CHECK(n6_asset_record_stage(2, NULL, 0));
  CHECK(fsbl() == 1);
  CHECK(user_is("hello2") && rec_is(2, NULL, 0));
  n = raw_payload(raw, &ver);
  CHECK(n == 6 && memcmp(raw, "hello2", 6) == 0 && ver == 5);

  /* 7. remove the user asset: an empty container */
  CHECK(n6_asset_record_stage(1, NULL, 0));
  CHECK(fsbl() == 1);
  CHECK(user_is(NULL) && !n6_asset_present(NULL, NULL));
  n = raw_payload(raw, &ver);
  CHECK(n == 4 && memcmp(raw, "N6RC", 4) == 0 && ver == 6);

  /* 8. a WiFi record alone, then a user asset beside it with an explicit lower version */
  CHECK(n6_asset_record_stage(2, "w", 1));
  CHECK(fsbl() == 1);
  CHECK(!n6_asset_present(&ver, NULL));
  CHECK(n6_asset_stage("cal", 3, 1));
  CHECK(fsbl() == 1);
  CHECK(user_is("cal") && rec_is(2, "w", 1));
  n = raw_payload(raw, &ver);
  CHECK(ver == 8);

  /* 9. a user asset whose bytes would pass for a container stays wrapped */
  {
    static const uint8_t tricky[] = { 'N', '6', 'R', 'C', 5, 0, 1, 0, 'X' };
    uint8_t buf[64];
    CHECK(n6_asset_record_stage(2, NULL, 0));
    CHECK(fsbl() == 1);
    CHECK(n6_asset_stage(tricky, sizeof(tricky), 0));
    CHECK(fsbl() == 1);
    CHECK(n6_asset_read(buf, sizeof(buf), &len, NULL) && len == sizeof(tricky) &&
          memcmp(buf, tricky, len) == 0);
    CHECK(rec_is(5, NULL, 0));
    n = raw_payload(raw, &ver);
    CHECK(n == 8 + sizeof(tricky));
  }

  /* 10. no room: a full-size user asset refuses a record beside it, arming nothing */
  memset(big, 0x5A, sizeof(big));
  CHECK(n6_asset_stage(big, sizeof(big), 0));
  CHECK(fsbl() == 1);
  CHECK(!n6_asset_record_stage(2, "w", 1));
  CHECK(g_susp == 0);
  CHECK(fsbl() == 0);
  {
    uint8_t buf[N6_ASSET_MAX_BYTES];
    CHECK(n6_asset_read(buf, sizeof(buf), &len, NULL) && len == sizeof(big) && memcmp(buf, big, len) == 0);
  }
  /* a record leaves less room for the user asset */
  CHECK(n6_asset_stage("small", 5, 0));
  CHECK(fsbl() == 1);
  CHECK(n6_asset_record_stage(2, "wifi", 4));
  CHECK(fsbl() == 1);
  CHECK(!n6_asset_stage(big, sizeof(big), 0));
  CHECK(fsbl() == 0);
  CHECK(n6_asset_stage(big, N6_ASSET_MAX_BYTES - 16, 0));
  CHECK(fsbl() == 1);
  CHECK(rec_is(2, "wifi", 4));

  /* 11. a store written without a container */
  memset(g_flash, 0xFF, sizeof(g_flash));
  write_copy(0, 7, "legacy", 6);
  CHECK(user_is("legacy") && rec_is(2, NULL, 0));
  CHECK(n6_asset_record_stage(2, "w", 1));
  CHECK(fsbl() == 1);
  CHECK(user_is("legacy") && rec_is(2, "w", 1));
  n = raw_payload(raw, &ver);
  CHECK(ver == 8);

  /* 12. a corrupt newest copy: the older one is read, and kept by the next write */
  {
    uint32_t v0 = 0, v1 = 0;
    int newest = (copy_ok(0, &v0), copy_ok(1, &v1), v1 > v0) ? 1 : 0;
    copy_at(newest)[30] ^= 0x01;
    CHECK(user_is("legacy") && rec_is(2, NULL, 0));
    CHECK(n6_asset_record_stage(3, "r3", 2));
    CHECK(fsbl() == 1);
    CHECK(user_is("legacy") && rec_is(3, "r3", 2));
  }

  /* 13. a buffer too small is refused, not truncated; type 0 is not a record */
  {
    uint8_t buf[2];
    CHECK(!n6_asset_read(buf, sizeof(buf), &len, NULL));
    CHECK(n6_asset_record_read(3, NULL, 0, &len) && len == 2);
    CHECK(!n6_asset_record_read(0, NULL, 0, &len));
    CHECK(!n6_asset_record_stage(0, "x", 1));
  }
  CHECK(g_susp == 0);

  /* 14. the only copy corrupt: the next write goes past the version its header claims */
  memset(g_flash, 0xFF, sizeof(g_flash));
  write_copy(0, 50, "broken", 6);
  copy_at(0)[22] ^= 0x10;
  CHECK(user_is(NULL));
  CHECK(n6_asset_stage("x", 1, 0));
  CHECK(fsbl() == 1);
  CHECK(user_is("x"));
  n = raw_payload(raw, &ver);
  CHECK(ver == 51);
  CHECK(g_susp == 0);

  /* 15. a bad read of the newest copy held in the cache is read again from
   *     the flash, a wrong magic included; erased flash is an empty copy */
  memset(g_flash, 0xFF, sizeof(g_flash));
  write_copy(0, 1, "old", 3);
  write_copy(1, 2, "new", 3);
  g_drops = 0;
  spoil(copy_at(1) + 21, 'X');
  CHECK(user_is("new") && g_drops == 1);
  spoil(copy_at(1) + 0, 0x00);
  CHECK(user_is("new") && g_drops == 2);
  memset(copy_at(1), 0xFF, 0x1000);
  CHECK(user_is("old"));

  /* 16. two stages before one reset are applied together */
  memset(g_flash, 0xFF, sizeof(g_flash));
  CHECK(n6_asset_stage("base", 4, 0));
  CHECK(fsbl() == 1);
  CHECK(n6_asset_record_stage(2, "wifi", 4));
  CHECK(n6_asset_stage("cal2", 4, 0));
  CHECK(fsbl() == 1);
  CHECK(user_is("cal2") && rec_is(2, "wifi", 4));
  n = raw_payload(raw, &ver);
  CHECK(ver == 3);

  /* 17. no version above N6_ASSET_VERSION_MAX; a store at it takes no more */
  memset(g_flash, 0xFF, sizeof(g_flash));
  write_copy(0, 5, "five", 4);
  CHECK(!n6_asset_stage("x", 1, 0xFFFFFFFFu));
  CHECK(fsbl() == 0);
  CHECK(n6_asset_stage("top", 3, N6_ASSET_VERSION_MAX));
  CHECK(fsbl() == 1 && user_is("top"));
  CHECK(!n6_asset_stage("past", 4, 0));
  CHECK(!n6_asset_record_stage(2, "w", 1));
  CHECK(fsbl() == 0 && user_is("top"));
  memset(g_flash, 0xFF, sizeof(g_flash));
  write_copy(0, 0xFFFFFFFFu, "max", 3);
  CHECK(!n6_asset_stage("y", 1, 0));
  CHECK(fsbl() == 0 && user_is("max"));
  CHECK(g_susp == 0);

  /* 18. a refusal leaves the stage before it armed and whole */
  memset(g_flash, 0xFF, sizeof(g_flash));
  write_copy(0, 1, "one", 3);
  CHECK(n6_asset_stage("two", 3, 0));
  CHECK(!n6_asset_record_stage(2, big, sizeof(big)));
  CHECK(fsbl() == 1 && user_is("two"));
  CHECK(g_susp == 0);

  /* 19. apply clears the FSBL's double-press key, and no other value */
  (&TAMP->BKP0R)[N6_BOOT_PRESS_BKPR] = N6_BOOT_PRESS_KEY;
  if (setjmp(g_reset) == 0) n6_asset_apply();
  CHECK(g_resets == 1 && (&TAMP->BKP0R)[N6_BOOT_PRESS_BKPR] == 0u);
  (&TAMP->BKP0R)[N6_BOOT_PRESS_BKPR] = 0x00F4CDF4u;   /* FORCEDFU */
  if (setjmp(g_reset) == 0) n6_asset_apply();
  CHECK(g_resets == 2 && (&TAMP->BKP0R)[N6_BOOT_PRESS_BKPR] == 0x00F4CDF4u);

  printf("%d checks, %d failed\n", checks, fails);
  return fails != 0;
}
