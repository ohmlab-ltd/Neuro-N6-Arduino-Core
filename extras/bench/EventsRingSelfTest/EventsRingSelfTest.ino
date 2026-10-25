/**
 ******************************************************************************
 * EventsRingSelfTest.ino
 *
 * Checks the event pipeline's bookkeeping on synthetic input, with no camera
 * and no SD card. Each case prints ok or FAIL with its name, and the closing
 * RESULT line gives the number of failures.
 *
 * Hardware: Neuro N6.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#include <N6Events.h>
#include <string.h>

#define BUF_BYTES 48000u
static uint8_t g_buf[BUF_BYTES] __attribute__((aligned(4)));

static int failures = 0;

static void ok(const char *name, bool pass, const char *detail)
{
  if (!pass) failures++;
  Serial.print("  ");
  Serial.print(pass ? "ok   " : "FAIL ");
  Serial.print(name);
  if (detail && *detail) { Serial.print("   "); Serial.print(detail); }
  Serial.println();
}

static void fill(uint8_t *p, uint32_t len, uint32_t seq)
{
  for (uint32_t i = 0; i < len; i++) p[i] = (uint8_t)(seq * 31u + i * 7u + 3u);
}
static bool verify(const uint8_t *p, uint32_t len, uint32_t seq)
{
  for (uint32_t i = 0; i < len; i++)
    if (p[i] != (uint8_t)(seq * 31u + i * 7u + 3u)) return false;
  return true;
}

static uint8_t g_frame[6000];

/* Ring */

static bool walk_check(n6_ring_t *r, uint32_t expect_count, uint32_t expect_last_seq,
                       uint32_t jpeg_len, char *why, size_t why_n)
{
  n6_ring_iter_t it; n6_ring_rec_t rec;
  uint32_t n = 0, last = 0; bool first = true;
  n6_ring_iter_begin(r, &it);
  while (n6_ring_iter_next(&it, &rec)) {
    if (!first && rec.seq != last + 1u) { snprintf(why, why_n, "seq %lu after %lu", (unsigned long)rec.seq, (unsigned long)last); return false; }
    if (rec.jpeg_len != jpeg_len)       { snprintf(why, why_n, "len %lu", (unsigned long)rec.jpeg_len); return false; }
    if (!verify(rec.jpeg, rec.jpeg_len, rec.seq)) { snprintf(why, why_n, "bytes of seq %lu", (unsigned long)rec.seq); return false; }
    last = rec.seq; first = false; n++;
  }
  if (n != expect_count) { snprintf(why, why_n, "count %lu, expected %lu", (unsigned long)n, (unsigned long)expect_count); return false; }
  if (n && last != expect_last_seq) { snprintf(why, why_n, "last seq %lu, expected %lu", (unsigned long)last, (unsigned long)expect_last_seq); return false; }
  why[0] = '\0';
  return true;
}

static void test_ring(void)
{
  n6_ring_t r;
  char why[80];
  n6_ring_stats_t st;

  Serial.println("[ring]");

  ok("init", n6_ring_init(&r, g_buf, BUF_BYTES), "");
  n6_ring_stats(&r, &st);
  ok("empty", st.count == 0 && st.used == 0, "");

  for (uint32_t s = 0; s < 3; s++) {
    fill(g_frame, 1000, s);
    ok("push", n6_ring_push(&r, g_frame, 1000, NULL, 0, 100u * (s + 1u), 0), "");
  }
  ok("three in order", walk_check(&r, 3, 2, 1000, why, sizeof why), why);
  ok("window [150,250] = 1", n6_ring_count_window(&r, 150, 250) == 1, "");
  ok("window [0,1000] = 3",  n6_ring_count_window(&r, 0, 1000) == 3, "");
  ok("window wrap-safe", n6_ring_in_window(5u, 0xFFFFFFF0u, 10u) && !n6_ring_in_window(20u, 0xFFFFFFF0u, 10u), "");

  {
    uint8_t meta[37]; for (int i = 0; i < 37; i++) meta[i] = (uint8_t)(200 - i);
    fill(g_frame, 500, 3);
    n6_ring_push(&r, g_frame, 500, meta, 37, 400, 0x5A5A);
    n6_ring_iter_t it; n6_ring_rec_t rec; bool found = false;
    n6_ring_iter_begin(&r, &it);
    while (n6_ring_iter_next(&it, &rec)) {
      if (rec.seq == 3) found = (rec.meta_len == 37 && rec.meta && memcmp(rec.meta, meta, 37) == 0 && rec.flags == 0x5A5A && rec.t_ms == 400);
    }
    ok("meta + flags round trip", found, "");
  }

  ok("too large rejected", !n6_ring_push(&r, g_frame, BUF_BYTES, NULL, 0, 0, 0), "");
  ok("empty rejected",     !n6_ring_push(&r, g_frame, 0, NULL, 0, 0, 0), "");
  n6_ring_pause(&r, true);
  ok("paused rejects",     !n6_ring_push(&r, g_frame, 100, NULL, 0, 0, 0), "");
  n6_ring_stats(&r, &st);
  ok("paused counted",     st.dropped_paused == 1, "");
  n6_ring_pause(&r, false);
  fill(g_frame, 100, 4);
  ok("unpaused accepts",   n6_ring_push(&r, g_frame, 100, NULL, 0, 0, 0), "");

  /* Implicit wrap: a record is align4(20 + 980) = 1000 bytes, so 40 of them
   * leave 8 bytes, less than a header. */
  {
    const uint32_t JL = 980, CAP = 40u * 1000u + 8u;
    ok("init 40008", n6_ring_init(&r, g_buf, CAP), "");
    for (uint32_t s = 0; s < 40; s++) { fill(g_frame, JL, s); n6_ring_push(&r, g_frame, JL, NULL, 0, s, 0); }
    ok("40 fill exactly", walk_check(&r, 40, 39, JL, why, sizeof why), why);
    n6_ring_stats(&r, &st);
    ok("head at 40000", r.head == 40000u && st.used == 40000u, "");
    fill(g_frame, JL, 40); n6_ring_push(&r, g_frame, JL, NULL, 0, 40, 0);
    ok("41st: implicit wrap, one evicted", walk_check(&r, 40, 40, JL, why, sizeof why), why);
    n6_ring_stats(&r, &st);
    ok("used == cap exactly", st.used == CAP, "");
    ok("evicted == 1", st.evicted == 1, "");
    for (uint32_t s = 41; s < 160; s++) { fill(g_frame, JL, s); n6_ring_push(&r, g_frame, JL, NULL, 0, s, 0); }
    ok("160 pushes, still 40 intact", walk_check(&r, 40, 159, JL, why, sizeof why), why);
    n6_ring_stats(&r, &st);
    ok("evicted == pushed - count", st.evicted == st.pushed - st.count, "");
    ok("no corruption", st.corrupt == 0, "");
  }

  /* Explicit skip: 40 records leave 100 bytes, enough for a header. */
  {
    const uint32_t JL = 980, CAP = 40u * 1000u + 100u;
    ok("init 40100", n6_ring_init(&r, g_buf, CAP), "");
    for (uint32_t s = 0; s < 41; s++) { fill(g_frame, JL, s); n6_ring_push(&r, g_frame, JL, NULL, 0, s, 0); }
    ok("41st: explicit SKIP, one evicted", walk_check(&r, 40, 40, JL, why, sizeof why), why);
    n6_ring_stats(&r, &st);
    ok("used == cap exactly", st.used == CAP, "");
    n6_ring_hdr_t h; memcpy(&h, g_buf + 40000u, sizeof h);
    ok("SKIP header at 40000", h.magic == N6_RING_MAGIC_SKIP && h.jpeg_len == 100u, "");
    for (uint32_t s = 41; s < 130; s++) { fill(g_frame, JL, s); n6_ring_push(&r, g_frame, JL, NULL, 0, s, 0); }
    ok("130 pushes, still 40 intact", walk_check(&r, 40, 129, JL, why, sizeof why), why);
  }

  {
    ok("init mixed", n6_ring_init(&r, g_buf, 10000), "");
    for (uint32_t s = 0; s < 8; s++) { fill(g_frame, 500, s); n6_ring_push(&r, g_frame, 500, NULL, 0, s, 0); }  /* 8 x 520 = 4160 */
    fill(g_frame, 5000, 8);
    ok("large push", n6_ring_push(&r, g_frame, 5000, NULL, 0, 8, 0), "");   /* 5020 more is 9180 <= 10000: no eviction */
    n6_ring_stats(&r, &st);
    ok("9 held, none evicted", st.count == 9 && st.evicted == 0, "");
    fill(g_frame, 5000, 9);
    n6_ring_push(&r, g_frame, 5000, NULL, 0, 9, 0);   /* wrap: skip 820, need 5020 -> evicts until free */
    n6_ring_iter_t it; n6_ring_rec_t rec; uint32_t n = 0; bool intact = true;
    n6_ring_iter_begin(&r, &it);
    while (n6_ring_iter_next(&it, &rec)) { n++; if (!verify(rec.jpeg, rec.jpeg_len, rec.seq)) intact = false; }
    n6_ring_stats(&r, &st);
    ok("mixed: all remaining intact", intact && n == st.count && st.used <= st.cap, "");
    ok("mixed: newest present", n > 0 && rec.seq == 9, "");
  }

  n6_ring_clear(&r);
  n6_ring_stats(&r, &st);
  ok("clear", st.count == 0 && st.used == 0, "");
}

/* META_BOXES parser */

static void test_boxes(void)
{
  Serial.println("[boxes]");
  /* TLV stream: [type 1, len 2, ..] [type 2 = BOXES, len 1+3*10, count 3,
   * scores 10/60/90] [type 0x17, len 3, "abc"] */
  uint8_t tlv[3 + 2 + 3 + 31 + 3 + 3];
  uint32_t i = 0;
  tlv[i++] = 1; tlv[i++] = 2; tlv[i++] = 0; tlv[i++] = 0x11; tlv[i++] = 0x22;
  tlv[i++] = 2; tlv[i++] = 31; tlv[i++] = 0; tlv[i++] = 3;
  const uint8_t scores[3] = { 10, 60, 90 };
  for (int b = 0; b < 3; b++) { for (int k = 0; k < 8; k++) tlv[i++] = (uint8_t)k; tlv[i++] = scores[b]; tlv[i++] = (uint8_t)b; }
  tlv[i++] = 0x17; tlv[i++] = 3; tlv[i++] = 0; tlv[i++] = 'a'; tlv[i++] = 'b'; tlv[i++] = 'c';
  const uint32_t len = i;

  ok("min 0 -> 3",   n6_events_count_boxes(tlv, len, 0)  == 3, "");
  ok("min 50 -> 2",  n6_events_count_boxes(tlv, len, 50) == 2, "");
  ok("min 91 -> 0",  n6_events_count_boxes(tlv, len, 91) == 0, "");
  ok("truncated mid-box is safe", n6_events_count_boxes(tlv, 5 + 3 + 15, 0) <= 1, "");
  ok("truncated header is safe",  n6_events_count_boxes(tlv, 2, 0) == 0, "");
  ok("count > sent is clamped",   ({ uint8_t t2[9] = {2, 6, 0, 9, 0,0,0,0,0}; n6_events_count_boxes(t2, 9, 0) == 0; }), "");
  ok("NULL is 0", n6_events_count_boxes(NULL, 100, 0) == 0, "");
}

/* Motion gate */

static uint8_t g_rgb[64 * 48 * 3];
static uint8_t g_gate_mem[2 * 32 * 24];

static void test_gate(void)
{
  Serial.println("[motion gate]");
  n6_motion_gate_t g;
  ok("begin", n6_motion_gate_begin(&g, 32, 24, g_gate_mem), "");

  memset(g_rgb, 90, sizeof g_rgb);
  ok("first frame scores 0", n6_motion_gate_score(&g, g_rgb, 64, 48) == 0, "");
  ok("same frame scores 0",  n6_motion_gate_score(&g, g_rgb, 64, 48) == 0, "");

  /* Half the frame changes: about 500 per mille. */
  for (int y = 0; y < 48; y++) for (int x = 0; x < 32; x++) { uint8_t *p = g_rgb + (y * 64 + x) * 3; p[0] = p[1] = p[2] = 220; }
  const uint16_t s = n6_motion_gate_score(&g, g_rgb, 64, 48);
  char d[40]; snprintf(d, sizeof d, "score=%u", s);
  ok("half changed ~500", s >= 450 && s <= 550, d);

  /* Hysteresis: on at >=30, stays on until <=10. */
  n6_motion_gate_reset(&g);
  memset(g_rgb, 90, sizeof g_rgb);
  n6_motion_gate_update(&g, g_rgb, 64, 48);                 /* baseline */
  for (int y = 0; y < 48; y++) for (int x = 0; x < 4; x++) { uint8_t *p = g_rgb + (y * 64 + x) * 3; p[0] = p[1] = p[2] = 220; }  /* ~62 per mille */
  ok("on above threshold", n6_motion_gate_update(&g, g_rgb, 64, 48), "");
  ok("still on, unchanged frame (score 0 <= off)", !n6_motion_gate_update(&g, g_rgb, 64, 48), "");
}

/* Lifecycle, with a recording sink */

typedef struct {
  int opened, closed, frames;
  bool close_ok;
  uint32_t t0, t1;         /* window the sink expects */
  bool all_in_window, seq_monotonic;
  uint32_t last_seq;
  n6_event_t ev;
} rec_sink_t;

static bool rs_open(void *c, const n6_event_t *ev)
{ rec_sink_t *s = (rec_sink_t *)c; s->opened++; s->ev = *ev; s->frames = 0; s->all_in_window = true; s->seq_monotonic = true; return true; }
static bool rs_frame(void *c, const n6_clip_frame_t *f)
{
  rec_sink_t *s = (rec_sink_t *)c;
  if (!n6_ring_in_window(f->t_ms, s->t0, s->t1)) s->all_in_window = false;
  if (s->frames && f->seq <= s->last_seq) s->seq_monotonic = false;
  s->last_seq = f->seq; s->frames++;
  return f->jpeg != NULL && f->jpeg_len > 0;
}
static bool rs_close(void *c, bool okk) { rec_sink_t *s = (rec_sink_t *)c; s->closed++; s->close_ok = okk; return true; }

static rec_sink_t g_rs;

static void pump(uint32_t ms)
{
  const uint32_t until = millis() + ms;
  while ((int32_t)(until - millis()) > 0) {
    fill(g_frame, 300, 0);
    n6_events_push(g_frame, 300, NULL, 0, n6_events_now());
    n6_events_service();
    delay(20);
  }
}

static void test_lifecycle(void)
{
  Serial.println("[lifecycle]");
  n6_events_cfg_t ec = N6_EVENTS_CFG_DEFAULT;
  ec.clip_fps = 0; ec.pre_ms = 200; ec.post_ms = 200; ec.refractory_ms = 300;
  ec.meta_max = 16; ec.announce_ms = 1000;
  ok("begin_buf", n6_events_begin_buf(&ec, g_buf, BUF_BYTES), n6_events_last_error());

  n6_event_sink_t sk = { "rec", rs_open, rs_frame, rs_close, &g_rs };
  memset(&g_rs, 0, sizeof g_rs);
  ok("add sink", n6_events_add_sink(&sk), "");
  ok("idle", n6_events_state() == N6_EVT_IDLE, "");

  { uint8_t m[20] = {0}; fill(g_frame, 300, 0); n6_events_push(g_frame, 300, m, 20, n6_events_now());
    n6_events_stats_t st; n6_events_stats(&st); ok("meta over cap dropped + counted", st.tap_meta_dropped == 1, ""); }

  pump(300);                                       /* pre-roll history */
  const uint32_t t_fire = n6_events_now();
  ok("fire", n6_events_fire(N6_EVT_SRC_MOTION, 500), "");
  ok("post-roll", n6_events_state() == N6_EVT_POST_ROLL, "");
  ok("fold", n6_events_fire(N6_EVT_SRC_NN, 900) && n6_events_current()->sources == (N6_EVT_SRC_MOTION | N6_EVT_SRC_NN) && n6_events_current()->score == 900, "");
  g_rs.t0 = t_fire - 200; g_rs.t1 = t_fire + 200;

  uint32_t guard = 0;
  while (n6_events_state() == N6_EVT_POST_ROLL && guard++ < 100) pump(20);
  ok("wrote and entered refractory", n6_events_state() == N6_EVT_REFRACTORY, "");
  ok("sink opened once", g_rs.opened == 1 && g_rs.closed == 1 && g_rs.close_ok, "");
  { char d[48]; snprintf(d, sizeof d, "frames=%d", g_rs.frames);
    ok("clip has frames, all in window, in order", g_rs.frames > 5 && g_rs.all_in_window && g_rs.seq_monotonic, d); }
  { const n6_event_t *e = n6_events_current();
    ok("event counts agree with sink", e->n_frames == (uint16_t)g_rs.frames && e->pre_frames > 0 && e->pre_frames < e->n_frames, "");
    ok("sinks_ok bit 0", e->sinks_ok == 0x01, ""); }
  { n6_events_stats_t st; n6_events_stats(&st);
    ok("one event, one clip, one fold", st.events == 1 && st.clips == 1 && st.fires_folded == 1, "");
    ok("no corruption after write", st.ring.corrupt == 0, ""); }
  fill(g_frame, 300, 0);
  ok("ring accepts after write", n6_events_push(g_frame, 300, NULL, 0, n6_events_now()), "");

  { uint8_t v[N6_EVENTS_TLV_BYTES]; n6_events_encode_tlv(v);
    const uint32_t id = v[0] | (v[1] << 8) | (v[2] << 16) | ((uint32_t)v[3] << 24);
    ok("tlv id/sources/state", id == 1 && v[8] == (N6_EVT_SRC_MOTION | N6_EVT_SRC_NN) && v[9] == (uint8_t)N6_EVT_REFRACTORY, "");
    ok("tlv score 900", (v[10] | (v[11] << 8)) == 900, "");
    ok("tlv pre+post == frames", (uint32_t)((v[12] | (v[13] << 8)) + (v[14] | (v[15] << 8))) == n6_events_current()->n_frames, ""); }

  ok("fire in refractory is deferred", n6_events_fire(N6_EVT_SRC_GPIO, 100) && n6_events_state() == N6_EVT_REFRACTORY, "");
  /* No pump() here: service() moves REFRACTORY to IDLE in one call and starts
   * a pending event on the following call. */
  guard = 0;
  while (n6_events_state() == N6_EVT_REFRACTORY && guard++ < 100) { n6_events_service(); delay(20); }
  ok("refractory ended", n6_events_state() == N6_EVT_IDLE, "");
  n6_events_service();                                                    /* idle: pending starts */
  ok("pending started event 2", n6_events_state() == N6_EVT_POST_ROLL && n6_events_current()->id == 2 && n6_events_current()->sources == N6_EVT_SRC_GPIO, "");
  guard = 0;
  while (n6_events_state() != N6_EVT_IDLE && guard++ < 200) pump(20);
  { n6_events_stats_t st; n6_events_stats(&st);
    ok("two events, two clips", st.events == 2 && st.clips == 2 && st.fires_pending == 1, ""); }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 4000) { }
}

void loop()
{
  failures = 0;
  Serial.println();
  Serial.println("=== N6Events self-test (no camera, no card) ===");
  test_ring();
  test_boxes();
  test_gate();
  test_lifecycle();
  Serial.println();
  Serial.print("RESULT: ");
  if (failures == 0) Serial.println("all event-pipeline cases match their known answers");
  else { Serial.print(failures); Serial.println(" case(s) FAILED"); }
  Serial.println("=== end ===");
  delay(5000);
}
