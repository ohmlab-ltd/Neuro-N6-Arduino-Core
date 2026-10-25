/**
 ******************************************************************************
 * @file    N6EventsSD.cpp
 * @brief   SD card clip sink.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */

#include "N6EventsSD.h"
#include <string.h>
#include <stdio.h>

namespace {

struct SdSink {
  SdFat   *sd = nullptr;
  char     root[32] = "/events";
  char     dir[48]  = "";
  char     last_dir[48] = "";
  FsFile   meta;                 /* meta.bin, open between open() and close() */
  uint32_t index = 0;            /* frames written this clip */
  bool     failed = false;
  n6_event_t ev;

  uint32_t st_frames = 0, st_bytes = 0, st_failures = 0;
};

SdSink g;

bool write_all(FsFile &f, const void *p, uint32_t n)
{
  if (n == 0) return true;
  const int32_t w = (int32_t)f.write(p, n);
  return w == (int32_t)n;
}

/* Reads the clip counter from the card, 0 when missing or unreadable. */
uint32_t read_counter(const char *path)
{
  FsFile f;
  if (!f.open(path, O_RDONLY)) return 0;
  char buf[16] = {0};
  const int n = f.read(buf, sizeof(buf) - 1);
  f.close();
  if (n <= 0) return 0;
  buf[n] = '\0';
  unsigned long v = 0;
  if (sscanf(buf, "%lu", &v) != 1) return 0;
  return (uint32_t)v;
}

void write_counter(const char *path, uint32_t v)
{
  FsFile f;
  if (!f.open(path, O_WRONLY | O_CREAT | O_TRUNC)) return;
  char buf[16];
  const int n = snprintf(buf, sizeof(buf), "%lu\n", (unsigned long)v);
  (void)write_all(f, buf, (uint32_t)n);
  f.close();
}

bool sink_open(void *, const n6_event_t *ev)
{
  if (g.sd == nullptr || ev == nullptr) return false;
  g.ev = *ev;
  g.index = 0;
  g.failed = false;

  if (!g.sd->exists(g.root) && !g.sd->mkdir(g.root)) { g.st_failures++; return false; }

  char seqpath[48];
  snprintf(seqpath, sizeof(seqpath), "%s/.seq", g.root);
  uint32_t n = read_counter(seqpath);

  /* An existing folder is never reused, whatever the counter says. */
  for (uint32_t tries = 0; tries < 1000; tries++, n++) {
    snprintf(g.dir, sizeof(g.dir), "%s/%06lu", g.root, (unsigned long)n);
    if (!g.sd->exists(g.dir)) break;
  }
  if (!g.sd->mkdir(g.dir)) { g.st_failures++; return false; }
  write_counter(seqpath, n + 1);

  char path[64];
  snprintf(path, sizeof(path), "%s/meta.bin", g.dir);
  if (!g.meta.open(path, O_WRONLY | O_CREAT | O_TRUNC)) { g.st_failures++; return false; }
  return true;
}

bool sink_frame(void *, const n6_clip_frame_t *f)
{
  if (g.sd == nullptr || f == nullptr || g.failed) return false;

  char path[64];
  snprintf(path, sizeof(path), "%s/%03lu.jpg", g.dir, (unsigned long)g.index);
  FsFile jf;
  if (!jf.open(path, O_WRONLY | O_CREAT | O_TRUNC)) { g.failed = true; g.st_failures++; return false; }
  const bool ok = write_all(jf, f->jpeg, f->jpeg_len);
  jf.close();
  if (!ok) { g.failed = true; g.st_failures++; return false; }

  /* meta.bin record: 16-byte head, then the frame's TLV bytes. */
  uint8_t head[16];
  auto put32 = [&](int o, uint32_t v) { head[o] = (uint8_t)v; head[o + 1] = (uint8_t)(v >> 8); head[o + 2] = (uint8_t)(v >> 16); head[o + 3] = (uint8_t)(v >> 24); };
  put32(0, f->seq);
  put32(4, f->t_ms);
  put32(8, g.index);
  head[12] = (uint8_t)f->meta_len; head[13] = (uint8_t)(f->meta_len >> 8);
  head[14] = 0; head[15] = 0;
  if (!write_all(g.meta, head, sizeof(head)) ||
      !write_all(g.meta, f->meta, f->meta_len)) { g.failed = true; g.st_failures++; return false; }

  g.index++;
  g.st_frames++;
  g.st_bytes += f->jpeg_len + sizeof(head) + f->meta_len;
  return true;
}

bool sink_close(void *, bool ok)
{
  if (g.sd == nullptr) return false;
  if (g.meta.isOpen()) g.meta.close();

  char path[64];
  snprintf(path, sizeof(path), "%s/event.txt", g.dir);
  FsFile tf;
  bool wrote = false;
  if (tf.open(path, O_WRONLY | O_CREAT | O_TRUNC)) {
    char line[160];
    const int n = snprintf(line, sizeof(line),
      "id=%lu\nt_ms=%lu\nsources=0x%02X\nscore=%u\nframes_declared=%u\npre_frames=%u\nframes_written=%lu\nok=%d\n",
      (unsigned long)g.ev.id, (unsigned long)g.ev.t_ms, g.ev.sources, g.ev.score,
      g.ev.n_frames, g.ev.pre_frames, (unsigned long)g.index, (ok && !g.failed) ? 1 : 0);
    wrote = write_all(tf, line, (uint32_t)n);
    tf.close();
  }
  strncpy(g.last_dir, g.dir, sizeof(g.last_dir) - 1);
  g.last_dir[sizeof(g.last_dir) - 1] = '\0';
  return ok && !g.failed && wrote;
}

const n6_event_sink_t g_sink = { "sd", sink_open, sink_frame, sink_close, nullptr };

} // namespace

const n6_event_sink_t *n6_event_sink_sd(SdFat *sd, const char *root)
{
  g.sd = sd;
  if (root && *root) { strncpy(g.root, root, sizeof(g.root) - 1); g.root[sizeof(g.root) - 1] = '\0'; }
  return &g_sink;
}

const char *n6_event_sink_sd_last_dir(void) { return g.last_dir; }

void n6_event_sink_sd_stats(uint32_t *frames, uint32_t *bytes, uint32_t *failures)
{
  if (frames)   *frames   = g.st_frames;
  if (bytes)    *bytes    = g.st_bytes;
  if (failures) *failures = g.st_failures;
}
