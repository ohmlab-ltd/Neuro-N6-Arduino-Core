/*
  AudioLoopback

  Plays tones through the speaker and measures them with the on-board
  microphone. The microphone's sample rate is first counted against the CPU
  clock.

  Hardware: Neuro N6, on-board speaker and microphone.

  Copyright (c) 2026 Ohm Lab Ltd.
  SPDX-License-Identifier: MIT
*/

#include <Microphone.h>
#include <Speaker.h>
#include <math.h>

Microphone mic;
Speaker speaker;

static const int N = 8192;                 // ~0.25 s at 32552 Hz
static int16_t g_buf[N];
static int16_t g_room[N];

static void drain() {
  int16_t junk[512];
  while (mic.available() > 0) mic.read(junk, 512);
}

static void collect(int16_t *dst) {
  int got = 0;
  while (got < N) {
    got += (int)mic.read(dst + got, (uint32_t)(N - got));
    if (got < N) delay(2);
  }
}

static float goertzel(const int16_t *x, float f, float fs) {
  const float c = 2.0f * cosf(2.0f * (float)M_PI * f / fs);
  float s1 = 0.0f, s2 = 0.0f;
  for (int i = 0; i < N; i++) {
    const float s0 = (float)x[i] + c * s1 - s2;
    s2 = s1;
    s1 = s0;
  }
  return s1 * s1 + s2 * s2 - c * s1 * s2;
}

static float rms(const int16_t *x) {
  double m = 0.0, s = 0.0;
  for (int i = 0; i < N; i++) m += x[i];
  m /= N;
  for (int i = 0; i < N; i++) { const double d = x[i] - m; s += d * d; }
  return (float)sqrt(s / N);
}

void setup() {
  Serial.begin(115200);
  if (!mic.begin()) Serial.println("[aud] microphone begin FAILED");
  mic.streamToNS(false);                   // the samples stay in this sketch
  if (!speaker.begin()) Serial.println("[aud] speaker begin FAILED");
  speaker.setVolume(0.25f);
}

void loop() {
  static const uint32_t tones[] = {440, 1000, 3000, 6000, 10000};
  char line[128];

  drain();
  uint32_t counted = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < 2000u) {
    counted += mic.read(g_buf, N);
    delay(5);
  }
  // The count resolves about 1%, the accuracy of the internal oscillator.
  const float counted_fs = counted / 2.0f;
  const float nominal_fs = (float)mic.sampleRate();
  const bool agrees = fabsf(counted_fs - nominal_fs) <= 0.01f * nominal_fs;
  const float fs = agrees ? nominal_fs : counted_fs;
  snprintf(line, sizeof(line), "[aud] speaker says %lu Hz, microphone says %lu Hz, counted %lu Hz (%s)",
           (unsigned long)speaker.sampleRate(), (unsigned long)mic.sampleRate(),
           (unsigned long)(counted_fs + 0.5f),
           agrees ? "within 1%: tones measured at the nominal rate" : "OFF: tones measured at the counted rate");
  Serial.println(line);
  speaker.debugDump();

  drain();
  collect(g_room);
  snprintf(line, sizeof(line), "[aud] room: rms %.1f", rms(g_room));
  Serial.println(line);

  for (uint32_t f : tones) {
    speaker.tone(f);
    delay(250);                             // the tone and the room settle
    drain();
    collect(g_buf);
    speaker.noTone();

    float best_f = 0.0f, best_p = 0.0f;
    for (float g = 100.0f; g <= 15000.0f; g += 10.0f) {
      const float p = goertzel(g_buf, g, fs);
      if (p > best_p) { best_p = p; best_f = g; }
    }
    const float lo = best_f - 10.0f, hi = best_f + 10.0f;
    for (float g = lo; g <= hi; g += 0.5f) {
      const float p = goertzel(g_buf, g, fs);
      if (p > best_p) { best_p = p; best_f = g; }
    }
    const float room = goertzel(g_room, best_f, fs);
    const float db = 10.0f * log10f((best_p + 1.0f) / (room + 1.0f));
    const float err = 100.0f * (best_f - (float)f) / (float)f;
    snprintf(line, sizeof(line), "[aud] tone %lu Hz: heard %.1f Hz (%+.2f%%), %.1f dB above the room",
             (unsigned long)f, best_f, err, db);
    Serial.println(line);
    delay(150);
  }
  Serial.println("[aud] ---");
  delay(8000);
}
