/*
  Speaker tone

  Plays a rising one-octave scale through the speaker, then streams a short
  sine sweep as raw samples.

  Hardware: Neuro N6, a speaker on the speaker connector.

  This example code is in the public domain.
*/

#include <Speaker.h>
#include <math.h>

Speaker speaker;
bool    spkOK = false;

// C4 to C5, in Hz.
static const uint16_t kScale[] = { 262, 294, 330, 349, 392, 440, 494, 523 };

void setup() {
  Serial.begin(115200);

  spkOK = speaker.begin();
  if (spkOK) {
    speaker.setVolume(1.0f);             // 0.0 to 1.0
    Serial.print("Speaker OK - ");
    Serial.print(speaker.sampleRate());
    Serial.println(" Hz.");
  } else {
    Serial.println("Speaker init FAILED (clock/SAI/DMA). See [SPKDBG] below.");
    speaker.debugDump();
  }
}

void loop() {
  if (!spkOK) { delay(1000); return; }

  for (unsigned i = 0; i < sizeof(kScale) / sizeof(kScale[0]); i++) {
    speaker.tone(kScale[i], 200);          // stops after 200 ms
    delay(260);                            // 200 ms note, 60 ms gap
  }
  speaker.noTone();
  delay(400);

  const uint32_t fs = speaker.sampleRate();
  const uint32_t total = fs / 2;           // 0.5 s of samples
  float phase = 0.0f;
  uint32_t sent = 0;
  while (sent < total) {
    int16_t chunk[128];
    uint32_t n = total - sent; if (n > 128) n = 128;
    while (speaker.availableForWrite() < n) { }
    for (uint32_t k = 0; k < n; k++) {
      float f = 660.0f + 220.0f * (float)(sent + k) / (float)total;  // 660 Hz to 880 Hz
      phase += 2.0f * 3.14159265f * f / (float)fs;
      chunk[k] = (int16_t)(32000.0f * sinf(phase));   // full scale is 32767
    }
    sent += speaker.write(chunk, n);
  }
  while (speaker.playing()) { delay(5); }
  delay(1400);
}
