/*
  Speaker clip

  Loops an audio clip built into the sketch through the speaker.

  Hardware: Neuro N6, a speaker on the speaker connector.

  This example code is in the public domain.
*/

#include <Speaker.h>
// kSong[], kSongLen and kSongRate: a synthesised chime, in the format
// clip_to_header.py writes. It makes one from an audio file, for example:
//   python clip_to_header.py clip.mp3 --start 65 --kb 400 --rate 22050
#include "song.h"

Speaker  speaker;
uint32_t pos = 0;

void setup() {
  Serial.begin(115200);
  bool ok = speaker.begin(kSongRate);
  speaker.setVolume(1.0f);              // 0.0 to 1.0
  if (ok) {
    Serial.print("Playing clip: ");
    Serial.print(kSongLen);
    Serial.print(" samples @ ");
    Serial.print(speaker.sampleRate());
    Serial.println(" Hz (looping).");
  } else {
    Serial.println("Speaker init FAILED - see [SPKDBG].");
    speaker.debugDump();
  }
}

void loop() {
  uint32_t space = speaker.availableForWrite();
  if (space == 0) return;
  uint32_t n = kSongLen - pos;
  if (n > space) n = space;
  if (n > 1024)  n = 1024;
  pos += speaker.write(&kSong[pos], n);
  if (pos >= kSongLen) pos = 0;
}
