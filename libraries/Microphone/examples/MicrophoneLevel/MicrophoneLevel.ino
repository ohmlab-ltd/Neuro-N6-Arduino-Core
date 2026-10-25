/*
  Microphone level

  Prints the on-board microphone's level to the Serial Monitor. The audio
  streams to Neuro Studio.

  Hardware: Neuro N6, on-board microphone.

  This example code is in the public domain.
*/

#include <Microphone.h>

Microphone mic;
bool       micOK = false;

void setup() {
  Serial.begin(115200);

  micOK = mic.begin();
  if (micOK) {
    Serial.print("Microphone OK - ");
    Serial.print(mic.sampleRate());
    Serial.println(" Hz, 16-bit mono. Listen in Neuro Studio (speaker icon).");
  } else {
    Serial.println("Microphone did not start (Tools > Debug console: On prints [MICDBG] details).");
  }
}

void loop() {
  if (micOK) {
    Serial.print("mic level=");
    Serial.println(mic.peak());   // 0 to 32767
  }
  delay(200);
}
