/*
  Sleep modes

  Sleeps with sleepLight(), then with sleepStop(), and prints how long each
  lasted by the RTC. The RGB LED is green while awake, blue in sleepLight()
  and off in sleepStop(). It flashes red when sleepStop() ends early.

  Hardware: Neuro N6.

  This example code is in the public domain.
*/

#include <NeuroN6_PowerSave.h>

#define SLEEP_MS 3000UL

// RTC milliseconds since startMs, across midnight.
static uint32_t rtcSince(uint32_t startMs) {
  return (PowerSave.rtcTimeMs() + 86400000UL - startMs) % 86400000UL;
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000UL) {
    delay(10);
  }

  PowerSave.begin();
}

void loop() {
  RGB.green();
  Serial.print("RTC ");
  Serial.print(PowerSave.rtcTimeMs());
  Serial.println(" ms of day");
  delay(1000);

  // Tasks and USB keep running.
  RGB.blue();
  uint32_t start = PowerSave.rtcTimeMs();
  PowerSave.sleepLight(SLEEP_MS);
  RGB.green();
  Serial.print("sleepLight() ");
  Serial.print(rtcSince(start));
  Serial.println(" ms");

  // false when an interrupt ends it early. With a USB host attached it sleeps
  // with sleepLight() and returns false.
  RGB.off();
  start = PowerSave.rtcTimeMs();
  bool full = PowerSave.sleepStop(SLEEP_MS);
  uint32_t slept = rtcSince(start);
  if (full) {
    RGB.green();
  } else {
    RGB.red();
    delay(200);
  }
  Serial.print("sleepStop() ");
  Serial.print(slept);
  Serial.println(full ? " ms" : " ms, ended early");
}
