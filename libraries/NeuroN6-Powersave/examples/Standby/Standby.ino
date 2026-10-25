/*
  Standby

  Prints the RTC time, then sleeps in Standby for 40 s. The wake is a reboot,
  so setup() runs again. The board is off USB while it sleeps: to upload
  another sketch then, double-tap reset.

  Hardware: Neuro N6.

  This example code is in the public domain.
*/

#include <NeuroN6_PowerSave.h>

#define SLEEP_MS 40000UL

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000UL) {
    delay(10);
  }

  PowerSave.begin();
  Serial.print("awake, RTC ");
  Serial.print(PowerSave.rtcTimeMs());
  Serial.println(" ms of day");
  delay(100);
}

void loop() {
  PowerSave.standby(SLEEP_MS);
}
