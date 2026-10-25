/*
  Read distance

  Reads the on-board VL53L1CB as a 4x4 grid of distances in millimetres,
  prints it and sends it to Neuro Studio.

  Hardware: Neuro N6.

  This example code is in the public domain.
*/

#include <Wire.h>
#include <VL53L1CB_Arduino.h>
#include <PostProcess.h>     // for sendNS()

VL53L1CB tof;
bool      tofOK = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  tofOK = tof.begin();
  if (tofOK) {
    Serial.print("VL53L1CB found (WHO_AM_I=0x");
    Serial.print(tof.whoami(), HEX);
    Serial.println(")");
  } else {
    Serial.println("VL53L1CB NOT found - check it is fitted");
  }
}

void loop() {
  if (!tofOK) {
    tofOK = tof.begin();
    if (!tofOK) Serial.println("VL53L1CB NOT found - check it is fitted");
    delay(1000);
    return;
  }

  // Each call starts or collects one zone. A full grid takes about 1 s.
  tofFrame f = tof.read();

  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      Serial.print(f.mm[row][col]);    // mm, 0 = no target
      Serial.print('\t');
    }
    Serial.println();
  }
  Serial.println();

  tof.sendNS();

  delay(100);
}
