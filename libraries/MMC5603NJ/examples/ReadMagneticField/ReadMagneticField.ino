/*
  Read magnetic field

  Prints the field on each axis in microtesla, a rough compass heading and the
  chip temperature to the Serial Monitor, and sends them to Neuro Studio.

  Hardware: Neuro N6, on-board MMC5603NJ sensor.

  This example code is in the public domain.
*/

#include <Wire.h>
#include <MMC5603NJ.h>
#include <PostProcess.h>     // needed by sendNS()
#include <math.h>

MMC5603NJ mag;
bool      magOK = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  magOK = mag.begin();
  if (magOK) {
    Serial.print("MMC5603NJ found (WHO_AM_I=0x");
    Serial.print(mag.whoami(), HEX);
    Serial.println(")");
  } else {
    Serial.println("MMC5603NJ NOT found - is it fitted? (I2C address 0x30)");
  }
}

void loop() {
  if (!magOK) {
    magOK = mag.begin();
    if (!magOK) Serial.println("MMC5603NJ NOT found - is it fitted? (I2C address 0x30)");
    delay(1000);
    return;
  }

  magSensorFloatXYZ m = mag.getMag(); // microtesla

  // Not calibrated for nearby metal or magnets.
  float heading = atan2(m.y, m.x) * 57.29578f;   // radians into degrees
  if (heading < 0.0f) heading += 360.0f;

  Serial.print("X=");          Serial.print(m.x, 2);
  Serial.print("  Y=");        Serial.print(m.y, 2);
  Serial.print("  Z=");        Serial.print(m.z, 2); Serial.print(" uT");
  Serial.print("   heading="); Serial.print(heading, 1); Serial.print(" deg");
  Serial.print("   temp=");    Serial.print(mag.getTemp(), 1); Serial.println(" C");

  mag.sendNS();   // to Neuro Studio

  delay(200);
}
