/*
  Read acceleration

  Prints the acceleration in g, the rotation rate in degrees per second and
  the chip temperature to the Serial Monitor, and sends them to Neuro Studio.

  Hardware: Neuro N6, on-board LSM6DSL sensor.

  This example code is in the public domain.
*/

#include <Wire.h>
#include <LSM6DSL_Arduino.h>
#include <PostProcess.h>     // needed by sendNS()

LSM6DSL imu;
bool     imuOK = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  imuOK = imu.begin();
  if (imuOK) {
    Serial.print("LSM6DSL found (WHO_AM_I=0x");
    Serial.print(imu.whoami(), HEX);
    Serial.println(")");
  } else {
    Serial.println("LSM6DSL NOT found - check it is fitted (I2C address 0x6A or 0x6B)");
  }
}

void loop() {
  if (!imuOK) {
    imuOK = imu.begin();
    if (!imuOK) Serial.println("LSM6DSL NOT found - check it is fitted (I2C address 0x6A or 0x6B)");
    delay(1000);
    return;
  }

  imuVec3 a = imu.readAccel();        // g
  imuVec3 g = imu.readGyro();         // degrees per second

  Serial.print("accel[g] X="); Serial.print(a.x, 3);
  Serial.print(" Y=");         Serial.print(a.y, 3);
  Serial.print(" Z=");         Serial.print(a.z, 3);
  Serial.print("   gyro[dps] X="); Serial.print(g.x, 1);
  Serial.print(" Y=");             Serial.print(g.y, 1);
  Serial.print(" Z=");             Serial.print(g.z, 1);
  Serial.print("   temp=");        Serial.print(imu.readTemp(), 1); Serial.println(" C");

  imu.sendNS();   // to Neuro Studio

  delay(200);
}
