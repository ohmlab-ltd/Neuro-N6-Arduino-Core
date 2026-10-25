/*
  Wire scanner

  Reports the address of every device that answers on the board's two I2C
  buses. Wire is the on-board sensor bus, and Wire1 is the SDA and SCL pins on
  the header.

  Hardware: Neuro N6. Optional: an I2C sensor on the header SDA, SCL, 3V3 and
  GND pins. The board already has the pull-up resistors.

  This example code is in the public domain.
*/

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Wire.begin();
  Wire1.begin();
}

void scan(TwoWire &bus, const char *name) {
  int found = 0;

  Serial.print("scanning ");
  Serial.println(name);

  for (byte address = 1; address < 127; address++) {
    bus.beginTransmission(address);
    if (bus.endTransmission() == 0) {
      Serial.print("  device at 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println("  no devices found");
  }
}

void loop() {
  scan(Wire, "Wire (on-board sensors)");
  scan(Wire1, "Wire1 (header SDA and SCL)");
  delay(5000);
}
