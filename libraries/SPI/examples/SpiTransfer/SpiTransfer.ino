/*
  SPI transfer

  Sends a byte over SPI and prints what came back on the MISO line.

  Hardware: Neuro N6, SPI device connected to MOSI, MISO and SCK on the
  header, its chip-select pin connected to D10.

  This example code is in the public domain.
*/

#include <SPI.h>

const int chipSelectPin = D10;   // any free digital pin

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);   // deselect the device
  SPI.begin();
}

void loop() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(chipSelectPin, LOW);

  byte reply = SPI.transfer(0x00);

  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();

  Serial.print("received 0x");
  Serial.println(reply, HEX);
  delay(500);
}
