/*
  BLE beacon

  Advertises a non-connectable BLE beacon named "Neuro-N6"
  with a counter in its manufacturer data that goes up once a second.

  Hardware: Neuro N6 with the MAYA-W476 wireless module.

  This example code is in the public domain.
*/

#include <NeuroN6.h>
#include <ArduinoBLE.h>

// 0xFFFF is the company ID set aside for testing.
uint8_t beacon[6] = { 0xFF, 0xFF, 'N', '6', 0, 0 };

void setup() {
  NeuroN6_StreamUsbOnlyStart();       // USB serial console, no camera
  Serial.begin(115200);

  if (!BLE.begin()) {
    Serial.println("[BLE] BLE.begin() FAILED - is the wireless firmware in flash?");
    while (1) { delay(1000); }
  }
  BLE.setLocalName("Neuro-N6");
  BLE.setConnectable(false);
  BLE.setManufacturerData(beacon, sizeof(beacon));
  BLE.advertise();

  Serial.print("[BLE] beacon 'Neuro-N6' from ");
  Serial.println(BLE.address());
}

void loop() {
  static uint16_t count = 0;
  delay(1000);
  count++;
  beacon[4] = (uint8_t)(count >> 8);
  beacon[5] = (uint8_t)count;
  BLE.setManufacturerData(beacon, sizeof(beacon));   // takes effect on the next advertise()
  BLE.stopAdvertise();
  BLE.advertise();
}
