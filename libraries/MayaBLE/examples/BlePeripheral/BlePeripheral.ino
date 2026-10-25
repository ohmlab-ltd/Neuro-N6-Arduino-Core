/*
  BLE peripheral

  Advertises a connectable BLE service named "Neuro-N6" with
  two values: one a phone can write a byte to, and one the board counts up
  every two seconds and notifies subscribers about.

  Hardware: Neuro N6 with the MAYA-W476 wireless module.

  This example code is in the public domain.
*/

#include <NeuroN6.h>
#include <ArduinoBLE.h>

BLEService                   svc      ("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEByteCharacteristic        ctrlChar ("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEUnsignedIntCharacteristic countChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

// Runs in the BLE task.
void onCtrlWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("[BLE] ctrl written: 0x");
  Serial.println(ctrlChar.value(), HEX);
}

void onConnect(BLEDevice central) {
  Serial.print("[BLE] connected: ");
  Serial.println(central.address());
}

void onDisconnect(BLEDevice central) {
  Serial.println("[BLE] disconnected.");
}

void setup() {
  // USB serial console, no camera.
  NeuroN6_StreamUsbOnlyStart();
  Serial.begin(115200);

  if (!BLE.begin()) {
    Serial.println("[BLE] BLE.begin() FAILED - is the wireless firmware in flash?");
    while (1) { delay(1000); }
  }

  BLE.setLocalName("Neuro-N6");
  BLE.setDeviceName("Neuro-N6");
  BLE.setAdvertisedService(svc);
  svc.addCharacteristic(ctrlChar);
  svc.addCharacteristic(countChar);
  BLE.addService(svc);
  ctrlChar.writeValue(0);
  countChar.writeValue(0);
  ctrlChar.setEventHandler(BLEWritten, onCtrlWritten);
  BLE.setEventHandler(BLEConnected, onConnect);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);
  BLE.advertise();

  Serial.print("[BLE] advertising 'Neuro-N6' as ");
  Serial.println(BLE.address());
}

void loop() {
  static uint32_t counter = 0;
  // BLE is serviced in the background, so nothing calls BLE.poll().
  delay(2000);
  countChar.writeValue(++counter);
}
