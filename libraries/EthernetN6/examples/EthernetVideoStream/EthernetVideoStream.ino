/*
  Ethernet video stream

  Streams the live camera picture to Neuro Studio over wired Ethernet, at a
  fixed address. Give the computer an address on the same network, such as
  192.168.20.2 with mask 255.255.255.0, and connect Neuro Studio to the
  board's address.

  Hardware: Neuro N6 with an OV5640 camera and a LAN8742A Ethernet PHY on the
  pins the core's README lists.

  This example code is in the public domain.
*/

#include <EthernetN6.h>
#include <OV5640_Arduino.h>

// OV5640 camera(CARRIER_TFT); on the TFT carrier.
OV5640 camera;

void setup() {
  // The board's address and the port Neuro Studio connects to.
  EthernetN6.setStaticIp(IPAddress(192, 168, 20, 1), 5000);

  VisionConfig cfg;
  cfg.camera    = CAMERA_VGA;
  cfg.transport = TRANSPORT_ETHERNET;
  Vision.begin(cfg);
}

void loop() {
  // A background task sends the frames.
  delay(1000);
}
