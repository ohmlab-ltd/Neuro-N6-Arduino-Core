/*
  Camera stream

  Streams the live camera picture over USB to Neuro Studio, with no neural
  network and no overlay. Every camera setting is written out.

  Hardware: Neuro N6 with an OV5640 camera.

  This example code is in the public domain.
*/

#include <OV5640_Arduino.h>

// OV5640 camera(CARRIER_TFT); on the TFT carrier.
OV5640 camera;

void setup() {
  VisionConfig cfg;

  //   CAMERA_WVGA   800x480
  //   CAMERA_VGA    640x480
  //   CAMERA_XGA    1024x768
  //   CAMERA_720P   1280x720   narrower view
  //   CAMERA_1080P  1920x1080  narrower view
  //   CAMERA_QSXGA  2592x1944  about 15 frames per second
  // The TFT carrier's display switches itself off above 800x480.
  cfg.camera = CAMERA_XGA;

  //   CAMERA_AUTO_FLIP        upright on either carrier
  //   CAMERA_MIRROR_FLIP_NONE the sensor's own orientation
  //   CAMERA_FLIP             upside down
  //   CAMERA_MIRROR           left and right swapped
  //   CAMERA_MIRROR_FLIP      both
  cfg.mirror_flip = CAMERA_AUTO_FLIP;

  // FPS_DEFAULT is the fastest the resolution can manage.
  cfg.fps    = FPS_DEFAULT;

  // Neural network input. This sketch runs no model, so these change nothing.
  cfg.nn.width  = 256;
  cfg.nn.height = 256;
  cfg.nn.aspect = ASPECT_STRETCH;  // STRETCH, CROP_CENTER, CROP_OFFSET or LETTERBOX
  cfg.nn.crop_x = 0;               // crop area, in camera pixels, for CROP_OFFSET
  cfg.nn.crop_y = 0;
  cfg.nn.crop_w = 0;               // 0 means no crop
  cfg.nn.crop_h = 0;

  //   TRANSPORT_USB           USB to Neuro Studio
  //   TRANSPORT_WIFI_SOFTAP   WiFi as an access point, needs the WiFi library
  //   TRANSPORT_LCD           the TFT carrier's display
  cfg.transport = TRANSPORT_USB;

  // Autofocus, on camera modules that have a moving lens.
  //   AF_DISABLED    focus left where it is
  //   AF_CONTINUOUS  refocuses as the scene changes
  //   AF_SINGLE      focuses once at start-up
  cfg.af = AF_CONTINUOUS;

  // STROBE_ENABLED pulses an LED on the sensor strobe pin with each exposure.
  cfg.strobe = STROBE_DISABLED;

  Vision.begin(cfg);
}

void loop() {
  // A background task sends the frames.
  delay(1000);
}
