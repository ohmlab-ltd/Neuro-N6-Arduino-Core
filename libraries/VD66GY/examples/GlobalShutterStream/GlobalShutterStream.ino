/*
  Global shutter stream

  Streams the global-shutter camera over USB to Neuro Studio, with no neural
  network and no overlay. The camera settings are written out.

  Hardware: Neuro N6 with a VD66GY camera on the Neuro Vision carrier.

  This example code is in the public domain.
*/

#include <VD66GY_Arduino.h>

// VD66GY camera(VD66GY_CARRIER_THERMAL); on the Thermal carrier.
VD66GY camera;

void setup() {
  VisionConfig cfg;

  //   CAMERA_QVGA       320x240    88 fps
  //   CAMERA_VGA        640x480    88 fps
  //   CAMERA_XGA        1024x768   60 fps
  //   CAMERA_720P       1120x720   60 fps
  //   CAMERA_VD66_FULL  1120x1364  45 fps, the whole sensor
  cfg.camera = CAMERA_VGA;

  // FPS_DEFAULT is the fastest the resolution can manage.
  cfg.fps = FPS_DEFAULT;

  //   CAMERA_AUTO_FLIP        the sensor's own orientation
  //   CAMERA_FLIP             upside down
  //   CAMERA_MIRROR           left and right swapped
  //   CAMERA_MIRROR_FLIP      both
  cfg.mirror_flip = CAMERA_AUTO_FLIP;

  //   isp_tuning_disabled     fixed white balance
  //   VD66_isp_tuning_auto    automatic white balance
  //   VD66_isp_tuning_manual  colour settings set from Neuro Studio
  cfg.isp_enabled = VD66_isp_tuning_auto;

  // STROBE_ENABLED lights the Neuro Vision carrier's strobe LED during each
  // exposure. The Thermal carrier has no strobe LED.
  cfg.strobe = STROBE_DISABLED;

  // The VD66GY carriers have no display and no WiFi.
  cfg.transport = TRANSPORT_USB;

  // The brightness the sensor's automatic exposure aims for, 0 to 100.
  VD66GY::setAeTargetPercent(25);

  Vision.begin(cfg);
}

void loop() {
  // A background task sends the frames.
  delay(1000);
}
