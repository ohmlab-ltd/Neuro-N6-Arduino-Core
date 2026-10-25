/*
  Global shutter exposure

  Holds a short, fixed exposure lit by the strobe LED, so that moving
  subjects stay sharp. Sliders in Neuro Studio set the exposure and the gain,
  or hand both back to the sensor's automatic exposure. Once a second the
  sketch reports the exposure and gain the sensor applies.

  Hardware: Neuro N6 with a VD66GY camera on the Neuro Vision carrier, which
  has the strobe LED.

  This example code is in the public domain.
*/

#include <VD66GY_Arduino.h>
#include <PostProcess.h>

// On the Thermal carrier, which has no strobe LED, declare
// VD66GY camera(VD66GY_CARRIER_THERMAL); and set cfg.strobe to STROBE_DISABLED.
VD66GY camera;

// Neuro Studio shows a slider for each.
static int32_t exposureUs   = 1000;   // microseconds
static float   gain         = 8.0f;   // analogue up to 8, digital above
static uint8_t autoExposure = 0;      // 1 hands exposure and gain to the sensor

static void applyExposure() {
  if (autoExposure) {
    camera.setAutoExposure(true);
  } else {
    camera.setExposureUs((uint32_t)exposureUs);
    camera.setGain(gain);
  }
}

void setup() {
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera      = CAMERA_VGA;              // 88 frames per second
  cfg.isp_enabled = VD66_isp_tuning_auto;    // automatic white balance
  cfg.strobe      = STROBE_ENABLED;          // the LED lights each exposure
  cfg.transport   = TRANSPORT_USB;
  Vision.begin(cfg);

  // Applied as soon as the camera streams.
  applyExposure();

  n6_tune_int("exposure us", &exposureUs, 100, 10000);
  n6_tune_float("gain", &gain, 1.0f, 64.0f);
  n6_tune_bool("auto exposure", &autoExposure);
}

void loop() {
  static int32_t appliedUs   = exposureUs;
  static float   appliedGain = gain;
  static uint8_t appliedAuto = autoExposure;
  if (exposureUs != appliedUs || gain != appliedGain || autoExposure != appliedAuto) {
    appliedUs   = exposureUs;
    appliedGain = gain;
    appliedAuto = autoExposure;
    applyExposure();
  }

  static uint32_t lastReport = 0;
  if (millis() - lastReport >= 1000) {
    lastReport = millis();
    char line[64];
    snprintf(line, sizeof(line), "[EXPOSURE] %s, %lu us, gain %.2f",
             autoExposure ? "auto" : "manual",
             (unsigned long)camera.exposureUs(), (double)camera.gain());
    Serial.println(line);
    vnd_meta_publish_debug_text(line);
  }
  delay(20);
}
