/*
  Thermal camera

  Streams the thermal camera to Neuro Studio over USB. No colour camera runs,
  so the thermal image is the video.

  Hardware: Neuro N6 on the Thermal carrier, with its FLIR Lepton camera.

  This example code is in the public domain.
*/

#include <NeuroVisionThermal.h>
#include <PostProcess.h>     // streamFrame() publishes through it

// true: the pixels are hundredths of a kelvin. false: raw display values.
#define THERMAL_TLINEAR  true

NeuroVisionThermal thermal;

// One 160x120 frame.
static uint16_t tframe[NVT_VOSPI_PIXELS];
static uint32_t s_ok = 0u;
static uint32_t s_fail = 0u;

static void thermalLog(const char *text)
{
  NeuroN6_DbgPrint(text);
  vnd_meta_publish_debug_text(text);
}

void setup()
{
  thermal.powerOn();

  // The camera is mounted upside-down on the Thermal carrier.
  thermal.setFlip180(true);

  const bool present = thermal.isPresent();
  const bool cciOK = present && thermal.begin(3000) && thermal.ping();
  const bool streamOK = thermal.streamBegin();
  char line[96];
  if (cciOK && streamOK) {
    snprintf(line, sizeof(line), "[thermal] camera started\r\n");
  } else {
    snprintf(line, sizeof(line),
             "[thermal] camera did not start: found=%u control=%u video=%u err=%d\r\n",
             present ? 1u : 0u, cciOK ? 1u : 0u, streamOK ? 1u : 0u,
             thermal.videoLastError());
  }
  thermalLog(line);
}

void loop()
{
  // Blocks until the next frame, about nine a second.
  if (thermal.readFrame(tframe, 2000)) {
    thermal.streamFrame(tframe, THERMAL_TLINEAR);
    s_ok++;
    if (s_ok == 1u || (s_ok % 50u) == 0u) {
      char line[56];
      snprintf(line, sizeof(line), "[thermal] frame ok #%lu\r\n", (unsigned long)s_ok);
      thermalLog(line);
    }
  } else {
    s_fail++;
    if (s_fail == 1u || (s_fail % 10u) == 0u) {
      char line[64];
      snprintf(line, sizeof(line), "[thermal] readFrame fail #%lu err=%d\r\n",
               (unsigned long)s_fail, thermal.videoLastError());
      thermalLog(line);
    }
    if ((s_fail % 15u) == 0u) {
      thermalLog("[thermal] no frames - power-cycling Lepton\r\n");
      thermal.powerOn();
      delay(1200);   // boot time
    }
  }
}
