/*
  Event recording low power

  Sleeps between cycles, wakes to take one picture, and saves a clip of the
  pictures around a trigger to the SD card. The neural network is not used.

  Hardware: Neuro N6 with a VD66GY camera on the Thermal carrier, the only
  camera this example works with. SD card module on the header MOSI, MISO
  and SCK pins, CS on A5. Optional PIR motion sensor: OUT to D6, plus 3V3 and
  GND.
  Needs the SdFat library from the Library Manager.

  This example code is in the public domain.
*/

#include <NeuroN6_PowerSave.h>
#include <PowerSaveCamera.h>
#include <VD66GY_Arduino.h>
#include <SPI.h>
#include <SdFat.h>
#include <N6Events.h>
#include <N6EventsSD.h>

// Declared at file scope so the low-power camera code can find it.
VD66GY camera(VD66GY_CARRIER_THERMAL);

// Settings
#define CYCLE_MS          10000UL  // time between pictures
#define POST_CYCLE_MS      1000UL  // time between pictures during an event
#define AE_SETTLE_MS         200   // exposure settling time after waking
#define PRE_MS             30000   // video kept from before the trigger
#define POST_MS             5000   // video kept from after it
#define REFRACTORY_MS      20000   // quiet time before another event
#define PIR_PIN               D6   // may be left unconnected
#define TIMER_EVERY            0   // fires every Nth cycle, 0 for off
#define LOW_POWER              0   // 1 slows the clocks and turns USB off
#define SD_CS_PIN             A5
#define SD_CLOCK_MHZ          20

static SdFat    sd;
static bool     g_sd_ok  = false;
static bool     g_cam_ok = false;
static uint32_t g_cycle  = 0;
static volatile bool g_pir = false;

static void pir_isr(void) { g_pir = true; }

static void status(uint32_t jpeg_len, bool slept_full)
{
  n6_events_stats_t st; n6_events_stats(&st);
  uint32_t sdf = 0, sdb = 0, sdx = 0; n6_event_sink_sd_stats(&sdf, &sdb, &sdx);
  char line[200];
  snprintf(line, sizeof(line),
    "[dormant] cycle=%lu state=%d jpeg=%luB ring=%lu events=%lu clips=%lu | sd frames=%lu fails=%lu | woke=%s%s%s",
    (unsigned long)g_cycle, (int)n6_events_state(), (unsigned long)jpeg_len,
    (unsigned long)st.ring.count, (unsigned long)st.events, (unsigned long)st.clips,
    (unsigned long)sdf, (unsigned long)sdx,
    slept_full ? "timer" : "early",
    g_cam_ok ? "" : "  (CAMERA FAILED)",
    g_sd_ok  ? "" : "  (NO CARD: clips are not being saved)");
  Serial.println(line);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

  PowerSave.begin();
  PowerSave.sensorsSleep();

  pinMode(PIR_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), pir_isr, RISING);   // also wakes the board

  PowerSaveCam.setJpegMode(true);
  g_cam_ok = PowerSaveCam.begin(CAMERA_VGA);
  PowerSaveCam.sensorSleep();
  PowerSaveCam.busSleep();

  // The ring lives in PSRAM, so PowerSave.psramSleep() must not be called. It
  // cannot be undone, and touching PSRAM afterwards crashes the board.
  n6_events_cfg_t ec = N6_EVENTS_CFG_DEFAULT;
  ec.ring_bytes    = 2u << 20;
  ec.clip_fps      = 0;                     // 0 keeps every picture
  ec.pre_ms        = PRE_MS;
  ec.post_ms       = POST_MS;
  ec.refractory_ms = REFRACTORY_MS;
  ec.meta_max      = 0;
  ec.announce_ms   = 3000;
  if (!n6_events_begin(&ec)) {
    Serial.print("[dormant] n6_events_begin failed: "); Serial.println(n6_events_last_error());
  }

  SPI.begin();
  SPI.setKernelSource(RCC_SPI2CLKSOURCE_HSI);
  SdSpiConfig sdcfg(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(SD_CLOCK_MHZ), &SPI);
  g_sd_ok = sd.begin(sdcfg);
  if (g_sd_ok) n6_events_add_sink(n6_event_sink_sd(&sd, "/events"));
  else         Serial.println("[dormant] no SD card: events will be reported, clips not saved");

  PowerSave.thermalPower(false);
  PowerSave.npuOff();

#if LOW_POWER
  PowerSave.setClockProfile(PS_CLOCK_LOW);
  PowerSave.usbOff();                       // cannot be undone
#endif
  PowerSave.idleWfiEnable();                // called last

  Serial.println("=== EventRecordingLowPower: Stop mode, one JPEG per wake, clip on trigger ===");
}

void loop()
{
  g_cycle++;

  uint32_t len = 0;
  if (g_cam_ok) {
    PowerSaveCam.busWake();
    if (PowerSaveCam.sensorWake()) {
      delay(AE_SETTLE_MS);
      len = PowerSaveCam.captureJpeg();
    }
    PowerSaveCam.sensorSleep();
    PowerSaveCam.busSleep();
  }
  if (len) n6_events_push(PowerSaveCam.jpeg(), len, NULL, 0, n6_events_now());

  // Triggers
  if (g_pir) { g_pir = false; n6_events_fire(N6_EVT_SRC_GPIO, 700); }
#if TIMER_EVERY
  if ((g_cycle % TIMER_EVERY) == 0) n6_events_fire(N6_EVT_SRC_TIMER, 100);
#endif

  // Writes the clip once the event has finished. Nothing else runs meanwhile.
  n6_events_service();

  // sleepStop() returns false when an interrupt, such as the PIR, ends the sleep
  // early, or when a USB host keeps the board in light sleep instead.
  const uint32_t nap = (n6_events_state() == N6_EVT_POST_ROLL) ? POST_CYCLE_MS : CYCLE_MS;
  const bool full = PowerSave.sleepStop(nap);
  status(len, full);
}
