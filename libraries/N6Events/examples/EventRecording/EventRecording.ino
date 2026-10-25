/*
  LICENSING NOTICE

  This example sketch embeds the YOLOv8 model file
  yolov8n_256_quant_pc_uf_pose_coco-st.tflite,
  which is licensed under the GNU Affero General Public License, version 3.0
  (AGPL-3.0), and not under the MIT licence that governs the remainder of
  this repository. Accordingly, this sketch is licensed under AGPL-3.0.

  A firmware image built from this sketch is a work based on that model.
  Any party who distributes such an image, or a product containing it,
  must comply with AGPL-3.0 in full, including the provision of
  Corresponding Source, or must hold an Ultralytics Enterprise License.
  Building and flashing the sketch for personal or internal use does not
  by itself trigger those obligations.

  The STMicroelectronics components that execute the model are supplied
  under ST licence SLA0044, whose terms in most cases forbid the
  distribution of firmware that combines them with AGPL-licensed
  software.

  The full notice and the licence text are in models/README.md and
  models/LICENSE-AGPL-3.0.txt.
*/

/*
  Event recording

  Keeps the last few seconds of video in memory and runs the person-detection
  network only when the picture changes. On a detection it saves a clip to
  the SD card covering the seconds before and after.

  Hardware: Neuro N6 with an OV5640 camera. SD card module on the header
  MOSI, MISO and SCK pins, CS on A5.
  Needs the SdFat library from the Library Manager.

  This example code is licensed under AGPL-3.0. See the notice above.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>
#include <SPI.h>
#include <SdFat.h>
#include <N6Events.h>
#include <N6EventsSD.h>

#pragma neuron6 model="yolov8n_256_quant_pc_uf_pose_coco-st.tflite" name=yolov8_mpe

// Settings
static const float    MPE_CONF      = 0.40f;  // confidence threshold
static const float    MPE_IOU       = 0.50f;  // overlap threshold
static const uint8_t  FIRE_SCORE    = 60;     // score, 0 to 255, that fires an event
static const uint16_t PRE_MS        = 5000;   // video kept from before the event
static const uint16_t POST_MS       = 5000;   // video kept from after it
static const uint16_t REFRACTORY_MS = 10000;  // quiet time before another event
static const uint32_t STATUS_MS     = 2000;   // status print period

#define SD_CS_PIN     A5
#define SD_CLOCK_MHZ  20

OV5640 camera;
NEURON6_DECLARE_MODEL(yolov8_mpe);

// A copy of the network's input, for the motion check.
#define SNAP_W 256
#define SNAP_H 256
static uint8_t g_snap[SNAP_W * SNAP_H * 3] __attribute__((aligned(32), section(".ext_psram")));

static n6_motion_gate_t g_gate;
static SdFat            sd;
static bool             g_sd_ok = false;

static void status(void)
{
  n6_events_stats_t st; n6_events_stats(&st);
  uint32_t sdf = 0, sdb = 0, sdx = 0; n6_event_sink_sd_stats(&sdf, &sdb, &sdx);
  char line[200];
  snprintf(line, sizeof(line),
    "[armed] state=%d events=%lu clips=%lu | ring held=%lu/%luKB tapped=%lu kept=%lu dropped=%lu | "
    "motion=%u%% | sd frames=%lu bytes=%luKB fails=%lu%s",
    (int)n6_events_state(), (unsigned long)st.events, (unsigned long)st.clips,
    (unsigned long)st.ring.count, (unsigned long)(st.ring.cap >> 10),
    (unsigned long)st.tap_frames, (unsigned long)st.tap_kept, (unsigned long)st.tap_dropped_paused,
    (unsigned)(g_gate.last_score / 10u),
    (unsigned long)sdf, (unsigned long)(sdb >> 10), (unsigned long)sdx,
    g_sd_ok ? "" : "  (NO CARD: clips are not being saved)");
  Serial.println(line);
}

void setup()
{
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera      = CAMERA_VGA;
  cfg.fps         = FPS_15;
  cfg.mirror_flip = CAMERA_AUTO_FLIP;
  cfg.nn.width    = SNAP_W;
  cfg.nn.height   = SNAP_H;
  cfg.nn.aspect   = ASPECT_CROP_CENTER;
  cfg.nn.max_fps  = 5;
  cfg.transport   = TRANSPORT_USB;
  cfg.af          = AF_CONTINUOUS;
  Vision.begin(&NN_Instance_yolov8_mpe,
               PostProcess_YOLOv8_MPE_OnSlot(MPE_CONF, MPE_IOU, VND_META_SLOT_MODEL_0),
               cfg);

  // The two calls below claim memory that Vision.begin() hands back, so they
  // come after it.
  n6_events_cfg_t ec = N6_EVENTS_CFG_DEFAULT;
  ec.pre_ms = PRE_MS; ec.post_ms = POST_MS; ec.refractory_ms = REFRACTORY_MS;
  if (!n6_events_begin(&ec)) {
    Serial.print("[armed] n6_events_begin failed: "); Serial.println(n6_events_last_error());
  }
  if (!n6_motion_gate_begin(&g_gate, 64, 48, NULL)) {
    Serial.println("[armed] motion gate: no memory");
  }

  SPI.begin();
  SPI.setKernelSource(RCC_SPI2CLKSOURCE_HSI);
  SdSpiConfig sdcfg(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(SD_CLOCK_MHZ), &SPI);
  g_sd_ok = sd.begin(sdcfg);
  if (g_sd_ok) n6_events_add_sink(n6_event_sink_sd(&sd, "/events"));
  else         Serial.println("[armed] no SD card: events will be reported, clips not saved");

  Serial.println("=== EventRecording: motion -> network -> clip to card, event to host ===");
}

void loop()
{
  bool moved = false;
  if (NN_capture_snapshot(g_snap, sizeof g_snap) > 0) {
    moved = n6_motion_gate_update(&g_gate, g_snap, g_nn_out_w, g_nn_out_h);
  }

  if (moved) {
    Vision.run();
    if (n6_events_last_boxes(FIRE_SCORE) > 0) {
      n6_events_fire(N6_EVT_SRC_MOTION | N6_EVT_SRC_NN, 1000);
    }
  }

  // Writes the clip when the event ends, which blocks loop() for a second or
  // two. Frames arriving meanwhile are dropped and counted.
  n6_events_service();

  static uint32_t last = 0;
  if (millis() - last >= STATUS_MS) { last = millis(); status(); }

  if (!moved) delay(20);
}
