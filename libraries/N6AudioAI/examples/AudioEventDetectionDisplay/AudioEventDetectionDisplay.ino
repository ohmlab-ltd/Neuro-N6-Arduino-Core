/*
  Audio event detection display

  Names one of ten sounds from the on-board microphone on the display, with a
  confidence bar for every class.

  Hardware: Neuro N6 with the TFT carrier and its 800x480 display.

  This example code is in the public domain. The model it embeds,
  aed_yamnet_int8.onnx, comes from ST's STM32N6 audio getting-started package
  and is licensed under ST's SLA0044; see models/README.md.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>
#include <ohmlab_lcd_draw.h>
#include <Microphone.h>
#include <n6_aed.h>
#include <vnd_meta_audio.h>
#include <NeuroN6_Tune.h>

// Classes: chainsaw, clock tick, crackling fire, crying baby, dog,
// helicopter, rain, rooster, sea waves, sneezing.
#pragma neuron6 model="aed_yamnet_int8.onnx" name=aed

// Score threshold x100, also set from Neuro Studio's tuning panel.
static int32_t confidence_x100 = 50;
#define CONFIDENCE  ((float)confidence_x100 / 100.0f)
static const int   QUIET_LEVEL = 250;    // quieter than this counts as silence

// The camera object brings up the display carrier. The image is unused.
OV5640     camera(CARRIER_TFT);
Microphone mic;
NEURON6_DECLARE_MODEL(aed);

static const int PANEL_W = 800;
static const int PANEL_H = 480;

static int16_t chunk[512];
static bool    micOK = false, aedOK = false;

// The class on the display, held for HOLD_MS.
static int      g_shown = -1;
static float    g_shown_score = 0.0f;
static uint32_t g_shown_at = 0;
static const uint32_t HOLD_MS = 2000;

void setup() {
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera    = CAMERA_WVGA;
  cfg.fps       = FPS_30;
  cfg.transport = TRANSPORT_LCD;
  Vision.begin(cfg);                 // brings up the display and the NPU

  micOK = mic.begin();
  if (micOK) {
    // The sketch takes the samples, so Neuro Studio plays no audio.
    mic.streamToNS(false);
    aedOK = n6_aed_begin(&NN_Instance_aed, mic.sampleRate());
    n6_aed_set_threshold(CONFIDENCE);
    n6_tune_int("confidence %", &confidence_x100, 5, 95);
  }

  Serial.println();
  Serial.println("=== audio event detection ===");
  if (!micOK)      Serial.println("microphone did not start (Tools > Debug console: On prints [MICDBG] details)");
  else if (!aedOK) Serial.println("AED init FAILED - model input is not 64x96 int8");
  else {
    Serial.print("mic ");  Serial.print(mic.sampleRate());
    Serial.print(" Hz -> 16000 Hz, 64x96 log-mel, ");
    Serial.print(N6_AED_NUM_CLASSES); Serial.println(" classes");
  }
}

void loop() {
  if (!micOK || !aedOK) { delay(200); return; }

  // Everything waiting is read: each drawing waits for the panel, which is
  // slower than 512 samples arrive.
  while (mic.available() >= 512u) {
    const uint32_t got = mic.read(chunk, 512);
    if (got) n6_aed_push(chunk, got);
  }

  if (n6_aed_ready()) {
    n6_aed_run();

    const int   top   = n6_aed_top();
    const float score = (top >= 0) ? n6_aed_scores()[top] : 0.0f;
    const bool  quiet = n6_aed_level() < QUIET_LEVEL;

    // Follows the tuning panel.
    n6_aed_set_threshold(CONFIDENCE);

    vnd_meta_audio_result(n6_aed_scores(), N6_AED_NUM_CLASSES,
                          top, n6_aed_level(), CONFIDENCE);
    vnd_meta_audio_mel(n6_aed_input_image(), N6_AED_NMEL, N6_AED_NFRAMES, 0);

    static uint32_t lastLabels = 0;
    if (millis() - lastLabels >= 1000u) {
      lastLabels = millis();
      vnd_meta_audio_labels(n6_aed_label, N6_AED_NUM_CLASSES);
    }

    if (top >= 0 && !quiet) {
      g_shown = top; g_shown_score = score; g_shown_at = millis();
      Serial.print("[aed] ");        Serial.print(n6_aed_label(top));
      Serial.print("  score=");      Serial.print(score, 2);
      Serial.print("  level=");      Serial.println(n6_aed_level());
    }
  }

  if (g_shown >= 0 && (millis() - g_shown_at) > HOLD_MS) g_shown = -1;

  uint16_t *ovl = OVL_GetBackBuffer();
  if (ovl == nullptr) return;

  ohm_lcd_bind_argb4444(ovl, PANEL_W, PANEL_H, PANEL_W);
  ohm_lcd_clear_transparent();
  ohm_lcd_set_font(&Font16);

  ohm_lcd_set_color(255, 255, 255);
  ohm_lcd_text(16, 10, "AUDIO EVENT DETECTION");

  if (g_shown >= 0) {
    ohm_lcd_set_color(60, 255, 120);
    ohm_lcd_rect_thick(16, 44, 768, 76, 3);
    ohm_lcd_text(36, 62, n6_aed_label(g_shown));
    char msg[32];
    snprintf(msg, sizeof(msg), "%.0f%%", (double)(g_shown_score * 100.0f));
    ohm_lcd_text(700, 62, msg);
  } else {
    ohm_lcd_set_color(110, 110, 110);
    ohm_lcd_rect_thick(16, 44, 768, 76, 2);
    ohm_lcd_text(36, 62, n6_aed_level() < QUIET_LEVEL ? "quiet" : "listening...");
  }

  const float *sc = n6_aed_scores();
  const int    y0 = 150, rowh = 30, barx = 210, barw = 500;
  for (int i = 0; i < (int)N6_AED_NUM_CLASSES; i++) {
    const int y = y0 + i * rowh;

    const bool win = (i == g_shown);
    ohm_lcd_set_color(win ? 60 : 190, win ? 255 : 190, win ? 120 : 190);
    ohm_lcd_text(16, y, n6_aed_label(i));

    int w = (int)(sc[i] * barw);
    if (w < 0) w = 0;
    if (w > barw) w = barw;

    ohm_lcd_set_color(70, 70, 70);
    ohm_lcd_hline(barx, y + 8, barw);
    if (w > 0) {
      if (win)                    ohm_lcd_set_color(60, 255, 120);
      else if (sc[i] >= CONFIDENCE) ohm_lcd_set_color(255, 200, 60);
      else                        ohm_lcd_set_color(90, 150, 255);
      ohm_lcd_fillrect(barx, y + 2, w, 14);
    }
  }

  ohm_lcd_set_color(255, 90, 90);
  ohm_lcd_vline(barx + (int)(CONFIDENCE * barw), y0 - 6,
                (int)N6_AED_NUM_CLASSES * rowh + 4);

  ohm_lcd_set_color(150, 150, 150);
  char msg[64];
  snprintf(msg, sizeof(msg), "level %u   threshold %.2f",
           (unsigned)n6_aed_level(), (double)CONFIDENCE);
  ohm_lcd_text(16, 456, msg);

  OVL_Commit();
}
