/*
  Audio event detection

  Names one of ten sounds from the on-board microphone. The class scores and
  the spectrogram appear in Neuro Studio's Audio panel.

  Hardware: Neuro N6 with an OV5640 camera.

  This example code is in the public domain. The model it embeds,
  aed_yamnet_int8.onnx, comes from ST's STM32N6 audio getting-started package
  and is licensed under ST's SLA0044; see models/README.md.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>
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

// Neuro Studio shows the results on a video stream, so the camera runs.
OV5640     camera;
Microphone mic;
NEURON6_DECLARE_MODEL(aed);

static int16_t chunk[512];
static bool    micOK = false, aedOK = false;

void setup() {
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera    = CAMERA_VGA;
  cfg.fps       = FPS_30;
  cfg.transport = TRANSPORT_USB;
  Vision.begin(cfg);                 // no model, the sketch runs it

  micOK = mic.begin();
  if (micOK) {
    // The sketch takes the samples, so Neuro Studio plays no audio.
    mic.streamToNS(false);
    aedOK = n6_aed_begin(&NN_Instance_aed, mic.sampleRate());
    n6_aed_set_threshold(CONFIDENCE);
    n6_tune_int("confidence %", &confidence_x100, 5, 95);
  }

  Serial.println();
  Serial.println("=== audio event detection -> Neuro Studio ===");
  if (!micOK)      Serial.println("microphone did not start (Tools > Debug console: On prints [MICDBG] details)");
  else if (!aedOK) Serial.println("AED init FAILED - model input is not 64x96 int8");
  else             Serial.println("open the Audio panel in Neuro Studio's sidebar");
}

void loop() {
  if (!micOK || !aedOK) { delay(200); return; }

  if (mic.available() >= 512u) {
    const uint32_t got = mic.read(chunk, 512);
    if (got) n6_aed_push(chunk, got);
  }

  if (!n6_aed_ready()) { delay(2); return; }

  n6_aed_run();
  n6_aed_set_threshold(CONFIDENCE);   // follows the tuning panel

  const int top = n6_aed_top();

  vnd_meta_audio_result(n6_aed_scores(), N6_AED_NUM_CLASSES,
                        top, n6_aed_level(), CONFIDENCE);
  vnd_meta_audio_mel(n6_aed_input_image(), N6_AED_NMEL, N6_AED_NFRAMES, 0);

  static uint32_t lastLabels = 0;
  if (millis() - lastLabels >= 1000u) {
    lastLabels = millis();
    vnd_meta_audio_labels(n6_aed_label, N6_AED_NUM_CLASSES);
  }

  static uint32_t lastBeat = 0;
  if (millis() - lastBeat >= 1000u) {
    lastBeat = millis();
    Serial.print("[aed] ");
    Serial.print(top >= 0 ? n6_aed_label(top) : "-");
    Serial.print("  level=");  Serial.print(n6_aed_level());
    Serial.print("  thr=");    Serial.println(CONFIDENCE, 2);
  }
}
