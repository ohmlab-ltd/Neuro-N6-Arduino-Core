/*
  Audio spectrum

  Shows the on-board microphone's spectrum on the display as 64 bars.

  Hardware: Neuro N6 with the TFT carrier and its 800x480 display.

  This example code is in the public domain.
*/

#include <OV5640_Arduino.h>
#include <PostProcess.h>
#include <ohmlab_lcd_draw.h>
#include <Microphone.h>
#include <n6_fft.h>
#include <n6_spectrum.h>

static const int   NBARS    = 64;
static const float F_LO     = 50.0f;      // leftmost band, Hz
static const float F_HI     = 16000.0f;   // rightmost band, Hz
static const float DB_FLOOR = -70.0f;     // quietest level shown, dB

// The camera object brings up the display carrier. The image is not used.
OV5640     camera(CARRIER_TFT);
Microphone mic;

static const int PANEL_W = 800;
static const int PANEL_H = 480;

static const int PLOT_TOP    = 66;
static const int PLOT_BOTTOM = 430;
static const int PLOT_H      = PLOT_BOTTOM - PLOT_TOP;

static int16_t        chunk[N6_FFT_HOP];
static n6_frame_t     frame;
static float          mag[N6_FFT_BINS];
static n6_spectrum_t  spec;
static bool           micOK = false;

// v from 0 to 1: blue, cyan, green, yellow, red.
static void bar_colour(float v, uint8_t *r, uint8_t *g, uint8_t *b)
{
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;

  if (v < 0.33f)        { const float t = v / 0.33f;
                          *r = 0;   *g = (uint8_t)(200 * t);       *b = 255; }
  else if (v < 0.60f)   { const float t = (v - 0.33f) / 0.27f;
                          *r = 0;   *g = 255;   *b = (uint8_t)(255 * (1 - t)); }
  else if (v < 0.82f)   { const float t = (v - 0.60f) / 0.22f;
                          *r = (uint8_t)(255 * t); *g = 255; *b = 0; }
  else                  { const float t = (v - 0.82f) / 0.18f;
                          *r = 255; *g = (uint8_t)(255 * (1 - t)); *b = 0; }
}

void setup() {
  Serial.begin(115200);

  VisionConfig cfg;
  cfg.camera    = CAMERA_WVGA;
  cfg.fps       = FPS_30;
  cfg.transport = TRANSPORT_LCD;
  Vision.begin(cfg);

  micOK = mic.begin();
  if (micOK) {
    // mic.read() needs the stream to Neuro Studio off.
    mic.streamToNS(false);
    n6_fft_init();
    n6_frame_init(&frame);
    n6_spectrum_init(&spec, NBARS, (float)mic.sampleRate(), F_LO, F_HI);
    spec.db_floor = DB_FLOOR;
  }

  Serial.println();
  Serial.println("=== audio spectrum analyser ===");
  if (micOK) {
    Serial.print("mic ");            Serial.print(mic.sampleRate());
    Serial.print(" Hz, FFT ");       Serial.print((unsigned)N6_FFT_SIZE);
    Serial.print(" hop ");           Serial.print((unsigned)N6_FFT_HOP);
    Serial.print(" -> ");            Serial.print((float)mic.sampleRate() / N6_FFT_SIZE, 1);
    Serial.print(" Hz/bin, ");       Serial.print(NBARS);
    Serial.println(" log bands");
  } else {
    Serial.println("microphone did not start (Tools > Debug console: On prints [MICDBG] details)");
  }
}

void loop() {
  if (!micOK) { delay(200); return; }

  if (mic.available() < N6_FFT_HOP) { delay(2); return; }
  // Each drawing waits for the panel, which is slower than the hops arrive.
  // Older hops only fill the window, so the bars keep up with the sound.
  while (mic.available() >= 2u * N6_FFT_HOP) {
    (void)n6_frame_push(&frame, chunk, mic.read(chunk, N6_FFT_HOP));
  }
  const uint32_t got = mic.read(chunk, N6_FFT_HOP);
  if (got == 0) return;
  if (!n6_frame_push(&frame, chunk, got)) return;   // window still filling

  n6_fft_magnitude(n6_frame_data(&frame), mag);
  n6_spectrum_update(&spec, mag);

  uint16_t *ovl = OVL_GetBackBuffer();
  if (ovl == nullptr) return;

  ohm_lcd_bind_argb4444(ovl, PANEL_W, PANEL_H, PANEL_W);
  ohm_lcd_clear_transparent();
  ohm_lcd_set_font(&Font16);

  const int slot = (PANEL_W - 32) / NBARS;      // 12 px at 64 bars
  const int bw   = slot - 1;                    // 1 px gap

  for (int i = 0; i < NBARS; i++) {
    const int x = 16 + i * slot;
    const int h = (int)(spec.bars[i] * PLOT_H);

    if (h > 0) {
      uint8_t r, g, b;
      bar_colour(spec.bars[i], &r, &g, &b);
      ohm_lcd_set_color(r, g, b);
      ohm_lcd_fillrect(x, PLOT_BOTTOM - h, bw, h);
    }

    const int ph = (int)(spec.peaks[i] * PLOT_H);
    if (ph > 2) {
      uint8_t r, g, b;
      bar_colour(spec.peaks[i], &r, &g, &b);
      ohm_lcd_set_color(r, g, b);
      ohm_lcd_fillrect(x, PLOT_BOTTOM - ph - 2, bw, 2);
    }
  }

  ohm_lcd_set_color(120, 120, 120);
  ohm_lcd_hline(16, PLOT_BOTTOM, NBARS * slot);

  const float ticks[] = { 100.0f, 1000.0f, 10000.0f };
  const char *names[] = { "100", "1k", "10k" };
  for (int t = 0; t < 3; t++) {
    for (int i = 0; i < NBARS - 1; i++) {
      if (spec.centre_hz[i] <= ticks[t] && spec.centre_hz[i + 1] > ticks[t]) {
        const int x = 16 + i * slot;
        ohm_lcd_set_color(120, 120, 120);
        ohm_lcd_vline(x, PLOT_BOTTOM + 1, 5);
        ohm_lcd_set_color(200, 200, 200);
        ohm_lcd_text(x - 8, PLOT_BOTTOM + 8, names[t]);
        break;
      }
    }
  }

  char msg[72];
  ohm_lcd_set_color(255, 255, 255);
  ohm_lcd_text(16, 10, "AUDIO SPECTRUM");

  const int loud = n6_spectrum_loudest(&spec);
  if (loud >= 0) {
    snprintf(msg, sizeof(msg), "peak %5.0f Hz", (double)spec.centre_hz[loud]);
  } else {
    snprintf(msg, sizeof(msg), "quiet");
  }
  ohm_lcd_set_color(120, 220, 255);
  ohm_lcd_text(16, 34, msg);

  snprintf(msg, sizeof(msg), "%.1f Hz/bin   floor %.0f dB",
           (double)mic.sampleRate() / N6_FFT_SIZE, (double)spec.db_floor);
  ohm_lcd_set_color(150, 150, 150);
  ohm_lcd_text(430, 34, msg);

  OVL_Commit();

  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last >= 1000u) {
    last = now;
    Serial.print("[spec] level=");  Serial.print(mic.peak());
    if (loud >= 0) {
      Serial.print(" peak=");       Serial.print(spec.centre_hz[loud], 0);
      Serial.print("Hz mag=");      Serial.print(spec.bars[loud], 2);
    }
    Serial.println();
  }
}
