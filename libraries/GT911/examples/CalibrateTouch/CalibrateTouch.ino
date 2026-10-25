/*
  Calibrate touch

  Finds the touch panel's axis mapping from three corner touches, prints the
  setMapping() line for it, then prints live touch positions.

  Hardware: Neuro N6 with the TFT carrier and its touch panel.

  This example code is in the public domain.
*/

#include <Wire.h>
#include <GT911.h>

GT911 touch;

// Panel size in pixels.
static const int PANEL_W = 800;
static const int PANEL_H = 480;

struct Sample { int x, y; };
static Sample s_tl, s_tr, s_bl;

// Waits for a touch and returns its raw position once the finger lifts.
static Sample capture(const char *corner)
{
  Serial.print("  touch the ");
  Serial.print(corner);
  Serial.println(" corner...");

  while (touch.read() > 0) { delay(20); }

  Sample s = { -1, -1 };
  while (s.x < 0) {
    uint8_t r[GT911_POINT_BYTES];
    if (touch.read() > 0 && touch.raw(0, r)) {
      s.x = (int)((uint16_t)r[1] | ((uint16_t)r[2] << 8));
      s.y = (int)((uint16_t)r[3] | ((uint16_t)r[4] << 8));
    }
    delay(15);
  }

  Serial.print("    got raw x="); Serial.print(s.x);
  Serial.print(" y=");            Serial.println(s.y);

  while (touch.read() > 0) { delay(20); }
  delay(150);                               // debounce
  return s;
}

static void calibrate()
{
  Serial.println();
  Serial.println("--- calibration ---");

  s_tl = capture("TOP-LEFT");
  s_tr = capture("TOP-RIGHT");
  s_bl = capture("BOTTOM-LEFT");

  // Along the top, whichever reading changes more is the screen's X axis.
  const int dx_tr = abs(s_tr.x - s_tl.x);
  const int dy_tr = abs(s_tr.y - s_tl.y);
  const bool swap_xy = (dy_tr > dx_tr);

  // An axis is flipped when its reading falls where it should rise.
  const int tl_sx = swap_xy ? s_tl.y : s_tl.x;
  const int tr_sx = swap_xy ? s_tr.y : s_tr.x;
  const int tl_sy = swap_xy ? s_tl.x : s_tl.y;
  const int bl_sy = swap_xy ? s_bl.x : s_bl.y;

  const bool flip_x = (tr_sx < tl_sx);
  const bool flip_y = (bl_sy < tl_sy);

  Serial.println();
  Serial.println("--- result ---");
  Serial.print("  left->right moved raw X by "); Serial.print(dx_tr);
  Serial.print(", raw Y by ");                   Serial.println(dy_tr);
  Serial.print("  => axes ");
  Serial.println(swap_xy ? "ARE swapped" : "are NOT swapped");
  Serial.print("  => flip_x "); Serial.println(flip_x ? "YES" : "no");
  Serial.print("  => flip_y "); Serial.println(flip_y ? "YES" : "no");

  Serial.println();
  Serial.println("  Paste this into your sketch's setup(), after touch.begin():");
  Serial.println();
  char line[120];
  snprintf(line, sizeof(line),
           "    touch.setMapping(%d, %d, /*swap_xy=*/%s, /*flip_x=*/%s, /*flip_y=*/%s);",
           PANEL_W, PANEL_H,
           swap_xy ? "true" : "false",
           flip_x  ? "true" : "false",
           flip_y  ? "true" : "false");
  Serial.println(line);
  Serial.println();

  touch.setMapping(PANEL_W, PANEL_H, swap_xy, flip_x, flip_y);

  Serial.println("--- live check: corners should now read as you expect ---");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }

  Wire.begin();

  Serial.println();
  Serial.println("=== GT911 CalibrateTouch ===");

  if (!touch.begin()) {
    Serial.println("NO PANEL FOUND.");
    Serial.println("  Neither 0x5D nor 0x14 answered with a GT911 chip id.");
    Serial.println("  Check the panel is on this I2C bus and powered. RESET is");
    Serial.println("  tied to the system reset on this carrier, so a panel held");
    Serial.println("  in reset is not something software can fix.");
    return;
  }

  Serial.print("Found GT911, chip id \"");
  Serial.print(touch.chipId());
  Serial.print("\" at I2C 0x");
  Serial.println(touch.address(), HEX);
  Serial.print("Configured for ");
  Serial.print(touch.controllerWidth());
  Serial.print(" x ");
  Serial.println(touch.controllerHeight());

  touch.setMapping(PANEL_W, PANEL_H, false, false, false);

  calibrate();
}

void loop() {
  if (touch.address() == 0) { delay(1000); return; }

  const int n = touch.read();
  if (n <= 0) { delay(20); return; }

  // read() repeats the last position until the panel has a fresh sample.
  static int last_x = -1, last_y = -1, last_n = -1;
  if (n == last_n && touch.x(0) == last_x && touch.y(0) == last_y) {
    delay(20);
    return;
  }
  last_n = n; last_x = touch.x(0); last_y = touch.y(0);

  for (int i = 0; i < n; i++) {
    uint8_t r[GT911_POINT_BYTES];
    touch.raw(i, r);
    char line[120];
    snprintf(line, sizeof(line),
             "p%d  id=%-3d x=%-4d y=%-4d size=%-4d  raw %02X %02X %02X %02X %02X %02X %02X %02X",
             i, touch.id(i), touch.x(i), touch.y(i), touch.size(i),
             r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]);
    Serial.println(line);
  }

  const int x = touch.x(0), y = touch.y(0);
  const char *hx = (x < PANEL_W / 3) ? "LEFT" : (x > (2 * PANEL_W) / 3) ? "RIGHT" : "middle";
  const char *hy = (y < PANEL_H / 3) ? "TOP"  : (y > (2 * PANEL_H) / 3) ? "BOTTOM" : "middle";
  Serial.print("    reads as: "); Serial.print(hy);
  Serial.print(" "); Serial.println(hx);

  delay(60);
}
