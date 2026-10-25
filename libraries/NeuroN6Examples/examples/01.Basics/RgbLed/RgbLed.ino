/*
  RGB LED

  Cycles the on-board RGB LED through its eight colours, one per second, and
  prints the name of each colour to the Serial Monitor.

  Hardware: Neuro N6, on-board RGB LED.

  This example code is in the public domain.
*/

struct Step { const char *name; bool r, g, b; };

static const Step kSteps[] = {
  { "red",     true,  false, false },
  { "yellow",  true,  true,  false },
  { "green",   false, true,  false },
  { "cyan",    false, true,  true  },
  { "blue",    false, false, true  },
  { "magenta", true,  false, true  },
  { "white",   true,  true,  true  },
  { "off",     false, false, false },
};

void setup() {
  Serial.begin(115200);
  RGB.begin();  // optional, the core has already configured the LED pins
}

void loop() {
  for (const Step &s : kSteps) {
    Serial.print("RGB: ");
    Serial.println(s.name);
    RGB.set(s.r, s.g, s.b);
    delay(1000);
  }
}
