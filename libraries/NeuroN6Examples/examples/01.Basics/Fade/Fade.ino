/*
  Fade

  Fades the red channel of the on-board RGB LED in and out using
  analogWrite(). The LED is active low, so a larger analogWrite value makes
  it dimmer.

  Hardware: Neuro N6, on-board RGB LED.

  This example code is in the public domain.
*/

int brightness = 0;   // 0 to 255
int fadeAmount = 5;   // change per step

void setup() {
  pinMode(LEDR, OUTPUT);
}

void loop() {
  // Inverted: the LED is active low.
  analogWrite(LEDR, 255 - brightness);

  brightness = brightness + fadeAmount;

  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }

  delay(30);
}
