/*
  Blink

  Turns the on-board LED on for one second, then off for one second,
  repeatedly. The Neuro N6 RGB LED is active low, so LOW lights it.

  Hardware: Neuro N6. LED_BUILTIN is the green channel of the RGB LED.

  This example code is in the public domain.
*/

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);   // on
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);  // off
  delay(1000);
}
