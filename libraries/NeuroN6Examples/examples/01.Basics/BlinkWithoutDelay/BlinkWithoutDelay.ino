/*
  Blink without delay

  Turns the on-board LED on and off without using delay(), so the rest of
  loop() keeps running.

  Hardware: Neuro N6, on-board LED.

  This example code is in the public domain.
*/

// Time between changes, in milliseconds.
const unsigned long interval = 1000;

unsigned long previousMillis = 0;
int ledState = HIGH;   // active low, so HIGH is off

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    ledState = (ledState == LOW) ? HIGH : LOW;
    digitalWrite(LED_BUILTIN, ledState);
  }
}
