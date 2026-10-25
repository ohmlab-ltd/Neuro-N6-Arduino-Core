/*
  Button interrupt

  Counts presses of the on-board USER button using an interrupt. Two pins
  with the same GPIO number, such as A2 (PF12) and D9 (PD12), share an
  interrupt line and cannot both have an interrupt attached.

  Hardware: Neuro N6, on-board USER button.

  This example code is in the public domain.
*/

// Written by the interrupt handler, read by loop().
volatile unsigned long pressCount = 0;

void onButtonPress() {
  pressCount++;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  pinMode(BTN_USER, INPUT);
  attachInterrupt(digitalPinToInterrupt(BTN_USER), onButtonPress, RISING);
}

void loop() {
  static unsigned long lastReported = 0;

  if (pressCount != lastReported) {
    lastReported = pressCount;
    Serial.print("presses: ");
    Serial.println(lastReported);
  }

  delay(50);
}
