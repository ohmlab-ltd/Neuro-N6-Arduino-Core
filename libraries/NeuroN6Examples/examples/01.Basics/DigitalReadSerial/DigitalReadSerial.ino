/*
  Digital read serial

  Reads the on-board USER button and prints its state to the Serial Monitor.
  The button has an external pull-down resistor, so the pin reads HIGH while
  the button is pressed.

  Hardware: Neuro N6, on-board USER button.

  This example code is in the public domain.
*/

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  pinMode(BTN_USER, INPUT);
}

void loop() {
  int buttonState = digitalRead(BTN_USER);
  Serial.println(buttonState ? "pressed" : "released");
  delay(100);
}
