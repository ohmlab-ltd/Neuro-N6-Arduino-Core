/*
  Analog read serial

  Reads an analog input on pin A0 and prints the result to the Serial
  Monitor. The analog inputs are referenced to 1.8 V and must not exceed the
  analog supply.

  Hardware: Neuro N6, potentiometer wiper to A0, its outer pins to 1V8 and
  GND.

  This example code is in the public domain.
*/

// Full scale of the analog input, in millivolts.
const int referenceMillivolts = 1800;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  // 12-bit range, 0 to 4095.
  analogReadResolution(12);
}

void loop() {
  int sensorValue = analogRead(A0);

  int millivolts = (sensorValue * referenceMillivolts) / 4095;

  Serial.print(sensorValue);
  Serial.print("  ");
  Serial.print(millivolts);
  Serial.println(" mV");
  delay(200);
}
