/*
  Save text

  Keeps a line of text in the asset store, the board's small persistent
  memory. Type a line in the Serial Monitor and press Enter: the board
  restarts once while the bootloader writes it to flash, then prints it again
  every time it starts.

  The store also holds the WiFi settings and the board's password. This
  sketch only replaces its own record.

  Hardware: Neuro N6.

  This example code is in the public domain.
*/

#include <n6_asset.h>

const unsigned int maxTextLength = 120;

void printStoredText() {
  uint32_t length = 0;
  uint32_t version = 0;
  if (!n6_asset_present(&version, &length)) {
    Serial.println("Nothing stored yet.");
    return;
  }

  char text[maxTextLength + 1];
  if (length > maxTextLength || !n6_asset_read(text, maxTextLength, &length, &version)) {
    Serial.print("A stored record of ");
    Serial.print(length);
    Serial.println(" bytes is too long to show here.");
    return;
  }
  text[length] = '\0';

  Serial.print("Stored text, version ");
  Serial.print(version);
  Serial.print(": ");
  Serial.println(text);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  uint32_t result = n6_asset_last_result();
  if (result != 0) {
    Serial.print("Last save: ");
    Serial.println(n6_asset_result_str(result));
  }
  printStoredText();
  Serial.println("Type a line and press Enter to save it. The board restarts once.");
}

void loop() {
  if (Serial.available() == 0) {
    return;
  }

  String text = Serial.readStringUntil('\n');
  text.trim();
  if (text.length() == 0) {
    return;
  }
  if (text.length() > maxTextLength) {
    text.remove(maxTextLength);
  }

  // Version 0 stores it as newer than the record already there.
  if (!n6_asset_stage(text.c_str(), text.length(), 0)) {
    Serial.println("The asset store refused the text.");
    return;
  }
  Serial.print("Saving \"");
  Serial.print(text);
  Serial.println("\". The board restarts to store it.");
  Serial.flush();
  delay(500);         // lets the message reach the Serial Monitor first
  n6_asset_apply();   // resets the board, does not return
}
