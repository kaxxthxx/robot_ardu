#include <Arduino.h>

const int ledPin = 4;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  for (int i = 0; i <= 255; i++) {
    analogWrite(ledPin, i);
    delay(1);
  }

  for (int i = 255; i >= 0; i--) {
    analogWrite(ledPin, i);
    delay(1);
  }
}
