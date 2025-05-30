#include <Arduino.h>

const int buttonPin = 33;
const int ledPin = 4; 
int buttonState = 0; 

void setup() { 
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
}

void loop() {
  buttonState = digitalRead(buttonPin);
if (buttonState == LOW) { 
  digitalWrite(ledPin, HIGH); 
} else {
  digitalWrite(ledPin, LOW);
}
}