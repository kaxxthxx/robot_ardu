#include <Arduino.h>

const int potPin = 34;
const int ledPin = 4; 
int potValue = 0; 

void setup() { 
  pinMode(potPin, INPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() { 
  potValue= analogRead(potPin);

  analogWrite(ledPin, map(potValue, 0, 4095, 0, 255));
  delay(10);
}