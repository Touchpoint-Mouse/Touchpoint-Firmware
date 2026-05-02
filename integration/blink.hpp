#ifndef BLINK_HPP
#define BLINK_HPP

#include <Arduino.h>

void setup() {
    pinMode(25, OUTPUT);
}

void loop() {
    digitalWrite(25, HIGH); // Turn the LED on
    delay(1000); // Wait for 1 second
    digitalWrite(25, LOW); // Turn the LED off
    delay(1000); // Wait for 1 second
}

#endif // BLINK_HPP