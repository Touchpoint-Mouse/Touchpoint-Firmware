/*
Tests buttons and scroll wheel inputs by printing their states to the serial monitor. Used for verifying correct wiring and functionality of user interface components.
 */

#ifndef USER_INPUTS_TEST_HPP
#define USER_INPUTS_TEST_HPP

#include <Arduino.h>
#include <Button.h>
#include <RotEncoder.h>
#include "V0_2_Config.h"

// Initialize buttons and rotary encoder with appropriate pins
Button leftButton(LEFT_BUTTON, true); // Left mouse button
Button rightButton(RIGHT_BUTTON, true); // Right mouse button
RotEncoder scrollWheel(SCROLLWHEEL_A, SCROLLWHEEL_B); // Scroll wheel encoder
RotEncoder zoomWheel(ZOOMWHEEL_A, ZOOMWHEEL_B); // Zoom wheel encoder

void setup() {
    Serial.begin(115200);
    pinMode(LEFT_BUTTON, INPUT_PULLUP);
    pinMode(RIGHT_BUTTON, INPUT_PULLUP);
    pinMode(SCROLLWHEEL_A, INPUT_PULLUP);
    pinMode(SCROLLWHEEL_B, INPUT_PULLUP);
    pinMode(ZOOMWHEEL_A, INPUT_PULLUP);
    pinMode(ZOOMWHEEL_B, INPUT_PULLUP);
}

void loop() {
    // Update button states
    leftButton.update();
    rightButton.update();
    
    // Update rotary encoder states
    scrollWheel.update();
    zoomWheel.update();

    // Check for events and print to serial
    if (leftButton.change()) {
        Serial.print("Left Button: ");
        Serial.println(leftButton.state() ? "Pressed" : "Released");
    }
    if (rightButton.change()) {
        Serial.print("Right Button: ");
        Serial.println(rightButton.state() ? "Pressed" : "Released");
    }
    if (scrollWheel.hasMoved()) {
        Serial.print("Scroll Wheel: ");
        Serial.print("Steps: ");
        Serial.print(scrollWheel.steps());
        Serial.print(" | Direction: ");
        Serial.println(scrollWheel.dir() ? "CW" : "CCW");
    }
    if (zoomWheel.hasMoved()) {
        Serial.print("Zoom Wheel: ");
        Serial.print("Steps: ");
        Serial.print(zoomWheel.steps());
        Serial.print(" | Direction: ");
        Serial.println(zoomWheel.dir() ? "CW" : "CCW");
    }
}

#endif // USER_INTERFACES_TEST_HPP
