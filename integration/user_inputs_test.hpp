/*
Tests buttons and scroll wheel inputs by printing their states to the serial monitor. Used for verifying correct wiring and functionality of user interface components.
 */

#ifndef USER_INPUTS_TEST_HPP
#define USER_INPUTS_TEST_HPP

#include <Arduino.h>
#include <Button.h>
#include <RotEncoder.h>
#include "hardware_config.h"

// Initialize buttons and rotary encoder with appropriate pins
Button leftButton(3, PullMode::PULLUP); // Left mouse button with debounce of 3ms
Button rightButton(3, PullMode::PULLUP); // Right mouse button with debounce of 3ms
RotEncoder scrollWheel(EncoderResolution::DOUBLE); // Scroll wheel encoder with double resolution
RotEncoder zoomWheel(EncoderResolution::SINGLE); // Zoom wheel encoder with single resolution

void setup() {
    Serial.begin(115200);

    // Uses internal pullups for testing without external resistors
    pinMode(LEFT_BUTTON, INPUT_PULLUP);
    pinMode(RIGHT_BUTTON, INPUT_PULLUP);
    pinMode(SCROLLWHEEL_A, INPUT_PULLUP);
    pinMode(SCROLLWHEEL_B, INPUT_PULLUP);
    pinMode(ZOOMWHEEL_A, INPUT_PULLUP);
    pinMode(ZOOMWHEEL_B, INPUT_PULLUP);
    
    // Attach pins to buttons and encoders
    leftButton.attach(LEFT_BUTTON);
    rightButton.attach(RIGHT_BUTTON);
    scrollWheel.attach(SCROLLWHEEL_A, SCROLLWHEEL_B);
    zoomWheel.attach(ZOOMWHEEL_A, ZOOMWHEEL_B);

    zoomWheel.setBounds(-2, 2); // Set zoom wheel bounds for testing
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
    if (scrollWheel.change() != 0) {
        Serial.print("Scroll Wheel: ");
        Serial.print("Steps: ");
        Serial.print(scrollWheel.totalSteps());
        Serial.print(" | CW Steps: ");
        Serial.print(scrollWheel.netSteps());
        Serial.print(" | Direction: ");
        Serial.println(scrollWheel.dir() ? "CW" : "CCW");
    }
    if (zoomWheel.change() != 0) {
        Serial.print("Zoom Wheel: ");
        Serial.print("Steps: ");
        Serial.print(zoomWheel.totalSteps());
        Serial.print(" | CW Steps: ");
        Serial.print(zoomWheel.netSteps());
        Serial.print(" | Direction: ");
        Serial.println(zoomWheel.dir() ? "CW" : "CCW");
    }
    Serial.flush();
}

#endif // USER_INPUTS_TEST_HPP
