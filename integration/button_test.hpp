#ifndef BUTTON_TEST_HPP
#define BUTTON_TEST_HPP

#include <Arduino.h>
#include "HydraFOCMotor.h"
#include "HydraFOCConfig.h"

// Loop counter
unsigned long loopCounter = 0;

// Button states
enum ButtonState {
    BUTTON_RELEASED,
    BUTTON_PRESSED
} currState;

// Click time in milliseconds
const unsigned long clickTime = 17; // ms

// Minimum time between clicks (debounce)
const unsigned long minTimeBetweenClicks = 50; // ms

// Click timer (0 means not active)
unsigned long clickStart = 0;

// Last click end time
unsigned long lastClickEnd = 0;

// Button height
const float buttonHeight = 2; // in radians

// Click threshold
const float clickThreshold = 0.007f; // in radians

const float intermediateThreshold = 0.002f; // in radians

const float releaseThreshold = 0.0001f;

// Force during click
const float clickForce = 0.f; // Volts

// HydraFOC motor object
HydraFOCMotor motor(focMotorPins[0][0], focMotorPins[0][1], focMotorPins[0][2], focMotorPins[0][3], focMotorPins[0][4], focMotorPins[0][5], I2C1_SDA, focCurrentPins[0][0], focCurrentPins[0][1]);

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    // Configure I2C
    Wire.begin(I2C0_SDA, I2C0_SCL);

    // Configure driver pins
    pinMode(focDriverSleepPin, OUTPUT);
    pinMode(focDriverResetPin, OUTPUT);
    digitalWrite(focDriverSleepPin, HIGH); // Wake up driver
    digitalWrite(focDriverResetPin, HIGH); // Release reset
    pinMode(focCurrentPins[0][0], INPUT);
    pinMode(focCurrentPins[0][1], INPUT);

    // Initialize HydraFOC motor
    motor.begin(Direction::CW, 1.04f, true);
    motor.resetEncoder();

    // Initial position
    motor.setPosition(buttonHeight);
    delay(500); // Wait for motor to stabilize
}

void updateSim() {
    // Get current position
    float position = motor.getPosition();

    // Determine new state based on position
    ButtonState newState;
    if (position > buttonHeight - clickThreshold) {
        newState = BUTTON_RELEASED;
    } else {
        newState = BUTTON_PRESSED;
    }

    // Check if timer is active
    if (clickStart != 0) {
        // Timer is running - check if it expired
        if (millis() - clickStart >= clickTime) {
            // Timer expired - always return to released position
            clickStart = 0;
            lastClickEnd = millis();
            if (newState == BUTTON_PRESSED) {
                motor.setPosition(buttonHeight-intermediateThreshold); // Slightly below button height
            } else {
                motor.setPosition(buttonHeight);
            }
        }
        // else: timer still running, stay in zero torque mode
    } else {
        // Timer not running - check for state change
        if (newState != currState) {
            // Check if enough time has passed since last click
            if (millis() - lastClickEnd >= minTimeBetweenClicks) {
                bool shouldTrigger = false;
                
                // Check if this is a valid transition
                if (newState == BUTTON_PRESSED) {
                    // Always give feedback on press
                    shouldTrigger = true;
                } else {
                    // Only give feedback on release if near button height
                    if (position > buttonHeight - releaseThreshold) {
                        shouldTrigger = true;
                    }
                }
                
                if (shouldTrigger) {
                    // State changed (threshold crossed) - start timer and apply zero torque
                    clickStart = millis();
                    motor.setTorque(clickForce);

                    // Updates current state
                    currState = newState;
                    
                    // Log the transition
                    if (newState == BUTTON_PRESSED) {
                        Serial.println("Button Pressed - Zero Torque");
                    } else {
                        Serial.println("Button Released - Zero Torque");
                    }
                }
            }
            // else: too soon after last click, ignore this crossing
        }
        // else: no state change, stay in position control
    }
}

void loop() {
    // Run button simulation
    updateSim();

    // Run FOC control loop
    motor.update();

    // Prints position
    if (loopCounter % 100 == 0) {
        //Serial.println(motor.getPosition(), 6);
    }

    loopCounter++;
}

#endif // BUTTON_TEST_HPP