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

// Press threshold (must press this deep to trigger press event)
const float pressThreshold = 0.007f; // in radians

// Release threshold (must return this far to trigger release event - hysteresis)
const float releaseThreshold = 0.0001f; // in radians

const float intermediateThreshold = 0.002f; // in radians

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
    
    // Initialize HydraFOC motor
    motor.begin(Direction::CW, 1.04f, true);
    motor.resetEncoder();

    // Initial position
    motor.setPosition(buttonHeight);
    currState = BUTTON_RELEASED;
    delay(500); // Wait for motor to stabilize
}

void updateSim() {
    // Get current position and velocity
    float position = motor.getPosition();
    float velocity = motor.getVelocity();

    // Determine new state based on position with hysteresis
    ButtonState newState = currState;  // Default to current state
    
    if (currState == BUTTON_RELEASED) {
        // Must cross press threshold to register press
        if (position <= buttonHeight - pressThreshold) {
            newState = BUTTON_PRESSED;
        }
    } else {
        // Must cross release threshold to register release (hysteresis)
        if (position >= buttonHeight - releaseThreshold) {
            newState = BUTTON_RELEASED;
        }
    }

    // Check if timer is active
    if (clickStart != 0) {
        // Timer is running - check if it expired
        if (millis() - clickStart >= clickTime) {
            // Timer expired - return to appropriate position based on current state
            clickStart = 0;
            lastClickEnd = millis();
            
            // If still pressed, use intermediate position to reduce motor fight
            if (currState == BUTTON_PRESSED) {
                motor.setPosition(buttonHeight - intermediateThreshold);
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
                // State changed (threshold crossed) - start timer and apply zero torque
                clickStart = millis();
                motor.setTorque(clickForce);

                // Updates current state
                currState = newState;
                
                // Log the transition
                if (newState == BUTTON_PRESSED) {
                    Serial.println("Button Pressed");
                } else {
                    Serial.println("Button Released");
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

    // Prints position and target position every 100 loop counts
    if (loopCounter % 100 == 0) {
        Serial.print(motor.getPosition(), 6);
        Serial.print(" ");
        Serial.println(motor.getTargetPosition(), 6);
    }

    loopCounter++;
}

#endif // BUTTON_TEST_HPP