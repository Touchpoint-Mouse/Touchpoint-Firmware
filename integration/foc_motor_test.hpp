#ifndef FOC_MOTOR_TEST_HPP
#define FOC_MOTOR_TEST_HPP

#include <Arduino.h>
#include "HydraFOCMotor.h"
#include "HydraFOCConfig.h"

// Motor parameters
constexpr float MOTOR_POLE_PAIRS = 7;
constexpr float MOTOR_KV = 100.0f;

// HydraFOC motor object
HydraFOCMotor motor(focMotorPins[0][0], focMotorPins[0][1], focMotorPins[0][2], focMotorPins[0][3], focMotorPins[0][4], focMotorPins[0][5], I2C1_SDA);

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
    motor.begin();

    delay(1000); // Wait for motor to stabilize
    Serial.println("FOC Motor Test Initialized.");

    // Set initial target position
    motor.setPosition(0.0f);
}

void loop() {
    // Run FOC control loop
    motor.update();

    // Prints current sensing readings
    /*Serial.print("Current A: ");
    Serial.print(analogRead(focCurrentPins[0][0]));
    Serial.print(" | Current B: ");
    Serial.println(analogRead(focCurrentPins[0][1]));*/

    // Prints encoder angle
    //Serial.print("Encoder Angle (rad): ");
    //Serial.println(motor.getPosition());
}

#endif // FOC_MOTOR_TEST_HPP