#ifndef FOC_MOTOR_TEST_HPP
#define FOC_MOTOR_TEST_HPP

#include <Arduino.h>
#include "HydraFOCMotor.h"
#include "HydraFOCConfig.h"

// Loop counter
unsigned long loopCounter = 0;

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

    delay(1000); // Wait for motor to stabilize
    Serial.println("FOC Motor Test Initialized.");

    // Set initial target position
    motor.setPosition(0.f);
}

void loop() {
    // Run FOC control loop
    motor.update();

    // Motor variable monitoring
    //motor.monitor();

    // Prints current sensing readings
    /*Serial.print("Current A: ");
    Serial.print(analogRead(focCurrentPins[0][0]));
    Serial.print(" | Current B: ");
    Serial.println(analogRead(focCurrentPins[0][1]));*/

    // Prints encoder angle with full precision (every 100 loop counts)
    if (loopCounter % 100 == 0) {
        //Serial.println(motor.getPosition(), 6);  // 6 decimal places
    }

    loopCounter++;
}

#endif // FOC_MOTOR_TEST_HPP