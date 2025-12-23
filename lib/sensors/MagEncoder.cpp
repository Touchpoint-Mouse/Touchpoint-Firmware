/**
 * MagEncoder.cpp - Rotary magnetic sensor driver
 * Created by Carson G. Ray
 */

#include "MagEncoder.h"

// AS5600 default I2C address is 0x36

MagEncoder::MagEncoder(uint8_t dirPin, TwoWire* wire)
    : as5600(wire), lastRawAngle(0), dirPin(dirPin) {
        pinMode(dirPin, OUTPUT);
    }

bool MagEncoder::begin() {
    digitalWrite(dirPin, HIGH); // Set direction pin high (adjust as needed)
    as5600.begin();
    // Check if device is connected
    return as5600.isConnected();
}

void MagEncoder::update() {
    lastRawAngle = as5600.readAngle();
}

uint16_t MagEncoder::getRawAngle() const {
    return lastRawAngle;
}

float MagEncoder::getAngleDegrees() const {
    // AS5600 outputs 12-bit value (0-4095) for 0-360 degrees
    return (lastRawAngle * AS5600_RAW_TO_DEGREES);;
}

float MagEncoder::getAngleRadians() const {
    return (lastRawAngle * AS5600_RAW_TO_RADIANS);
}
