/**
 * MagEncoder.cpp - Rotary magnetic sensor driver
 * Created by Carson G. Ray
 */

#include "MagEncoder.h"

// AS5600 default I2C address is 0x36

MagEncoder::MagEncoder(uint8_t dirPin, TwoWire* wire)
    : as5600(wire), dir(1), rawAngle(0), relAngle(0), dirPin(dirPin) {
        pinMode(dirPin, OUTPUT);
    }

bool MagEncoder::begin() {
    digitalWrite(dirPin, HIGH); // Set direction pin high (adjust as needed)
    as5600.begin();
    // Check if device is connected
    if (as5600.isConnected()) {
        // Initialize rawAngle with current sensor position
        rawAngle = as5600.readAngle();
        return true;
    }
    return false;
}

void MagEncoder::update() {
    uint16_t newRawAngle = as5600.readAngle();

    // Calculate relative angle change
    int16_t angleDiff = static_cast<int16_t>(newRawAngle - rawAngle);
    // Handle wrap-around
    if (angleDiff > 2048) {
        angleDiff -= 4096;
    } else if (angleDiff < -2048) {
        angleDiff += 4096;
    }
    relAngle += static_cast<float>(angleDiff) * dir * AS5600_RAW_TO_RADIANS;
    rawAngle = newRawAngle;
}

void MagEncoder::reset() {
    relAngle = 0;
}

void MagEncoder::setDir(int8_t dir) {
    this->dir = dir;
}

uint16_t MagEncoder::getRawAngle() const {
    return rawAngle;
}

float MagEncoder::getAbsAngle() const {
    // AS5600 outputs 12-bit value (0-4095) for 0-360 degrees
    return (rawAngle * AS5600_RAW_TO_RADIANS);
}

float MagEncoder::getRelAngle() const {
    return relAngle;
}
