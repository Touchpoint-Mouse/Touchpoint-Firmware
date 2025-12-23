/**
 * MagEncoder.h - Rotary magnetic sensor driver
 * Created by Carson G. Ray
 */

#ifndef MAGENCODER_H
#define MAGENCODER_H

#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

class MagEncoder {
public:
    MagEncoder(uint8_t dirPin, TwoWire* wire = &Wire);
    bool begin();
    void update();
    uint16_t getRawAngle() const;
    float getAngleDegrees() const;
    float getAngleRadians() const;

private:
    AS5600 as5600;
    uint16_t lastRawAngle;
    uint8_t dirPin;
};

#endif // MAGENCODER_H
