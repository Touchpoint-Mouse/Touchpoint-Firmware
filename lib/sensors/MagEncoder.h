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
    void reset();
    void setDir(int8_t dir);
    uint16_t getRawAngle() const;
    float getAbsAngle() const;
    float getRelAngle() const;

private:
    AS5600 as5600;
    int8_t dir;
    uint16_t rawAngle;
    float relAngle;
    uint8_t dirPin;
};

#endif // MAGENCODER_H
