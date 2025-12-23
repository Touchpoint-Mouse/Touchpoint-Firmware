/**
 * MagEncoder.h - Rotary magnetic sensor driver
 * Created by Carson G. Ray
 */

#ifndef MAGENCODER_H
#define MAGENCODER_H

#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include "BusChain.h"
#include "I2CDevice.h"

class MagEncoder : public I2CDevice {
public:
    MagEncoder();
    bool begin(TwoWire* wire) override;
    bool begin(uint8_t sensorPort, BusChain* busChain) override;
    void update();
    uint16_t getRawAngle();
    float getAngleDegrees();
    float getAngleRadians();

private:
    AS5600 as5600;
    TwoWire* i2cPort;
    uint8_t sensorChannel = 0;
    BusChain* busChain;
    bool busChainEnable = false;
    uint16_t lastRawAngle;
};

#endif // MAGENCODER_H
