/*
    * HapticDriver.h - Library for controlling haptic actuators.
    * Created by Carson G. Ray on 2024-06-01.
*/

#ifndef HAPTIC_DRIVER_H
#define HAPTIC_DRIVER_H

#include <Arduino.h>
#include <Adafruit_DRV2605.h>
#include <Wire.h>

class HapticDriver {
    public:
        HapticDriver();
        bool begin(TwoWire* wire=&Wire);
        void setEffect(uint8_t effect);
        void stop();
        bool isPlaying();

    private:
        Adafruit_DRV2605 drv;
};

#endif // HAPTIC_DRIVER_H