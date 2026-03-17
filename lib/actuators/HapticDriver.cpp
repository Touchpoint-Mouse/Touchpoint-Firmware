/*
    * HapticDriver.cpp - Implementation for controlling haptic actuators.
    * Created by Carson G. Ray on 2024-06-01.
*/

#include "HapticDriver.h"

HapticDriver::HapticDriver() {
    // Constructor can be used for initialization if needed
}

bool HapticDriver::begin(TwoWire* wire=&Wire) {
    // Initialize the haptic driver hardware
    // Initialize I2C with pins from config
    if (!drv.begin(wire)) {
        return false;
    }
    drv.useLRA();
    drv.selectLibrary(6);
    // I2C trigger by sending 'go' command 
    // default, internal trigger when sending GO command
    drv.setMode(DRV2605_MODE_INTTRIG); 
    return true;
}

void HapticDriver::setEffect(uint8_t effect) {
    // Send command to haptic driver to play the specified effect
    drv.setWaveform(0, effect); // Set effect in slot 0
}

void HapticDriver::stop() {
    // Send command to stop any currently playing effect
    drv.stop();
}

bool HapticDriver::isPlaying() {
    // Query the haptic driver to check if an effect is currently playing
    // Return true if playing, false otherwise
    return false; // Placeholder return value
}