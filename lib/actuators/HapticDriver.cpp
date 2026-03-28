/*
    * HapticDriver.cpp - Implementation for controlling haptic actuators.
    * Created by Carson G. Ray on 2024-06-01.
*/

#include "HapticDriver.h"

namespace {
    constexpr uint8_t DRV2605_REG_GO_ADDR = 0x0C;
}

HapticDriver::HapticDriver() {
    queueCount = 0;
}

bool HapticDriver::begin(TwoWire* wire) {
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

bool HapticDriver::playEffect(uint8_t effect) {
    clearQueue();
    if (!queueEffect(effect)) {
        return false;
    }

    return playQueuedEffects();
}

bool HapticDriver::queueEffect(uint8_t effect) {
    if (queueCount >= MAX_WAVEFORM_SLOTS) {
        return false;
    }

    queuedEffects[queueCount] = effect;
    queueCount++;
    return true;
}

void HapticDriver::clearQueue() {
    queueCount = 0;
}

uint8_t HapticDriver::queuedEffectCount() const {
    return queueCount;
}

bool HapticDriver::playQueuedEffects() {
    if (queueCount == 0) {
        return false;
    }

    applyQueuedWaveforms();
    drv.go();
    clearQueue();
    return true;
}

void HapticDriver::enableRealtimeMode() {
    if (!realtimeMode) {
        drv.setMode(DRV2605_MODE_REALTIME);
        clearQueue();
        realtimeMode = true;
    }
}

void HapticDriver::disableRealtimeMode() {
    if (realtimeMode) {
        drv.setMode(DRV2605_MODE_INTTRIG);
        realtimeMode = false;
    }
}

bool HapticDriver::isRealtimeMode() const {
    return realtimeMode;
}

void HapticDriver::setRealtimeValue(int8_t value) {
    drv.setRealtimeValue(value);
}

void HapticDriver::stop() {
    // Send command to stop any currently playing effect
    drv.stop();
}

bool HapticDriver::isPlaying() {
    return (drv.readRegister8(DRV2605_REG_GO_ADDR) & 0x01) != 0;
}

void HapticDriver::applyQueuedWaveforms() {
    for (uint8_t slot = 0; slot < queueCount; slot++) {
        drv.setWaveform(slot, queuedEffects[slot]);
    }
    drv.setWaveform(queueCount, 0);
}