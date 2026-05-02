/*
    * HapticDriver.cpp - Implementation for controlling haptic actuators.
    * Created by Carson G. Ray on 2024-06-01.
*/

#include "HapticDriver.h"

namespace {
    constexpr uint8_t DRV2605_REG_GO_ADDR = 0x0C;
}

//#include <FreeRTOS.h>
//#include <task.h>

HapticDriver::HapticDriver() {
    queueCount = 0;
}

bool HapticDriver::requestPriority(uint8_t priority) {
    // Protect quick checks/updates with a short critical section.
    //taskENTER_CRITICAL();
    if (priority < currentPriority) {
        //taskEXIT_CRITICAL();
        return false;
    }

    bool needStop = false;
    bool needSetIntTrig = false;
    if (priority > currentPriority) {
        // Raise priority: clear queued commands and mark to stop the driver outside
        // the critical section (to avoid holding critical while doing I/O).
        currentPriority = priority;
        queueCount = 0;
        if (realtimeMode) {
            realtimeMode = false;
            needSetIntTrig = true;
        }
        needStop = true;
    }
    //taskEXIT_CRITICAL();

    if (needStop) {
        drv.stop();
        if (needSetIntTrig) {
            drv.setMode(DRV2605_MODE_INTTRIG);
        }
    }

    return true;
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

bool HapticDriver::playEffect(uint8_t effect, uint8_t priority) {
    clearQueue();
    if (!queueEffect(effect, priority)) {
        return false;
    }

    return playQueuedEffects();
}

bool HapticDriver::queueEffect(uint8_t effect, uint8_t priority) {
    if (!requestPriority(priority)) {
        return false;
    }

    // If realtime mode was active, switch it off; do minimal protected update
    bool needSetIntTrig = false;
    //taskENTER_CRITICAL();
    if (realtimeMode) {
        realtimeMode = false;
        needSetIntTrig = true;
    }

    if (queueCount >= MAX_WAVEFORM_SLOTS) {
        //taskEXIT_CRITICAL();
        return false;
    }

    queuedEffects[queueCount] = effect;
    queueCount++;
    //taskEXIT_CRITICAL();

    if (needSetIntTrig) {
        drv.setMode(DRV2605_MODE_INTTRIG);
    }

    return true;
}

void HapticDriver::clearQueue() {
    //taskENTER_CRITICAL();
    queueCount = 0;
    //taskEXIT_CRITICAL();
}

uint8_t HapticDriver::queuedEffectCount() const {
    uint8_t c;
    //taskENTER_CRITICAL();
    c = queueCount;
    //taskEXIT_CRITICAL();
    return c;
}

bool HapticDriver::playQueuedEffects() {
    // Snapshot queue count and realtime flag under a short critical section.
    //taskENTER_CRITICAL();
    uint8_t localCount = queueCount;
    bool localRealtime = realtimeMode;
    //taskEXIT_CRITICAL();

    if (localCount == 0) {
        if (!localRealtime && (drv.readRegister8(DRV2605_REG_GO_ADDR) & 0x01) == 0) {
            //taskENTER_CRITICAL();
            currentPriority = 0;
            //taskEXIT_CRITICAL();
        }
        return false;
    }

    // Copy queued effects into local buffer while protected briefly.
    uint8_t localEffects[MAX_WAVEFORM_SLOTS];
    //taskENTER_CRITICAL();
    for (uint8_t i = 0; i < localCount; ++i) {
        localEffects[i] = queuedEffects[i];
    }
    // Clear queue now that we've taken a snapshot
    queueCount = 0;
    //taskEXIT_CRITICAL();

    // Apply waveforms from the local snapshot (performing I/O outside critical)
    for (uint8_t slot = 0; slot < localCount; slot++) {
        drv.setWaveform(slot, localEffects[slot]);
    }
    drv.setWaveform(localCount, 0);
    drv.go();
    return true;
}

void HapticDriver::enableRealtimeMode(uint8_t priority) {
    if (!requestPriority(priority)) {
        return;
    }

    bool doEnable = false;
    //taskENTER_CRITICAL();
    if (!realtimeMode) {
        realtimeMode = true;
        queueCount = 0;
        doEnable = true;
    }
    //taskEXIT_CRITICAL();

    if (doEnable) {
        drv.setMode(DRV2605_MODE_REALTIME);
    }
}

void HapticDriver::disableRealtimeMode() {
    bool doDisable = false;
    //taskENTER_CRITICAL();
    if (realtimeMode) {
        realtimeMode = false;
        doDisable = true;
    }
    //taskEXIT_CRITICAL();

    if (doDisable) {
        drv.setMode(DRV2605_MODE_INTTRIG);
    }
}

bool HapticDriver::isRealtimeMode() const {
    return realtimeMode;
}

void HapticDriver::setRealtimeValue(int8_t value, uint8_t priority) {
    if (!requestPriority(priority)) {
        return;
    }

    bool needSetRealtime = false;
    //taskENTER_CRITICAL();
    if (!realtimeMode) {
        realtimeMode = true;
        queueCount = 0;
        needSetRealtime = true;
    }
    //taskEXIT_CRITICAL();

    if (needSetRealtime) {
        drv.setMode(DRV2605_MODE_REALTIME);
    }

    drv.setRealtimeValue(value);

    if (value == 0) {
        //taskENTER_CRITICAL();
        currentPriority = 0;
        //taskEXIT_CRITICAL();
    }
}

void HapticDriver::stop() {
    // Stop hardware I/O outside critical sections, but update shared state safely.
    drv.stop();

    //taskENTER_CRITICAL();
    queueCount = 0;
    bool wasRealtime = realtimeMode;
    realtimeMode = false;
    currentPriority = 0;
    //taskEXIT_CRITICAL();

    if (wasRealtime) {
        drv.setMode(DRV2605_MODE_INTTRIG);
    }
}

bool HapticDriver::isPlaying() {
    return (drv.readRegister8(DRV2605_REG_GO_ADDR) & 0x01) != 0;
}

void HapticDriver::applyQueuedWaveforms() {
    // Note: this helper is now unused; keeping implementation minimal in case it's
    // referenced elsewhere. Prefer playQueuedEffects() which snapshots the queue.
    //taskENTER_CRITICAL();
    uint8_t localCount = queueCount;
    uint8_t localEffects[MAX_WAVEFORM_SLOTS];
    for (uint8_t i = 0; i < localCount; ++i) localEffects[i] = queuedEffects[i];
    //taskEXIT_CRITICAL();

    for (uint8_t slot = 0; slot < localCount; slot++) {
        drv.setWaveform(slot, localEffects[slot]);
    }
    drv.setWaveform(localCount, 0);
}