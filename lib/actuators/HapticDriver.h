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
        bool playEffect(uint8_t effect, uint8_t priority = 0);
        bool queueEffect(uint8_t effect, uint8_t priority = 0);
        void clearQueue();
        uint8_t queuedEffectCount() const;
        bool playQueuedEffects();

        void enableRealtimeMode(uint8_t priority = 0);
        void disableRealtimeMode();
        bool isRealtimeMode() const;
        void setRealtimeValue(int8_t value, uint8_t priority = 0);
        void stop();
        bool isPlaying();

    private:
        Adafruit_DRV2605 drv;
        static constexpr uint8_t MAX_WAVEFORM_SLOTS = 8;
        uint8_t queuedEffects[MAX_WAVEFORM_SLOTS];
        uint8_t queueCount;
        bool realtimeMode = false;
        uint8_t currentPriority = 0;

        void applyQueuedWaveforms();
        bool requestPriority(uint8_t priority);
};

#endif // HAPTIC_DRIVER_H