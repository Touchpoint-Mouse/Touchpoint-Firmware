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
        bool playEffect(uint8_t effect);
        bool queueEffect(uint8_t effect);
        void clearQueue();
        uint8_t queuedEffectCount() const;
        bool playQueuedEffects();

        void enableRealtimeMode();
        void disableRealtimeMode();
        void setRealtimeValue(uint8_t value);
        void stop();
        bool isPlaying();

    private:
        Adafruit_DRV2605 drv;
        static constexpr uint8_t MAX_WAVEFORM_SLOTS = 8;
        uint8_t queuedEffects[MAX_WAVEFORM_SLOTS];
        uint8_t queueCount;
        bool realtimeMode = false;

        void applyQueuedWaveforms();
};

#endif // HAPTIC_DRIVER_H