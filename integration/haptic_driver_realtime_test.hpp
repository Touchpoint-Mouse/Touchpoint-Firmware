#ifndef HAPTIC_DRIVER_REALTIME_TEST_HPP
#define HAPTIC_DRIVER_REALTIME_TEST_HPP

#include <Arduino.h>
#include <Wire.h>
#include "V0_2_Config.h"
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;

namespace {
  // RTP is signed in hardware (-127..127 represented as int8_t).
  int8_t amplitude = 0;
  int8_t step = 8;
  uint32_t lastStepMs = 0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("DRV2605 LRA realtime test");

  Wire.setSDA(SDA);
  Wire.setSCL(SCL);
  Wire.begin();

  if (!drv.begin()) {
    Serial.println("DRV2605 not found on I2C");
    while (true) {
      delay(1000);
    }
  }

  drv.useLRA();
  drv.selectLibrary(6);

  // Quick known-good pulse to verify electrical path before RTP mode.
  drv.setMode(DRV2605_MODE_INTTRIG);
  drv.setWaveform(0, 47);  // Strong buzz
  drv.setWaveform(1, 0);
  drv.go();
  delay(300);

  // Set Real-Time Playback mode
  drv.setMode(DRV2605_MODE_REALTIME);
  drv.setRealtimeValue(0);
  Serial.println("Entering RTP sweep");
}

void loop() {
  const uint32_t nowMs = millis();
  if (nowMs - lastStepMs >= 100u) {
    lastStepMs = nowMs;

    // Sweep from -120..120 to exercise both drive directions.
    if (amplitude >= 127) {
      step = -8;
    } else if (amplitude <= -128) {
      step = 8;
    }
    Serial.print("Amplitude: ");
    Serial.println(amplitude);
    amplitude = static_cast<int8_t>(amplitude + step);

    drv.setRealtimeValue(static_cast<uint8_t>(amplitude));
  }
}

#endif