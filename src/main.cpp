//////////////////////////////////////////////////////////////
// Note: uncomment the following line to enable integration testing
// This will include an hpp file for testing purposes
// Be sure to comment out this line for production builds
//////////////////////////////////////////////////////////////
#define INTEGRATION_TESTING

#ifdef INTEGRATION_TESTING
#include <Button.h>
#include <RotEncoder.h>
#include <DigitalServo.h>
#include <Adafruit_NeoPixel.h>
// Adafruit TinyUSB only for ESP32 (RP2040 has built-in USB support)
#ifndef ARDUINO_ARCH_RP2040
#include <Adafruit_TinyUSB.h>
#endif
#include <Adafruit_BNO08x.h>
//#include <SongbirdCore.h>
//#include <SongbirdUART.h>
#include "../integration/mouse_emulator_test.hpp" // Testing file to run
#endif
//////////////////////////////////////////////////////////////

#ifndef INTEGRATION_TESTING
#define INTEGRATION_TESTING

#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

#endif // INTEGRATION_TESTING