// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// Released under the GPLv3 license to match the rest of the
// Adafruit NeoPixel library

#ifndef NEOPIXEL_TEST_HPP
#define NEOPIXEL_TEST_HPP

#include <Arduino.h>
#include "V0_2_Config.h"
#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        NEOPIXEL_DIN // On Trinket or Gemma, suggest changing this to 1

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(1, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

void loop() {
  // Loops through a bunch of hues
  for(int i=0; i < 65536; i++) { // For each pixel...
    pixels.setPixelColor(0, pixels.ColorHSV(i));

    pixels.show();   // Send the updated pixel colors to the hardware.

    delay(1); // Pause before next pass through loop
  }
}

#endif // NEOPIXEL_TEST_HPP