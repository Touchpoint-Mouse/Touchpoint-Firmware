//////////////////////////////////////////////////////////////
// Note: uncomment the following line to enable integration testing
// This will include an hpp file for testing purposes
// Be sure to comment out this line for production builds
//////////////////////////////////////////////////////////////
//#define INTEGRATION_TESTING

#ifdef INTEGRATION_TESTING
#include <Button.h>
#include <RotEncoder.h>
#include <DigitalServo.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BNO08x.h>
//#include <SongbirdCore.h>
//#include <SongbirdUART.h>
#include "../integration/user_inputs_test.hpp" // Testing file to run
#endif
//////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

#include <Button.h>
#include <RotEncoder.h>
#include <DigitalServo.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BNO08x.h>
#include <SongbirdCore.h>
#include <SongbirdUART.h>

#include "V0_2_Config.h"
#include "HapticDriver.h"
#include "OpticalSensor.h"
#include "MouseDriver.h"

// Initialize neopixel
Adafruit_NeoPixel pixel(1, NEOPIXEL_DIN, NEO_GRB + NEO_KHZ800);

// Initialize buttons and rotary encoder
Button leftButton(3, PullMode::PULLUP); // Left mouse button with debounce of 3ms
Button rightButton(3, PullMode::PULLUP); // Right mouse button with debounce of 3ms
RotEncoder scrollWheel(3, EncoderResolution::DOUBLE); // Scroll wheel encoder with double resolution
RotEncoder zoomWheel(3, EncoderResolution::SINGLE); // Zoom wheel encoder with single resolution

// Initialize elevation servo
DigitalServo elevationServo;

// Initialize haptic driver
HapticDriver hapticDriver;

// Initialize optical sensor
OpticalSensor opticalSensor;

// Initialize mouse driver
MouseDriver mouseDriver;

// Initialize IMU
Adafruit_BNO08x  bno08x(IMU_RST);
sh2_SensorValue_t sensorValue;

void setReports(void);

TaskHandle_t gBlinkTaskHandle = nullptr;
TaskHandle_t gOpticalPollTaskHandle = nullptr;
bool gOpticalSensorReady = false;

void vBlinkTask(void* pvParameters) {
	(void)pvParameters;
	uint16_t hue = 0;

	for (;;) {
		pixel.setPixelColor(0, pixel.gamma32(pixel.ColorHSV(hue, 255, 50)));
		pixel.show();

		hue += 256;
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}

void vOpticalPollTask(void* pvParameters) {
	(void)pvParameters;
	if (!gOpticalSensorReady) {
		vTaskDelete(nullptr);
	}

	OpticalSensor::MotionData motionData;

	for (;;) {
		if (opticalSensor.poll(motionData)) {
			const bool lifted = !motionData.onSurface;
			mouseDriver.handleOpticalDelta(motionData.deltaX, motionData.deltaY, lifted);
		}

		vTaskDelay(pdMS_TO_TICKS(5));
	}
}

void setup() {
	Serial.begin(115200);
	pixel.begin();

	// Uses internal pullups for testing without external resistors
    pinMode(LEFT_BUTTON, INPUT_PULLUP);
    pinMode(RIGHT_BUTTON, INPUT_PULLUP);
    pinMode(SCROLLWHEEL_A, INPUT_PULLUP);
    pinMode(SCROLLWHEEL_B, INPUT_PULLUP);
    pinMode(ZOOMWHEEL_A, INPUT_PULLUP);
    pinMode(ZOOMWHEEL_B, INPUT_PULLUP);
    
    // Attach pins to buttons and encoders
    leftButton.attach(LEFT_BUTTON);
    rightButton.attach(RIGHT_BUTTON);
    scrollWheel.attach(SCROLLWHEEL_A, SCROLLWHEEL_B);
    zoomWheel.attach(ZOOMWHEEL_A, ZOOMWHEEL_B);

	// Setup haptic driver
	Wire.setSDA(SDA);
	Wire.setSCL(SCL);
	hapticDriver.begin(&Wire);
	mouseDriver.begin();

	gOpticalSensorReady = opticalSensor.begin();
	if (!gOpticalSensorReady) {
		Serial.println("Optical sensor init failed");
	}

	xTaskCreate(vBlinkTask, "BlinkTask", 512, nullptr, 1, &gBlinkTaskHandle);
	xTaskCreate(vOpticalPollTask, "OpticalPollTask", 1024, nullptr, 1, &gOpticalPollTaskHandle);
}

void loop() {
	vTaskDelay(pdMS_TO_TICKS(1000));
}