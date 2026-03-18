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
#include <SongbirdCore.h>
#include <SongbirdUART.h>

#include "V0_2_Config.h"
#include "HapticDriver.h"
#include "OpticalSensor.h"
#include "MouseDriver.h"
#include "IMU.h"
#include "LightState.h"

// Initialize light state machine
LightState lightState(NEOPIXEL_DIN);
LightState::OffEffect lightOff;
LightState::SolidEffect lightSolid(LightState::colorFromPreset(LightState::LightColor::Cyan));
LightState::RainbowEffect lightRainbow(64, 1.0f, 255, 0);
LightState::PulseEffect lightPulse(180, 1.5f, {255, 64, 32}, 0.0f);

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

// Initialize IMU
IMU imu(IMU_RST);

// Initialize mouse driver
MouseDriver mouseDriver(opticalSensor, imu, scrollWheel, zoomWheel, leftButton, rightButton);

TaskHandle_t gMouseTaskHandle = nullptr;
TaskHandle_t gLightTaskHandle = nullptr;

void vLightTask(void* pvParameters) {
	(void)pvParameters;

	for (;;) {
		lightState.update();
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}

void vMouseTask(void* pvParameters) {
	(void)pvParameters;

	for (;;) {
		mouseDriver.update();
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}

void setup() {
	Serial.begin(115200);
	lightState.begin();
	lightState.setEffect(lightRainbow);

	// Uses internal pullups for button logic without external resistors
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
	Wire.begin();
	if (!hapticDriver.begin(&Wire)) {
		Serial.println("Haptic driver init failed");
	}

	// Setup imu
	SPI.setSCK(IMU_SCK);
	SPI.setTX(IMU_MOSI);
	SPI.setRX(IMU_MISO);
	SPI.begin();
	if (!imu.begin(&SPI, IMU_CS, IMU_INT)) {
		Serial.println("IMU init failed");
	}

	if (!opticalSensor.begin()) {
		Serial.println("Optical sensor init failed");
	}
	opticalSensor.setCpi(1600);

	mouseDriver.begin();
	mouseDriver.setPointerSensitivity(1.0f);
	mouseDriver.setScrollSensitivity(1.0f);
	mouseDriver.setZoomSensitivity(1.0f);

	xTaskCreate(vLightTask, "LightTask", 512, nullptr, 1, &gLightTaskHandle);
	xTaskCreate(vMouseTask, "MouseTask", 1024, nullptr, 1, &gMouseTaskHandle);
}

void loop() {
	vTaskDelay(pdMS_TO_TICKS(1000));
}