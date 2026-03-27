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
#ifndef INTEGRATION_TESTING
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
LightState::PulseEffect lightInit(180, 5.f, LightState::colorFromPreset(LightState::LightColor::Red), 0.0f);
LightState::PulseEffect lightIdle(180, 0.5f, LightState::colorFromPreset(LightState::LightColor::Green), 0.0f);

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
TaskHandle_t gDebugTaskHandle = nullptr;

void vLightTask(void* pvParameters) {
	(void)pvParameters;

	for (;;) {
		lightState.update();
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void vMouseTask(void* pvParameters) {
	(void)pvParameters;
	uint32_t lastTransportDebugMs = 0;

	for (;;) {
		mouseDriver.update();
		const uint32_t nowMs = millis();
		if (nowMs - lastTransportDebugMs >= 1000u) {
			lastTransportDebugMs = nowMs;
			//mouseDriver.printTransportDebug(Serial);
		}
		if (leftButton.changeTo(HIGH)) {
			hapticDriver.playEffect(24); // Play a click effect
		} else if (leftButton.changeTo(LOW)) {
			hapticDriver.playEffect(25); // Stop effect on release
		}
		// Always yield so TinyUSB/CDC servicing and lower-priority tasks can run.
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

void vDebugTask(void* pvParameters) {
	(void)pvParameters;

	for (;;) {
		const MouseDriver::SensorReadings readings = mouseDriver.getSensorReadings();

		Serial.print("SENSORS ");

		Serial.print("BTN[L:");
		Serial.print(readings.leftPressed ? 1 : 0);
		Serial.print(" R:");
		Serial.print(readings.rightPressed ? 1 : 0);
		Serial.print("] ");

		Serial.print("ENC[S:");
		Serial.print(readings.scrollSteps);
		Serial.print(" Z:");
		Serial.print(readings.zoomSteps);
		Serial.print("] ");

		Serial.print("OPT[V:");
		Serial.print(readings.opticalValid ? 1 : 0);
		Serial.print(" L:");
		Serial.print(readings.lifted ? 1 : 0);
		Serial.print(" M:");
		Serial.print(readings.optical.hasMotion ? 1 : 0);
		Serial.print(" dX:");
		Serial.print(readings.optical.deltaX);
		Serial.print(" dY:");
		Serial.print(readings.optical.deltaY);
		Serial.print(" SQ:");
		Serial.print(readings.optical.squal);
		Serial.print("] ");

		Serial.print("IMU[V:");
		Serial.print(readings.imuValid ? 1 : 0);
		Serial.print(" q:");
		Serial.print(readings.imuRotation[0], 4);
		Serial.print(",");
		Serial.print(readings.imuRotation[1], 4);
		Serial.print(",");
		Serial.print(readings.imuRotation[2], 4);
		Serial.print(",");
		Serial.print(readings.imuRotation[3], 4);
		Serial.println("]");

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void setup() {
	lightState.begin();
	lightState.setEffect(lightInit);
	xTaskCreate(vLightTask, "LightTask", 512, nullptr, 1, &gLightTaskHandle);

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
		while (true) {
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	// Setup imu
	SPI1.setSCK(IMU_SCK);
	SPI1.setTX(IMU_MOSI);
	SPI1.setRX(IMU_MISO);
	SPI1.begin();
	if (!imu.begin(&SPI1, IMU_CS, IMU_INT)) {
		Serial.println("IMU init failed");
		while (true) {
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	SPI.setSCK(OPTICAL_SCK);
	SPI.setTX(OPTICAL_MOSI);
	SPI.setRX(OPTICAL_MISO);
	SPI.begin();
	if (!opticalSensor.begin(&SPI, OPTICAL_CS, OPTICAL_INT)) {
		Serial.println("Optical sensor init failed");
		while (true) {
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	mouseDriver.setCPI(1100);
	mouseDriver.setHeadlessModeEnabled(true);
	mouseDriver.setPointerOffset(Vector2f(-29.676f, -45.375f));
	mouseDriver.setPointerSensitivity(2.0f);
	mouseDriver.setScrollSensitivity(1.0f);
	mouseDriver.setZoomSensitivity(1.0f);
	imu.setZAxisOrientation(IMU::ZAxisOrientation::Down);
	mouseDriver.setOpticalRotation(MouseDriver::OpticalRotation::Deg270);
	mouseDriver.setScrollClockwisePositive(true);
	mouseDriver.setZoomClockwisePositive(true);

	mouseDriver.begin();

	/*Serial.begin(115200);
	const uint32_t serialStartMs = millis();
	while (!Serial && (millis() - serialStartMs) < 1500u) {
		delay(10);
	}
	Serial.println("Starting Touchpoint Mouse Firmware");
	Serial.println("Init complete");*/

	xTaskCreate(vMouseTask, "MouseTask", 2048, nullptr, 1, &gMouseTaskHandle);
	// xTaskCreate(vDebugTask, "DebugTask", 1024, nullptr, 1, &gDebugTaskHandle);
	// Serial.println("Setup complete");
	lightState.setEffect(lightIdle);
}

void loop() {
	vTaskDelay(pdMS_TO_TICKS(1000));
}

#endif // INTEGRATION_TESTING