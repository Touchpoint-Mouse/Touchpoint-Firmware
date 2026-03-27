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
#include <SongbirdUART.h>
#include <SongbirdCore.h>

#include "V0_2_Config.h"
#include "HapticDriver.h"
#include "OpticalSensor.h"
#include "MouseDriver.h"
#include "IMU.h"
#include "LightState.h"

#define SERIAL_BAUD 460800

// Initialize Songbird core for CDC UART
// Serial node object
SongbirdUART uart("UART");
// Serial protocol object
std::shared_ptr<SongbirdCore> core;

// Header enum
enum ElevationVibrationHeaders {
	PING = 0xFF,
	ELEVATION = 0x10,
	ELEVATION_SPEED = 0x11,
	VIBRATION = 0x20
};

// Elevation paramaters
float lastElevation = 0.f;
float elevationTarget = 0.f;
float maxElevationSpeed = 0.f; // units per second (0 = no smoothing)
uint64_t lastElevationTime = 0;

// Vibration parameters
uint8_t currVibPriority = 0;

// RTOS Task Handles
TaskHandle_t mouseTaskHandle = NULL;
TaskHandle_t updateTaskHandle = NULL;
TaskHandle_t servoTaskHandle = NULL;
TaskHandle_t debugTaskHandle = NULL;

// Connection state
volatile bool connectedToDesktop = false;

// Initialize light state machine
LightState lightState(NEOPIXEL_DIN);
LightState::OffEffect lightOff;
LightState::PulseEffect lightInit(180, 5.f, LightState::colorFromPreset(LightState::LightColor::Red), 0.0f);
LightState::PulseEffect lightIdle(180, 0.5f, LightState::colorFromPreset(LightState::LightColor::Green), 0.0f);
LightState::PulseEffect lightConnected(180, 0.5f, LightState::colorFromPreset(LightState::LightColor::Blue), 0.0f);

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

// Ping handler
void pingHandler(std::shared_ptr<SongbirdCore::Packet> pkt) {
	// Respond to ping
	auto response = core->createPacket(PING);
	core->sendPacket(response);

	// Mark as connected
	connectedToDesktop = true;
	// Set light effect to connected
	lightState.setEffect(lightConnected);
}

// Elevation feedback handler
void elevationFeedbackHandler(std::shared_ptr<SongbirdCore::Packet> pkt) {
	// Read elevation value from packet (float)
	float elevation = pkt->readFloat();
	// Constrain elevation between 0 and 1
	elevation = constrain(elevation, 0.f, 1.f);
	elevationTarget = elevation;
}

// Elevation smoothing handler
void elevationSmoothingHandler(std::shared_ptr<SongbirdCore::Packet> pkt) {
	// Read max speed value from packet (float)
	float speed = pkt->readFloat();
	// Constrain speed to positive values
	if (speed < 0.f) {
		speed = 0.f;
	}
	maxElevationSpeed = speed;
}

// Vibration feedback handler
void vibrationFeedbackHandler(std::shared_ptr<SongbirdCore::Packet> pkt) {
	// Get priority from packet
	uint8_t priority = pkt->readByte();

	// If priority is zero use realtime values
	if (priority == 0) {
		// Read realtime amplitude from packet
		uint8_t amplitude = pkt->readByte();
		hapticDriver.enableRealtimeMode();
		hapticDriver.setRealtimeValue(amplitude);

		// Reset vibration priority
		currVibPriority = 0;
		return;
	} else {
		// Otherwise, disable realtime mode to play queued effects
		hapticDriver.disableRealtimeMode();
	}

	// If priority is greater than current priority, stop all vibrations
	if (priority > currVibPriority) {
		hapticDriver.stop();
		hapticDriver.clearQueue();
		currVibPriority = priority;
	} else if (priority < currVibPriority) {
		// Ignore this command if it's lower priority than current vibration
		return;
	}

	// While there are vibration commands left in the packet
	while (pkt->getRemainingBytes() > 0) {
		// Get effect id
		uint8_t effectId = pkt->readByte();

		// Queue the effect
		hapticDriver.queueEffect(effectId);
	}
}

float getSmoothedElevation() {
	// Handle elevation smoothing
	uint64_t currentTime = micros();
	uint64_t elapsed = currentTime - lastElevationTime;
	
	// If no smoothing, immediately snap to target
	if (maxElevationSpeed == 0.f) {
		lastElevation = elevationTarget;
		lastElevationTime = currentTime;
		return lastElevation;
	}
	
	float maxDelta = maxElevationSpeed * (elapsed / 1000000.f); // max change in elevation
	if (elevationTarget > lastElevation) {
		// Increase elevation towards target
		lastElevation += maxDelta;
		if (lastElevation > elevationTarget) {
			lastElevation = elevationTarget;
		}
	} else if (elevationTarget < lastElevation) {
		// Decrease elevation towards target
		lastElevation -= maxDelta;
		if (lastElevation < elevationTarget) {
			lastElevation = elevationTarget;
		}
	}
	lastElevationTime = currentTime;
	return lastElevation;
}

uint8_t servoLookup(float elevation) {
	// Simple linear mapping for demo purposes (0 to maxElevation maps to 0 to 180 degrees)
	return (uint8_t) map(elevation, 0, 1, 0, 180);
}

// Servo task - runs motor control loop at high frequency
void vServoTask(void* pvParameters) {
  while (true) {
	// Calculate target position with elevation and vibration
	elevationServo.writeDegrees(servoLookup(getSmoothedElevation()));
    vTaskDelay(pdMS_TO_TICKS(5));  // Run at 200 Hz
  }
}

void vUpdateTask(void* pvParameters) {
	(void)pvParameters;

	for (;;) {
		uart.updateData();
		lightState.update();
		
		if (connectedToDesktop) {
			// Plays vibration effects
			if (!hapticDriver.playQueuedEffects()) {
				// Resets vibration priority if no effects are playing
				if (!hapticDriver.isPlaying()) {
					currVibPriority = 0;
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void vMouseTask(void* pvParameters) {
	(void)pvParameters;
	uint32_t lastTransportDebugMs = 0;

	for (;;) {
		mouseDriver.update();
		
		if (leftButton.changeTo(HIGH)) {
			hapticDriver.playEffect(24); // Play a click effect
		} else if (leftButton.changeTo(LOW)) {
			hapticDriver.playEffect(25); // Stop effect on release
		}

		/*const uint32_t nowMs = millis();
		if (nowMs - lastTransportDebugMs >= 1000u) {
			lastTransportDebugMs = nowMs;
			mouseDriver.printTransportDebug(Serial);
		}*/
		
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
	xTaskCreate(vUpdateTask, "LightTask", 512, nullptr, 1, &updateTaskHandle);

	//Serial.begin(115200);
	
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

	//Serial.println("Init complete");

	// Initialize Songbird UART protocol
  	core = uart.getProtocol();
  	uart.begin(SERIAL_BAUD);

	// Register ping handler
	core->setHeaderHandler(PING, pingHandler);

	xTaskCreate(vMouseTask, "MouseTask", 2048, nullptr, 1, &mouseTaskHandle);
	xTaskCreate(vServoTask, "ServoTask", 512, nullptr, 2, &servoTaskHandle);
	// xTaskCreate(vDebugTask, "DebugTask", 1024, nullptr, 1, &debugTaskHandle);
	lightState.setEffect(lightIdle);
}

void loop() {
	vTaskDelay(pdMS_TO_TICKS(1000));
}

#endif // INTEGRATION_TESTING