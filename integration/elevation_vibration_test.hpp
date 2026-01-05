#ifndef ELEVATION_VIBRATION_TEST
#define ELEVATION_VIBRATION_TEST

#include <Arduino.h>
#include <Button.h>
#include <memory>
#include "HydraFOCConfig.h"
#include "HydraFOCMotor.h"
#include "SongbirdCore.h"
#include "SongbirdUART.h"

#define SERIAL_BAUD 460800

// HydraFOC motor object - using pointer to control initialization timing
HydraFOCMotor* motor = nullptr;

// Maximum motor elevation
const float maxElevation = 3.75f; // in radians

// Last elevation value
float elevationTarget = 0.f;

// RTOS Task Handles
TaskHandle_t pingTaskHandle = NULL;
TaskHandle_t mouseTaskHandle = NULL;
TaskHandle_t updateTaskHandle = NULL;
TaskHandle_t focTaskHandle = NULL;

// Serial node object
SongbirdUART uart("UART");
// Serial protocol object
std::shared_ptr<SongbirdCore> core;

// Header enum
enum ElevationVibrationHeaders {
	PING = 0xFF,
	ELEVATION = 0x10,
	VIBRATION = 0x20
};

// Vibration parameters
float amplitude = 0.f;
float frequency = 0.f;
uint64_t startTime = 0;
uint64_t duration = 0;

// Connection state
volatile bool connectedToDesktop = false;

// RTOS task function prototypes
void pingTask(void* pvParameters);
void updateTask(void* pvParameters);
void focTask(void* pvParameters);

// Elevation feedback handler
void elevationFeedbackHandler(std::shared_ptr<SongbirdCore::Packet> pkt) {
	// Read elevation value from packet (float)
	float elevation = pkt->readFloat();
	// Constrain elevation between 0 and 1
	elevation = constrain(elevation, 0.f, 1.f);
	elevationTarget = elevation;
}

// Vibration feedback handler
void vibrationFeedbackHandler(std::shared_ptr<SongbirdCore::Packet> pkt) {
	// Read amplitude from packet (unit is mm displacement)
	amplitude = pkt->readFloat();
	// Read frequency from packet (unit is Hz)
	frequency = pkt->readFloat();
	// Read duration from packet (unit is num cycles)
	if (frequency <= 0.f) {
		duration = 0; // infinite duration
	} else {
		uint16_t numCycles = pkt->readInt16();
		duration = (uint64_t) (numCycles / frequency * 1000000); // convert to microseconds
	}
	// Sets start time
	startTime = micros();
}

float getVibrationOffset() {
	// Handle vibration effect
	if (amplitude > 0.f && frequency > 0.f) {
		return 0.f;
	}
	uint64_t elapsed = micros() - startTime;
	// Check if duration has not been exceeded (duration = 0 means infinite)
	if (duration == 0 || elapsed < duration) {
		// Calculate vibration offset
		return amplitude * sinf(2.f * PI * frequency * (elapsed / 1000000.f));
	} else {
		// Reset vibration parameters
		amplitude = 0.f;
		frequency = 0.f;
		duration = 0.f;

		return 0.f;
	}
}

// Ping task - sends ping to desktop and waits for response
void pingTask(void* pvParameters) {
  std::shared_ptr<SongbirdCore::Packet> response = nullptr;
  while (!response) {
    // Send ping
    auto pkt = core->createPacket(PING);
    core->sendPacket(pkt);
    
    // Wait for response
    response = core->waitForHeader(PING, 1000);
    core->flush();
  }
  
  connectedToDesktop = true;
  
  // Suspend this task - it's done
  vTaskSuspend(NULL);
}

// FOC task - runs motor control loop at high frequency
void focTask(void* pvParameters) {
  while (true) {
	// Calculate target position with elevation and vibration
	motor->setPosition((elevationTarget + getVibrationOffset()) * maxElevation);
    
    motor->update();  // Run FOC control loop
    taskYIELD();     // Allow other tasks to run (still very fast)
  }
}

// Update task - handles protocol processing
void updateTask(void* pvParameters) {
  while (true) {
    uart.updateData();

    vTaskDelay(1); // Yield to other tasks
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
  	// Configure I2C
    Wire.begin(I2C0_SDA, I2C0_SCL);

    // Configure driver pins
    pinMode(focDriverSleepPin, OUTPUT);
    pinMode(focDriverResetPin, OUTPUT);
    digitalWrite(focDriverSleepPin, HIGH); // Wake up driver
    digitalWrite(focDriverResetPin, HIGH); // Release reset

	// Create motor object AFTER GPIO configuration AND joystick calibration
	motor = new HydraFOCMotor(focMotorPins[0][0], focMotorPins[0][1], focMotorPins[0][2], 
	                          focMotorPins[0][3], focMotorPins[0][4], focMotorPins[0][5], 
	                          I2C1_SDA, focCurrentPins[0][0], focCurrentPins[0][1]);

	// Initialize motor AFTER joystick calibration
    motor->begin(Direction::CCW, 3.92f, true);
    motor->resetEncoder();
	motor->setPosition(0.f);
  
	// Initialize Songbird UART protocol
  	core = uart.getProtocol();
  	uart.begin(SERIAL_BAUD);

	// Register elevation feedback handler
	core->setHeaderHandler(ELEVATION, elevationFeedbackHandler);

	// Register vibration feedback handler
	core->setHeaderHandler(VIBRATION, vibrationFeedbackHandler);
  
  	// Create RTOS tasks
 	xTaskCreatePinnedToCore(
    	pingTask,           // Task function
    	"Ping_Task",        // Task name
    	4096,               // Stack size
    	NULL,               // Parameters
    	3,                  // Priority (highest - needs to connect first)
    	&pingTaskHandle,    // Task handle
    	0                   // Core 0
  	);
  
  	xTaskCreatePinnedToCore(
    	updateTask,         // Task function
    	"Update_Task",      // Task name
    	4096,               // Stack size
    	NULL,               // Parameters
    	1,                  // Priority (low - just processes data)
    	&updateTaskHandle,  // Task handle
    	0                   // Core 0 (moved to free Core 1 for FOC)
  	);
  
  	xTaskCreatePinnedToCore(
    	focTask,            // Task function
    	"FOC_Task",         // Task name
    	4096,               // Stack size
    	NULL,               // Parameters
    	4,                  // Priority (highest - real-time motor control)
    	&focTaskHandle,     // Task handle
    	1                   // Core 1 (dedicated to real-time control)
  	);
}

void loop() {
  	// Nothing to do here, all work is in RTOS tasks
  	vTaskDelay(portMAX_DELAY);
}

#endif // ELEVATION_VIBRATION_TEST
