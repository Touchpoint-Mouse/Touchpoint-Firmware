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
#define MAX_VIBRATION_COMMANDS 4

// HydraFOC motor object - using pointer to control initialization timing
HydraFOCMotor* motor = nullptr;

// Elevation paramaters

const float maxElevation = 3.75f; // in radians
float lastElevation = 0.f;
float elevationTarget = 0.f;
float maxElevationSpeed = 0.f; // units per second (0 = no smoothing)
uint64_t lastElevationTime = 0;

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
	ELEVATION_SPEED = 0x11,
	VIBRATION = 0x20
};

// RTOS queue for vibration commands
typedef struct {
	float amplitude;
	float frequency;
	uint64_t endTime;
	bool indefinite;
} VibrationCommand;

QueueHandle_t vibrationQueue;

// Vibration parameters
VibrationCommand currVib = {0.f, 0.f, 0, false};
VibrationCommand nextVib = {0.f, 0.f, 0, false};
bool hasNext = false;

// Connection state
volatile bool connectedToDesktop = false;

// RTOS task function prototypes
void pingTask(void* pvParameters);
void updateTask(void* pvParameters);
void focTask(void* pvParameters);

// Ping handler
void pingHandler(std::shared_ptr<SongbirdCore::Packet> pkt) {
	// Respond to ping
	auto response = core->createPacket(PING);
	core->sendPacket(response);

	// Mark as connected
	connectedToDesktop = true;
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
	// Read amplitude from packet (unit is mm displacement)
	float amplitude = pkt->readFloat();
	// Read frequency from packet (unit is Hz)
	float frequency = pkt->readFloat();
	// Read duration from packet (unit is num cycles)
	uint16_t numCycles = pkt->readInt16();
	
	// Calculate endTime and indefinite flag
	uint64_t currentTime = micros();
	uint64_t endTime = 0;
	bool indefinite = false;
	
	if (frequency <= 0.f || numCycles == 0) {
		// Indefinite duration
		indefinite = true;
		endTime = 0;
	} else {
		// Calculate end time based on number of cycles
		uint64_t duration = (uint64_t) (numCycles / frequency * 1000000); // convert to microseconds
		endTime = currentTime + duration;
		indefinite = false;
	}
	
	// Adds vibration command to queue
	VibrationCommand cmd = {amplitude, frequency, endTime, indefinite};
	xQueueSend(vibrationQueue, &cmd, 0);
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

void goToNextVibration() {
	// Move to next vibration command if available
	if (hasNext) {
		currVib = nextVib;
		hasNext = false;
	} else {
		// No more commands, stop vibration
		currVib.amplitude = 0.f;
		currVib.frequency = 0.f;
		currVib.endTime = 0;
		currVib.indefinite = false;
	}
}

float getVibrationOffset() {
	// Check for new vibration commands
	if (!hasNext && xQueueReceive(vibrationQueue, &nextVib, 0) == pdTRUE) {
		hasNext = true;
		// If current vibration is infinite duration, interrupt it
		goToNextVibration();
	}
	// If amplitude or frequency is zero, no vibration
	if (currVib.amplitude <= 0.f || currVib.frequency <= 0.f) {
		return 0.f;
	}
	uint64_t currentTime = micros();
	// Check if duration has not been exceeded (indefinite = true means infinite)
	if (currVib.indefinite || currentTime < currVib.endTime) {
		// Calculate vibration offset
		return currVib.amplitude * sinf(2.f * PI * currVib.frequency * (currentTime / 1000000.f));
	} else {
		// Move to next vibration command if available
		goToNextVibration();

		return 0.f;
	}
}

// FOC task - runs motor control loop at high frequency
void focTask(void* pvParameters) {
  while (true) {
	// Calculate target position with elevation and vibration
	motor->setPosition((getSmoothedElevation() + getVibrationOffset()) * maxElevation);
    
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
    motor->begin(Direction::CCW, 5.34f, true);
    motor->resetEncoder();
	motor->setPosition(0.f);
  
	// Initialize Songbird UART protocol
  	core = uart.getProtocol();
  	uart.begin(SERIAL_BAUD);

	// Register ping handler
	core->setHeaderHandler(PING, pingHandler);

	// Register elevation feedback handler
	core->setHeaderHandler(ELEVATION, elevationFeedbackHandler);

	// Register vibration feedback handler
	core->setHeaderHandler(VIBRATION, vibrationFeedbackHandler);

	// Register elevation smoothing handler
	core->setHeaderHandler(ELEVATION_SPEED, elevationSmoothingHandler);
	
	// Create vibration command queue
	vibrationQueue = xQueueCreate(MAX_VIBRATION_COMMANDS, sizeof(VibrationCommand));

  	// Create RTOS tasks
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
