#ifndef UART_COMMAND_TEST
#define UART_COMMAND_TEST

#include <Arduino.h>
#include <Button.h>
#include <memory>
#include "HydraFOCConfig.h"
#include "SongbirdCore.h"
#include "SongbirdUART.h"

#define SERIAL_BAUD 460800

// Joystick x coordinate
#define xPin SPI_MOSI

// Joystick y coordinate
#define yPin SPI_SCK

// RTOS Task Handles
TaskHandle_t pingTaskHandle = NULL;
TaskHandle_t mouseTaskHandle = NULL;
TaskHandle_t updateTaskHandle = NULL;

// Serial node object
SongbirdUART uart("Mouse UART");
// Serial protocol object
std::shared_ptr<SongbirdCore> core;

// Joystick button
Button joystickButton(SPI_CS, true);

// Joystick calibration
int xCenter = 2910;  // ADC center value (12-bit ADC: 0-4095)
int yCenter = 2910;
int deadzone = 20;  // Deadzone radius in ADC units

// Mouse movement parameters
float mouseSensitivity = 25.0f;  // Pixels per unit of joystick deflection
int8_t maxMouseSpeed = 127;      // Max mouse speed (-127 to 127)

// Mouse data streaming interval (ms)
const uint32_t mouseUpdateInterval = 10;  // 100Hz update rate
// Connection state
volatile bool connectedToDesktop = false;

// RTOS task function prototypes
void pingTask(void* pvParameters);
void mouseTask(void* pvParameters);
void updateTask(void* pvParameters);

// Joystick processing function
float processAxis(int rawValue, int center, int deadzone) {
    // Calculate offset from center
    int offset = rawValue - center;
    
    // Apply deadzone
    if (abs(offset) < deadzone) {
        return 0.0f;
    }
    
    // Map to -1.0 to 1.0 range outside deadzone
    float maxRange = max(center - deadzone, 4095 - center - deadzone);
    float adjustedOffset = (offset > 0) ? (offset - deadzone) : (offset + deadzone);
    
    return constrain(adjustedOffset / maxRange, -1.0f, 1.0f);
}

// Ping task - sends ping to desktop and waits for response
void pingTask(void* pvParameters) {
  std::shared_ptr<SongbirdCore::Packet> response = nullptr;
  while (!response) {
    // Send ping
    auto pkt = core->createPacket(0xFF);
    core->sendPacket(pkt);
    
    // Wait for response
    response = core->waitForHeader(0xFF, 1000);
    core->flush();
  }
  
  connectedToDesktop = true;
  
  // Suspend this task - it's done
  vTaskSuspend(NULL);
}

// Mouse task - streams mouse data at regular intervals
void mouseTask(void* pvParameters) {
  // Wait for connection to desktop
  while (!connectedToDesktop) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
  TickType_t lastWakeTime = xTaskGetTickCount();
  
  while (true) {
    // Read raw values
    int xRaw = analogRead(xPin);
    int yRaw = analogRead(yPin);
    
    // Process with center and deadzone
    float xProcessed = processAxis(xRaw, xCenter, deadzone);
    float yProcessed = processAxis(yRaw, yCenter, deadzone);
    
    // Convert to mouse movement (scale and constrain)
    int8_t mouseX = constrain((int)(xProcessed * mouseSensitivity), -maxMouseSpeed, maxMouseSpeed);
    int8_t mouseY = constrain((int)(yProcessed * mouseSensitivity), -maxMouseSpeed, maxMouseSpeed);
    
	// Send mouse data if there is movement
	if (mouseX != 0 || mouseY != 0) {
		// Create and send mouse data packet (header 0x01 for mouse data)
		// Payload: [mouseX (int8), mouseY (int8)]
		auto pkt = core->createPacket(0x01);
		pkt.writeByte((uint8_t)mouseX);
		pkt.writeByte((uint8_t)mouseY);
		core->sendPacket(pkt);
	}
    

    // Wait for next interval
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(mouseUpdateInterval));
  }
}

// Update task - handles protocol processing
void updateTask(void* pvParameters) {
  while (true) {
    uart.updateData();
	joystickButton.update();

	// Sends button packet if button event
	if (joystickButton.change()) {
		// Create and send button data packet (header 0x02 for button data)
		auto pkt = core->createPacket(0x02);
		pkt.writeByte(joystickButton.state() ? 1 : 0);
		core->sendPacket(pkt);
	}

    vTaskDelay(1); // Yield to other tasks
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
  // Configure pins
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  
  // Calibrate center position
  delay(100);
  xCenter = analogRead(xPin);
  yCenter = analogRead(yPin);
  
  // Initialize Songbird UART protocol
  core = uart.getProtocol();
  uart.begin(SERIAL_BAUD);
  
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
    mouseTask,          // Task function
    "Mouse_Task",       // Task name
    4096,               // Stack size
    NULL,               // Parameters
    2,                  // Priority (medium)
    &mouseTaskHandle,   // Task handle
    0                   // Core 0
  );
  
  xTaskCreatePinnedToCore(
    updateTask,         // Task function
    "Update_Task",      // Task name
    4096,               // Stack size
    NULL,               // Parameters
    1,                  // Priority (low - just processes data)
    &updateTaskHandle,  // Task handle
    1                   // Core 1
  );
}

void loop() {
  // Nothing to do here, all work is in RTOS tasks
  vTaskDelay(portMAX_DELAY);
}

#endif // UART_COMMAND_TEST
