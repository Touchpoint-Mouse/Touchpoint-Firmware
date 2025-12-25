#ifndef HID_COMMAND_TEST
#define HID_COMMAND_TEST

#include <Arduino.h>
#include <Button.h>
#include "Adafruit_TinyUSB.h"
#include "HydraFOCConfig.h"

// Joystick x coordinate
#define xPin SPI_MOSI

// Joystick y coordinate
#define yPin SPI_SCK

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_MOUSE()
};

// USB HID object
Adafruit_USBD_HID usb_hid;

// Joystick button
Button joystickButton(SPI_CS, true);

// Joystick calibration
int xCenter = 2910;  // ADC center value (12-bit ADC: 0-4095)
int yCenter = 2910;
int deadzone = 20;  // Deadzone radius in ADC units

// Mouse movement parameters
float mouseSensitivity = 10.0f;  // Pixels per unit of joystick deflection
int8_t maxMouseSpeed = 127;      // Max mouse speed (-127 to 127)

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

// the setup function runs once when you press reset or power the board
void setup() {
  	Serial.begin(SERIAL_BAUD_RATE);
	
	// Wait longer for Serial - TinyUSB needs more time
	delay(2000);
	
	Serial.println("=== HID Command Test Starting ===");
	Serial.flush();
	delay(100);

  	pinMode(xPin, INPUT);
 	pinMode(yPin, INPUT);
  
  	// Calibrate center position (read initial values)
	delay(100);
    xCenter = analogRead(xPin);
    yCenter = analogRead(yPin);

	Serial.print("Joystick calibrated - X center: ");
	Serial.print(xCenter);
	Serial.print(", Y center: ");
	Serial.println(yCenter);
	Serial.flush();
	delay(100);

	Serial.println("Starting USB HID...");
	Serial.flush();
	delay(100);
	
	// Manual begin() is required on core without built-in support e.g. mbed rp2040
  	if (!TinyUSBDevice.isInitialized()) {
    	TinyUSBDevice.begin(0);
  	}

  	// Set up HID
  	usb_hid.setBootProtocol(HID_ITF_PROTOCOL_MOUSE);
  	usb_hid.setPollInterval(2);
  	usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  	usb_hid.setStringDescriptor("TinyUSB Mouse");
  	usb_hid.begin();

  	// If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  	if (TinyUSBDevice.mounted()) {
    	TinyUSBDevice.detach();
    	delay(10);
    	TinyUSBDevice.attach();
  	}

  	Serial.println("Adafruit TinyUSB HID Mouse example");
}

void process_hid() {
  	// Update button state
    joystickButton.update();
    
    // Read raw values
    int xRaw = analogRead(xPin);
    int yRaw = analogRead(yPin);
    
    // Process with center and deadzone
    float xProcessed = processAxis(xRaw, xCenter, deadzone);
    float yProcessed = processAxis(yRaw, yCenter, deadzone);
    
    // Convert to mouse movement (scale and constrain)
    int8_t mouseX = constrain((int)(xProcessed * mouseSensitivity), -maxMouseSpeed, maxMouseSpeed);
    int8_t mouseY = constrain((int)(yProcessed * mouseSensitivity), -maxMouseSpeed, maxMouseSpeed);

  	// Log button changes
  	if (joystickButton.change()) {
    	Serial.print("Button ");
    	Serial.println(joystickButton.state() ? "Pressed" : "Released");
  	}

  	// Log data periodically
  	static uint32_t logMs = 0;
  	if (millis() - logMs > 100) {
    	logMs = millis();
    	Serial.print("X: ");
    	Serial.print(xRaw);
    	Serial.print(" Y: ");
    	Serial.print(yRaw);
    	Serial.print(" -> mX: ");
    	Serial.print(mouseX);
    	Serial.print(" mY: ");
    	Serial.print(mouseY);
    	Serial.print(" Btn: ");
    	Serial.println(joystickButton.state());
  	}

  	// nothing to do if button is not pressed and joystick has no movement
  	bool btn_pressed = joystickButton.state();
  	if (!btn_pressed && mouseX == 0 && mouseY == 0) return;

  	// Remote wakeup
  	if (TinyUSBDevice.suspended()) {
    	// Wake up host if we are in suspend mode
    	// and REMOTE_WAKEUP feature is enabled by host
    	TinyUSBDevice.remoteWakeup();
  	}

  	if (usb_hid.ready()) {
    	uint8_t const report_id = 0; // no ID
    	int8_t const delta = 5;
    	usb_hid.mouseMove(report_id, mouseX, mouseY);
  	}
}

void loop() {
  	#ifdef TINYUSB_NEED_POLLING_TASK
  	// Manual call tud_task since it isn't called by Core's background
  	TinyUSBDevice.task();
	#endif

  	// Check mounting status
  	static bool wasMounted = false;
  	bool isMounted = TinyUSBDevice.mounted();
  	
  	if (isMounted && !wasMounted) {
    	Serial.println("USB Device mounted!");
    	wasMounted = true;
  	} else if (!isMounted && wasMounted) {
    	Serial.println("USB Device unmounted!");
    	wasMounted = false;
  	}

  	// poll gpio once each 10 ms
  	static uint32_t ms = 0;
  	if (millis() - ms > 10) {
    	ms = millis();
    	process_hid();
  	}
}

#endif