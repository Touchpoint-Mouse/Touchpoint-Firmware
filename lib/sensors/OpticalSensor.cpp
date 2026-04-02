#include "OpticalSensor.h"
#include "SROM.h"

#ifdef ARDUINO_ARCH_RP2040
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#else
#include <avr/pgmspace.h>
#endif

OpticalSensor::OpticalSensor() :
	spi(nullptr),
	csPin_(255),
	intPin_(255),
	initialized_(false),
	burstReady_(false),
	cpi_(1100),
	spiSettings(2000000, MSBFIRST, SPI_MODE3) {}

bool OpticalSensor::begin(SPIClass* spiPort, uint8_t csPin, uint8_t intPin) {
	if (spiPort == nullptr) {
		return false;
	}

	spi = spiPort;
	csPin_ = csPin;
	intPin_ = intPin;

	// Configure GPIO and initialize the selected SPI peripheral.
	pinMode(csPin_, OUTPUT);
	digitalWrite(csPin_, HIGH);
	pinMode(intPin_, INPUT_PULLUP);

	spi->begin();

	performStartup();

	// Prime burst mode once after startup.
	writeReg(PMW3389::Motion_Burst, 0x00);
	burstReady_ = true;
	initialized_ = true;
	return true;
}

bool OpticalSensor::poll(MotionData& outData) {
	if (!initialized_ || !burstReady_) {
		return false;
	}

	// PMW3389 Motion_Burst packet is 12 bytes.
	uint8_t burstData[BURST_SIZE] = {0};

	select();
	spi->beginTransaction(spiSettings);
	spi->transfer(PMW3389::Motion_Burst);
	delayMicroseconds(35);

	for (uint8_t i = 0; i < BURST_SIZE; i++) {
		burstData[i] = spi->transfer(0);
	}

	delayMicroseconds(1);
	spi->endTransaction();
	deselect();

	// Decode burst packet fields.
	outData.motion = burstData[0];
	outData.squal = burstData[6];
	outData.maxRaw = burstData[8];
	outData.minRaw = burstData[9];
	outData.shutter = static_cast<uint16_t>(burstData[10] << 8) | burstData[11];

	const int16_t deltaX = static_cast<int16_t>((burstData[3] << 8) | burstData[2]);
	const int16_t deltaY = static_cast<int16_t>((burstData[5] << 8) | burstData[4]);
	outData.deltaX = deltaX;
	outData.deltaY = deltaY;

	// Motion bit 7 indicates movement; bit 3 indicates lift when set.
	outData.hasMotion = (outData.motion & 0x80) != 0;
	outData.onSurface = (outData.motion & 0x08) == 0;
	return true;
}

bool OpticalSensor::isInitialized() const {
	return initialized_;
}

bool OpticalSensor::setCpi(uint16_t cpi) {
	if (!initialized_) {
		return false;
	}

	if (cpi < 50) {
		cpi = 50;
	}
	if (cpi > 16000) {
		cpi = 16000;
	}

	const uint8_t regValue = static_cast<uint8_t>((cpi / 50) - 1);
	writeReg(PMW3389::Config1, regValue);
	cpi_ = static_cast<uint16_t>((regValue + 1) * 50);
	return true;
}

uint16_t OpticalSensor::getCpi() const {
	return cpi_;
}

void OpticalSensor::select() {
	digitalWrite(csPin_, LOW);
}

void OpticalSensor::deselect() {
	digitalWrite(csPin_, HIGH);
}

uint8_t OpticalSensor::readReg(uint8_t regAddr) {
	// Single-register read transaction with PMW3389 timing delays.
	select();
	spi->beginTransaction(spiSettings);
	spi->transfer(regAddr & 0x7F);
	delayMicroseconds(35);
	uint8_t data = spi->transfer(0);
	delayMicroseconds(1);
	spi->endTransaction();
	deselect();
	delayMicroseconds(19);
	return data;
}

void OpticalSensor::writeReg(uint8_t regAddr, uint8_t value) {
	// Single-register write transaction with PMW3389 timing delays.
	select();
	spi->beginTransaction(spiSettings);
	spi->transfer(regAddr | 0x80);
	spi->transfer(value);
	delayMicroseconds(20);
	spi->endTransaction();
	deselect();
	delayMicroseconds(100);
}

void OpticalSensor::uploadFirmware() {
	// SROM upload sequence from PMW3389 datasheet.
	writeReg(PMW3389::Config2, 0x00);
	writeReg(PMW3389::SROM_Enable, 0x1D);
	delay(10);
	writeReg(PMW3389::SROM_Enable, 0x18);

	select();
	spi->beginTransaction(spiSettings);
	spi->transfer(PMW3389::SROM_Load_Burst | 0x80);
	delayMicroseconds(15);

	for (int i = 0; i < firmware_length; i++) {
		uint8_t value = static_cast<uint8_t>(pgm_read_byte(firmware_data + i));
		spi->transfer(value);
		delayMicroseconds(15);
	}

	spi->endTransaction();
	deselect();

	// Verify SROM load and apply baseline run configuration.
	readReg(PMW3389::SROM_ID);
	writeReg(PMW3389::Config2, 0x00);
	writeReg(PMW3389::Config1, static_cast<uint8_t>((cpi_ / 50) - 1));
	writeReg(PMW3389::Lift_Config, 0x02);
	delay(1);
	writeReg(PMW3389::Min_SQ_Run, 0x01);
	delay(1);

	delay(10);
	readReg(PMW3389::Motion);
	readReg(PMW3389::Delta_X_L);
	readReg(PMW3389::Delta_X_H);
	readReg(PMW3389::Delta_Y_L);
	readReg(PMW3389::Delta_Y_H);
	delay(10);
}

void OpticalSensor::performStartup() {
	// Bring the interface to a known state before reset + firmware load.
	deselect();
	select();
	deselect();

	writeReg(PMW3389::Power_Up_Reset, 0x5A);
	delay(50);

	readReg(PMW3389::Motion);
	readReg(PMW3389::Delta_X_L);
	readReg(PMW3389::Delta_X_H);
	readReg(PMW3389::Delta_Y_L);
	readReg(PMW3389::Delta_Y_H);

	uploadFirmware();
	delay(10);
}
