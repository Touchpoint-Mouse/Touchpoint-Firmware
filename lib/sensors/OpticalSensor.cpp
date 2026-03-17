#include "OpticalSensor.h"
#include "SROM.h"

#ifdef ARDUINO_ARCH_RP2040
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#else
#include <avr/pgmspace.h>
#endif

// Constructor stores pin assignments and default PMW3389 SPI timing.

OpticalSensor::OpticalSensor(
	SPIClassRP2040& spi,
	uint8_t csPin,
	uint8_t intPin,
	uint8_t sckPin,
	uint8_t misoPin,
	uint8_t mosiPin
) :
	spi_(&spi),
	csPin_(csPin),
	intPin_(intPin),
	sckPin_(sckPin),
	misoPin_(misoPin),
	mosiPin_(mosiPin),
	initialized_(false),
	burstReady_(false),
	spiSettings_(2000000, MSBFIRST, SPI_MODE3) {}

bool OpticalSensor::begin() {
	// Configure GPIO and route the selected pins onto the SPI peripheral.
	pinMode(csPin_, OUTPUT);
	digitalWrite(csPin_, HIGH);
	pinMode(intPin_, INPUT_PULLUP);

	spi_->setSCK(sckPin_);
	spi_->setTX(mosiPin_);
	spi_->setRX(misoPin_);
	spi_->begin();

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
	spi_->beginTransaction(spiSettings_);
	spi_->transfer(PMW3389::Motion_Burst);
	delayMicroseconds(35);

	for (uint8_t i = 0; i < BURST_SIZE; i++) {
		burstData[i] = spi_->transfer(0);
	}

	delayMicroseconds(1);
	spi_->endTransaction();
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

void OpticalSensor::select() {
	digitalWrite(csPin_, LOW);
}

void OpticalSensor::deselect() {
	digitalWrite(csPin_, HIGH);
}

uint8_t OpticalSensor::readReg(uint8_t regAddr) {
	// Single-register read transaction with PMW3389 timing delays.
	select();
	spi_->beginTransaction(spiSettings_);
	spi_->transfer(regAddr & 0x7F);
	delayMicroseconds(35);
	uint8_t data = spi_->transfer(0);
	delayMicroseconds(1);
	spi_->endTransaction();
	deselect();
	delayMicroseconds(19);
	return data;
}

void OpticalSensor::writeReg(uint8_t regAddr, uint8_t value) {
	// Single-register write transaction with PMW3389 timing delays.
	select();
	spi_->beginTransaction(spiSettings_);
	spi_->transfer(regAddr | 0x80);
	spi_->transfer(value);
	delayMicroseconds(20);
	spi_->endTransaction();
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
	spi_->beginTransaction(spiSettings_);
	spi_->transfer(PMW3389::SROM_Load_Burst | 0x80);
	delayMicroseconds(15);

	for (int i = 0; i < firmware_length; i++) {
		uint8_t value = static_cast<uint8_t>(pgm_read_byte(firmware_data + i));
		spi_->transfer(value);
		delayMicroseconds(15);
	}

	spi_->endTransaction();
	deselect();

	// Verify SROM load and apply baseline run configuration.
	readReg(PMW3389::SROM_ID);
	writeReg(PMW3389::Config2, 0x00);
	writeReg(PMW3389::Config1, 0x15);
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
