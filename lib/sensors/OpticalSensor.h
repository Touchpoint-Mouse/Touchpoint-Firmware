#ifndef OPTICAL_SENSOR_H
#define OPTICAL_SENSOR_H

#include <Arduino.h>
#include <SPI.h>
#include "V0_2_Config.h"

// PMW3389 register map used by the driver implementation.
namespace PMW3389 {
constexpr uint8_t Product_ID = 0x00;
constexpr uint8_t Revision_ID = 0x01;
constexpr uint8_t Motion = 0x02;
constexpr uint8_t Delta_X_L = 0x03;
constexpr uint8_t Delta_X_H = 0x04;
constexpr uint8_t Delta_Y_L = 0x05;
constexpr uint8_t Delta_Y_H = 0x06;
constexpr uint8_t SQUAL = 0x07;
constexpr uint8_t Raw_Data_Sum = 0x08;
constexpr uint8_t Maximum_Raw_data = 0x09;
constexpr uint8_t Minimum_Raw_data = 0x0A;
constexpr uint8_t Shutter_Lower = 0x0B;
constexpr uint8_t Shutter_Upper = 0x0C;
constexpr uint8_t Control = 0x0D;
constexpr uint8_t Config1 = 0x0F;
constexpr uint8_t Config2 = 0x10;
constexpr uint8_t Angle_Tune = 0x11;
constexpr uint8_t Frame_Capture = 0x12;
constexpr uint8_t SROM_Enable = 0x13;
constexpr uint8_t Run_Downshift = 0x14;
constexpr uint8_t Rest1_Rate_Lower = 0x15;
constexpr uint8_t Rest1_Rate_Upper = 0x16;
constexpr uint8_t Rest1_Downshift = 0x17;
constexpr uint8_t Rest2_Rate_Lower = 0x18;
constexpr uint8_t Rest2_Rate_Upper = 0x19;
constexpr uint8_t Rest2_Downshift = 0x1A;
constexpr uint8_t Rest3_Rate_Lower = 0x1B;
constexpr uint8_t Rest3_Rate_Upper = 0x1C;
constexpr uint8_t Observation = 0x24;
constexpr uint8_t Data_Out_Lower = 0x25;
constexpr uint8_t Data_Out_Upper = 0x26;
constexpr uint8_t Raw_Data_Dump = 0x29;
constexpr uint8_t SROM_ID = 0x2A;
constexpr uint8_t Min_SQ_Run = 0x2B;
constexpr uint8_t Raw_Data_Threshold = 0x2C;
constexpr uint8_t Config5 = 0x2F;
constexpr uint8_t Power_Up_Reset = 0x3A;
constexpr uint8_t Shutdown = 0x3B;
constexpr uint8_t Inverse_Product_ID = 0x3F;
constexpr uint8_t LiftCutoff_Tune3 = 0x41;
constexpr uint8_t Angle_Snap = 0x42;
constexpr uint8_t LiftCutoff_Tune1 = 0x4A;
constexpr uint8_t Motion_Burst = 0x50;
constexpr uint8_t LiftCutoff_Tune_Timeout = 0x58;
constexpr uint8_t LiftCutoff_Tune_Min_Length = 0x5A;
constexpr uint8_t SROM_Load_Burst = 0x62;
constexpr uint8_t Lift_Config = 0x63;
constexpr uint8_t Raw_Data_Burst = 0x64;
constexpr uint8_t LiftCutoff_Tune2 = 0x65;
}

// PMW3389 optical sensor driver for RP2040 + Arduino-Pico SPI.
class OpticalSensor {
public:
  // One decoded sample from a Motion_Burst transfer.
  struct MotionData {
    int16_t deltaX = 0;
    int16_t deltaY = 0;
    uint8_t motion = 0;
    uint8_t squal = 0;
    uint8_t minRaw = 0;
    uint8_t maxRaw = 0;
    uint16_t shutter = 0;
    bool hasMotion = false;
    bool onSurface = false;
  };

  OpticalSensor(
    SPIClassRP2040& spi = SPI,
    uint8_t csPin = OPTICAL_CS,
    uint8_t intPin = OPTICAL_INT,
    uint8_t sckPin = OPTICAL_SCK,
    uint8_t misoPin = OPTICAL_MISO,
    uint8_t mosiPin = OPTICAL_MOSI
  );

  // Configures SPI pins, initializes sensor, and uploads SROM firmware.
  bool begin();

  // Polls motion burst registers and fills outData with decoded values.
  // Returns false when the sensor is not initialized.
  bool poll(MotionData& outData);

  // True after a successful begin() sequence.
  bool isInitialized() const;

private:
  static constexpr uint8_t BURST_SIZE = 12;

  SPIClassRP2040* spi_;
  uint8_t csPin_;
  uint8_t intPin_;
  uint8_t sckPin_;
  uint8_t misoPin_;
  uint8_t mosiPin_;
  bool initialized_;
  bool burstReady_;
  SPISettings spiSettings_;

  void select();
  void deselect();
  uint8_t readReg(uint8_t regAddr);
  void writeReg(uint8_t regAddr, uint8_t value);
  void uploadFirmware();
  void performStartup();
};

#endif // OPTICAL_SENSOR_H
