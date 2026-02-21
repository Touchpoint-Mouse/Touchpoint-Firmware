#ifndef PMW3389_INTERRUPT_TEST_HPP
#define PMW3389_TEST_HPP

#include <Arduino.h>
#include <SPI.h>
#include "V0_2_Config.h"
#include "SROM.h"

// For RP2040, data is in flash by default, no PROGMEM needed
#ifdef ARDUINO_ARCH_RP2040
  #define PROGMEM
  #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#else
  #include <avr/pgmspace.h>
#endif

// Registers
#define Product_ID  0x00
#define Revision_ID 0x01
#define Motion  0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum  0x08
#define Maximum_Raw_data  0x09
#define Minimum_Raw_data  0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Config1 0x0F
#define Config2 0x10
#define Angle_Tune  0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower  0x15
#define Rest1_Rate_Upper  0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower  0x18
#define Rest2_Rate_Upper  0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower  0x1B
#define Rest3_Rate_Upper  0x1C
#define Observation 0x24
#define Data_Out_Lower  0x25
#define Data_Out_Upper  0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run  0x2B
#define Raw_Data_Threshold  0x2C
#define Config5 0x2F
#define Power_Up_Reset  0x3A
#define Shutdown  0x3B
#define Inverse_Product_ID  0x3F
#define LiftCutoff_Tune3  0x41
#define Angle_Snap  0x42
#define LiftCutoff_Tune1  0x4A
#define Motion_Burst  0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length  0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst  0x64
#define LiftCutoff_Tune2  0x65

//Set this to what pin your "INT0" hardware interrupt feature is on
#define Motion_Interrupt_Pin OPTICAL_INT

const int ncs = OPTICAL_CS;  //This is the SPI "slave select" pin that the sensor is hooked up to

byte initComplete=0;
volatile int xydat[2];
volatile byte movementflag=0;
bool inBurst = false;  // Track if burst mode has been initialized

// SPI settings for PMW3389 - max 2MHz clock, MODE3
SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE3);

// function prototypes
void performStartup(void);
void dispRegisters(void);
void UpdatePointer(void);
void adns_com_begin(void);
void adns_com_end(void);
byte adns_read_reg(byte reg_addr);
void adns_write_reg(byte reg_addr, byte data);
void adns_upload_firmware(void);
int convTwosComp(int b);

void setup() {
  Serial.begin(115200);
  
  // Wait for serial connection on RP2040 (timeout after 5 seconds)
  unsigned long start = millis();
  while (!Serial && (millis() - start < 5000)) {
    delay(100);
  }
  
  Serial.println("\n\n=== PMW3389 Test Starting ===");
  Serial.print("Chip select pin: ");
  Serial.println(ncs);
  Serial.print("Interrupt pin: ");
  Serial.println(Motion_Interrupt_Pin);
  
  pinMode (ncs, OUTPUT);
  digitalWrite(ncs, HIGH);  // Start with CS high (inactive)
  
  pinMode(Motion_Interrupt_Pin, INPUT_PULLUP);

  // Configure SPI with pins from V0_2_Config.h
  SPI.setSCK(OPTICAL_SCK);
  SPI.setTX(OPTICAL_MOSI);
  SPI.setRX(OPTICAL_MISO);
  SPI.begin();

  performStartup();  
  
  delay(5000);
  
  dispRegisters();
  
  // Check surface quality and sensor status
  byte squal = adns_read_reg(SQUAL);
  Serial.print("Surface Quality (SQUAL): ");
  Serial.println(squal);
  
  initComplete=9;
  Serial.println("Ready for motion detection");
  
  // Initialize burst mode once at startup (as per reference implementation)
  adns_write_reg(Motion_Burst, 0x00);
  inBurst = true;
}

void adns_com_begin(){
  digitalWrite(ncs, LOW);
}

void adns_com_end(){
  digitalWrite(ncs, HIGH);
}

byte adns_read_reg(byte reg_addr){
  adns_com_begin();
  SPI.beginTransaction(spiSettings);
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(35); // tSRAD (per reference implementation and datasheet)
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  SPI.endTransaction();
  adns_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}

void adns_write_reg(byte reg_addr, byte data){
  adns_com_begin();
  SPI.beginTransaction(spiSettings);
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  SPI.endTransaction();
  adns_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
}

void adns_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware...");

  //Write 0 to Rest_En bit of Config2 register to disable Rest mode.
  adns_write_reg(Config2, 0x00);
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10);
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin();
  SPI.beginTransaction(spiSettings);
  SPI.transfer(SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  SPI.endTransaction();
  adns_com_end();

  //Read the SROM_ID register to verify the ID before any other register reads or writes.
  byte srom_id = adns_read_reg(SROM_ID);
  Serial.print("SROM ID: 0x");
  Serial.println(srom_id, HEX);

  //Write 0x00 to Config2 register for wired mouse or 0x20 for wireless mouse design.
  adns_write_reg(Config2, 0x00);

  // set initial CPI resolution (0x15 = ~2100 CPI)
  adns_write_reg(Config1, 0x15);
  
  // Configure lift detection - use default/sensitive settings
  // Lower values = more sensitive to surface contact
  adns_write_reg(Lift_Config, 0x02);  // 2mm lift height (default)
  delay(1);
  
  // Try adjusting Min_SQ_Run for better surface tracking
  // This sets minimum SQUAL for valid motion (default is usually 0x04)
  adns_write_reg(Min_SQ_Run, 0x01);  // Lower threshold to accept lower quality surfaces
  delay(1);
  
  // Additional configuration for proper operation
  delay(10);
  
  // Clear any motion data
  adns_read_reg(Motion);
  adns_read_reg(Delta_X_L);
  adns_read_reg(Delta_X_H);
  adns_read_reg(Delta_Y_L);
  adns_read_reg(Delta_Y_H);
  
  delay(10);
  
  adns_com_end();
  }


void performStartup(void){
  Serial.println("Resetting sensor...");
  adns_com_end(); // ensure that the serial port is reset
  adns_com_begin(); // ensure that the serial port is reset
  adns_com_end(); // ensure that the serial port is reset
  
  adns_write_reg(Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(Motion);
  adns_read_reg(Delta_X_L);
  adns_read_reg(Delta_X_H);
  adns_read_reg(Delta_Y_L);
  adns_read_reg(Delta_Y_H);
  
  // upload the firmware
  adns_upload_firmware();
  delay(10);
  Serial.println("Optical Chip Initialized");
  Serial.flush();
  }

void UpdatePointer(void){
  if(initComplete==9){

    // Read motion registers - must read in sequence
    byte motion = adns_read_reg(Motion);
    
    // Check if motion bit is actually set
    bool hasMotion = (motion & 0x80) != 0;
    
    // Read 16-bit X and Y delta values
    int xl = adns_read_reg(Delta_X_L);
    int xh = adns_read_reg(Delta_X_H);
    int yl = adns_read_reg(Delta_Y_L);
    int yh = adns_read_reg(Delta_Y_H);
    
    // Combine high and low bytes into 16-bit signed values
    xydat[0] = (xh << 8) | xl;
    xydat[1] = (yh << 8) | yl;
    
    // Only set movementflag if there's actual motion
    if(hasMotion || xydat[0] != 0 || xydat[1] != 0) {
      movementflag=1;
    }
    }
  }

void dispRegisters(void){
  int oreg[4] = {0x00, 0x3F, 0x2A, 0x02};
  const char* oregname[] = {"Product_ID", "Inverse_Product_ID", "SROM_Version", "Motion"};
  
  for(int rctr=0; rctr<4; rctr++){
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.print("Register 0x");
    Serial.print(oreg[rctr], HEX);
    Serial.print(": ");
    
    byte regres = adns_read_reg(oreg[rctr]);
    
    Serial.print("0x");
    Serial.print(regres, HEX);
    Serial.print(" (");
    Serial.print(regres, BIN);
    Serial.println(")");
  }
}


int convTwosComp(int b){
  //Convert from 2's complement for 16-bit values
  if(b & 0x8000){
    b = -1 * ((b ^ 0xffff) + 1);
    }
  return b;
  }
  

void loop() {
  // Poll for motion using Motion_Burst (as per PMW3389 reference implementation)
  static unsigned long lastCheck = 0;
  unsigned long currentTime = millis();
  
  if(currentTime - lastCheck >= 100) {  // Check every 100ms
    lastCheck = currentTime;
    
    // Read Motion_Burst following reference implementation pattern
    adns_com_begin();
    SPI.beginTransaction(spiSettings);
    
    // Send Motion_Burst register address (no masking needed, it's 0x50)
    SPI.transfer(Motion_Burst);
    delayMicroseconds(35);  // tSRAD_MOTBR - wait for data to be ready
    
    // Read 12 bytes of burst data
    byte burstData[12];
    for(int i = 0; i < 12; i++) {
      burstData[i] = SPI.transfer(0);
    }
    
    delayMicroseconds(1);  // tSCLK-NCS for read operation
    SPI.endTransaction();
    adns_com_end();
    
    // Parse burst data according to PMW3389 Motion_Burst format:
    // [0] Motion, [1] Observation, [2] Delta_X_L, [3] Delta_X_H,
    // [4] Delta_Y_L, [5] Delta_Y_H, [6] SQUAL, [7] Raw_Data_Sum,
    // [8] Maximum_Raw_Data, [9] Minimum_Raw_Data, [10] Shutter_Upper, [11] Shutter_Lower
    byte motion = burstData[0];
    int xl = burstData[2];
    int xh = burstData[3];
    int yl = burstData[4];
    int yh = burstData[5];
    byte squal = burstData[6];
    int shutter = (burstData[10] << 8) | burstData[11];
    byte maxRaw = burstData[8];
    byte minRaw = burstData[9];
    
    // Combine to 16-bit signed values
    int deltaX = (xh << 8) | xl;
    int deltaY = (yh << 8) | yl;
    
    Serial.print("Motion: 0x");
    Serial.print(motion, HEX);
    Serial.print(" | SQUAL: ");
    Serial.print(squal);
    Serial.print(" | Shutter: ");
    Serial.print(shutter);
    Serial.print(" | Raw[");
    Serial.print(minRaw);
    Serial.print("-");
    Serial.print(maxRaw);
    Serial.print("]");
    
    bool hasMotion = (motion & 0x80) != 0;  // Motion detected bit
    bool onSurface = (motion & 0x08) == 0;  // Surface bit (0=on surface, 1=lifted)
    
    // Show raw delta values to debug surface tracking
    Serial.print(" | dX: ");
    Serial.print(convTwosComp(deltaX));
    Serial.print(" dY: ");
    Serial.print(convTwosComp(deltaY));
    
    if(hasMotion && onSurface) {
      Serial.println(" [MOTION ON SURFACE]");
    } else if(!hasMotion && onSurface) {
      Serial.println(" [ON SURFACE, no motion]");
    } else if(!onSurface && hasMotion) {
      Serial.println(" [LIFTED, with motion]");
    } else {
      Serial.println(" [LIFTED, no motion]");
    }
  }

  if(movementflag){
    
    //uncomment below if using Teensy from PJRC to move mouse on screen
    //Mouse.move(convTwosComp(xydat[0]),convTwosComp(xydat[1]));
    
    Serial.print("x = ");
    Serial.print( convTwosComp(xydat[0]) );
    Serial.print(" | ");
    Serial.print("y = ");
    Serial.println( convTwosComp(xydat[1]) );
    
    movementflag=0;
    }
    
  }

#endif // PMW3389_TEST_HPP
