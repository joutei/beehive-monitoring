#ifndef HIVE_H
#define HIVE_H

/******************************************************************************
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

//LoRa
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// Sleep/Low Power
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>

// Old Setup : Temp sensor MPU6050 ,accel gyro temp
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Temp humid sensor, SHT31
#include <Adafruit_SHT31.h>

//Neo6m GNSS
//#include <SoftwareSerial.h>
//#include <TinyGPS++.h>



//MPU6050
#include <Wire.h>
 //Analog port 4 (A4) = SDA (serial data)
//Analog port 5 (A5) = SCL (serial clock)
#define SIGNAL_PATH_RESET 0x68
#define I2C_SLV0_ADDR 0x37
#define ACCEL_CONFIG 0x1C
#define MOT_THR 0x1F // Motion detection threshold bits [7:0]
#define MOT_DUR 0x20 // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MOT_DETECT_CTRL 0x69
#define INT_ENABLE 0x38
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68
#define INT_STATUS 0x3A
//when nothing connected to AD0 than address is 0x68
#define ADO 0
#if ADO
#define MPU6050_ADDRESS 0x69 // Device address when ADO = 1
#else
#define MPU6050_ADDRESS 0x68 // Device address when ADO = 0
#endif




// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
constexpr PROGMEM u1_t NWKSKEY[16] = {0x80, 0x70, 0x8B, 0x65, 0x3A, 0xFC, 0xF5, 0x39, 0x58, 0xDE, 0xD5, 0x73, 0xA7, 0x12, 0xF0, 0xB3};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
constexpr u1_t PROGMEM APPSKEY[16] = {0x5B, 0xEC, 0xE3, 0x55, 0x78, 0x63, 0x52, 0x8E, 0x96, 0x4B, 0xEC, 0x44, 0xB8, 0xDC, 0x4D, 0x24};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
constexpr u4_t DEVADDR = 0x260B8ACB ; // <-- Change this address for every node!


// Pin mapping
extern const lmic_pinmap lmic_pins; 

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
constexpr unsigned TX_INTERVAL = 60; // seconds
constexpr unsigned CRIT_CHECK_INTERVAL = 500; // milliseconds
constexpr unsigned CRIT_CHECK_DURATION = 20; // milliseconds -- just a largely generous guess
constexpr unsigned UPDATE_INTERVAL = TX_INTERVAL;
constexpr unsigned UPDATE_DURATION = 150; // milliseconds -- I2C readings take time but it's a generous guess too
constexpr unsigned SLEEP_AND_WAKE_UP_DURATION = 8350; // milliseconds -- 8 seconds sleeping + 350ms wakeup routine

//Old Setup : MPU6050
extern Adafruit_MPU6050 mpu;

extern Adafruit_SHT31 sht31;

//Neo6m GPS
/*constexpr unsigned GPS_RX_PIN = A0;  // Set this to your chosen pin
constexpr unsigned GPS_TX_PIN = A1;  // Set this to your chosen pin
extern SoftwareSerial gpsSerial;
extern TinyGPSPlus gps;*/

//Limit switch (fin de course)
constexpr int limitSwitchPin = 5;
extern uint16_t openCount;
extern uint16_t closeCount;
extern uint32_t lastOpen;

//Added to reduce the processing load (updating mydata with no eventual changes)
extern bool updateOpen; // true : means openCount and lastOpen have changed and we should update mydata[] with the new data
extern bool updateClose; // true : means closeCount have changed and we should update mydata[] with the new data

extern uint8_t mydata[]; // For some reason we need an extra unused byte (for 13 slots "0-12" used we need 14)
extern osjob_t sendjob;
extern osjob_t readCriticalJob;
extern osjob_t updatejob;

// Power/Reset flag
extern uint8_t isReset; // 1 : the device has just been turned ON or reset


void do_send(osjob_t*);
void do_update(osjob_t*);
void do_read_critical(osjob_t*);
void onEvent (ev_t);
void initSensors();
void initAndSetupOS();







#endif