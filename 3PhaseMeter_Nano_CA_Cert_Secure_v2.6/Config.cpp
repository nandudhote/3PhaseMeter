#include "Config.h"  // INCLUDE CONFIG HEADER

String ServerMQTT = "a6kvi1np2cmrt-ats.iot.ap-south-1.amazonaws.com";  // MQTT BROKER ADDRESS
int MqttPort = 8883;                                                   // MQTT BROKER
String mqttUserName = "NONE";                                          // MQTTT USER NAME
String mqttUserPassword = "NONE";                                      // MQTT USER PASSWORD

String deviceID = "861192075437379";                // DEVICE ID
String mqttPublishTopic = deviceID + "/status";     // MQTT PUBLISH TOPIC
String mqttSubscribeTopic = deviceID + "/control";  // MQTT SUBSCRIBE TOPIC

const char _volDivPinOne = A1;      // PHASE 1 VOLTAGE READ ADC PIN
const char _volDivPinTwo = A2;      // PHASE 2 VOLTAGE READ ADC PIN
const char _volDivPinThree = A3;    // PHASE 3 VOLTAGE READ ADC PIN
const char _contactorPin = 2;       // CONTACTOR RELAY OUTPUT PIN
const char _lteResetPin = 3;        // LTE RESET RELAY OUTPUT PIN
const char _statusLEDPin = A0;      // STATUS LED OUTPUT PIN
const char _measurementLEDPin = 8;  // MEASUREMENT LED OUTPUT PIN

const byte powerOneInKwhEEPROMAdd = 0;     // PHASE 1 POWER IN KWH STORING EEPROM ADDRESS
const byte powerTwoInKwhEEPROMAdd = 32;    // PHASE 2 POWER IN KWH STORING EEPROM ADDRESS
const byte powerThreeInKwhEEPROMAdd = 64;  // PHASE 3 POWER IN KWH STORING EEPROM ADDRESS
const byte loadStateEEPROMAdd = 100;       // LOAD STATUS STORING EEPROM ADDRESS

const byte CT1I2cAddr = 0x0A;     // CT1 MCU I2C ADDRESS
const byte CT2I2cAddr = 0x10;     // CT2 MCU I2C ADDRESS
const byte CT3I2cAddr = 0x52;     // CT3 MCU I2C ADDRESS
const byte eepromI2cAddr = 0x50;  // EEPROM IC I2C ADDRESS

byte lteRestartCounter = 2;      // THIS VARIABLE HOLD THE COUNT, HOW MANY TIMES LTE MODULE FAILED TO SEND DATA OVER MQTT, IF MODULE FAILED TO SEND MESSAGE TWO TIMES THEN MODULE WILL REBEGIN, INITIALLY ITS 2 TO BEGIN THE MODULE AFTER 1 MINTS OF POWER UP THE DEVICE
volatile byte mqttRecMsg;        // HOLD MESSAGE RECEIVED FROM MQTT (FROM MINI)
unsigned long startMillis = 0;   // HOLD START MILLI SECONDS TIMING FOR 1 MINTS DATA SEND LOOP
unsigned long previousTime = 0;  // HOLD RESET PREVIOUS TIME FOR 1 MINTS DATA SEND LOOP
bool lteBeginFailed = false;     // ENABLE IF BEGIN FAILED // MEANS MODULE UNABLE TO CONNECT WITH MQTT AND FAILED TO SUBSCRIBE THE MQTT TOPIC
int sampleCount = 0;             // INCREMENT, HOW MANY TIMES VOLTAGE AND CURRENT SAMPLES TAKEN
byte loadState = 0;              // HOLD THE CONTACTOR STATE
bool requestOn200 = false;       // IF RECEIVE REQUEST OVER MQTT TO SEND CURRENT DATA
bool ledState = false;           // STATUS AND MEASUREMENT LED'S STATUS FOR TOGGLE
bool lteRestarted = false;       // INDICATE THE LTE MODULE HAS RESTARTED // TO INITIATE REBEGIN PROCESS
bool gotNewLoadStatus = false;   // IF CONTACTOR TRIGGER MESSAGE RECEIVED FROM MQTT // TO STORE THIS LOAD STATUS USE THIS FLAG
byte dataStoreCounter = 0;       // WRITE THE DATA IN EEPROM ONCE IN 5 MINUTS // HOLD THE COUNT TO MEASURE 5 MINTS
