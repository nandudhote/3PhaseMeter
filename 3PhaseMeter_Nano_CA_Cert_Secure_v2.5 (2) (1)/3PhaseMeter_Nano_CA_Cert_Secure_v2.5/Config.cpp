#include "Config.h"

// String ServerMQTT = "mqtt.evoluzn.in";
String ServerMQTT = "a6kvi1np2cmrt-ats.iot.ap-south-1.amazonaws.com";
int MqttPort = 8883;
String mqttUserName = "NONE";
String mqttUserPassword = "NONE";

String deviceID = "861192075437379";
String mqttPublishTopic = deviceID + "/status";
String mqttSubscribeTopic = deviceID + "/control";

const char _volDivPinOne = A1;
const char _volDivPinTwo = A2;
const char _volDivPinThree = A3;
const char _contactorPin = 2;
const char _lteResetPin = 3;        //D3
const char _statusLEDPin = A0;      //A0
const char _measurementLEDPin = 8;  //A6

const byte powerOneInKwhEEPROMAdd = 0;
const byte powerTwoInKwhEEPROMAdd = 32;
const byte powerThreeInKwhEEPROMAdd = 64;
const byte loadStateEEPROMAdd = 100;

const byte CT1I2cAddr = 0x0A;
const byte CT2I2cAddr = 0x10;
const byte CT3I2cAddr = 0x52;
const byte eepromI2cAddr = 0x50;

int dataCounter = 0;
byte lteRestartCounter = 2;
volatile byte mqttRecMsg;

unsigned long startMillis = 0;
unsigned long previousTime = 0;

byte durationCounter = 0;

unsigned long hoursTime = 0;
bool enableDataReading = false;

const unsigned long eventInterval = 29000;
unsigned long previousMillis = 0;

bool lteResetInProgress = false;
unsigned long lteResetStartTime = 0;
bool lteBeginFailed = false;

int sampleCount = 0;

byte loadState = 0;
bool requestOn200 = false;
bool ledState = false;
bool lteRestarted = false;
bool gotNewLoadStatus = false;

byte dataStoreCounter = 0;
