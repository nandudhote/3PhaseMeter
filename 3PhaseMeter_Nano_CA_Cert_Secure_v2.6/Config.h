#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"

extern String ServerMQTT;
extern int MqttPort;
extern String mqttUserName;
extern String mqttUserPassword;

extern String deviceID;
extern String mqttPublishTopic;
extern String mqttSubscribeTopic;

extern const char _volDivPinOne;
extern const char _volDivPinTwo;
extern const char _volDivPinThree;
extern const char _contactorPin;
extern const char _lteResetPin;
extern const char _statusLEDPin;
extern const char _measurementLEDPin;
extern const byte powerOneInKwhEEPROMAdd;
extern const byte powerTwoInKwhEEPROMAdd;
extern const byte powerThreeInKwhEEPROMAdd;
extern const byte loadStateEEPROMAdd;
extern const byte CT1I2cAddr;
extern const byte CT2I2cAddr;
extern const byte CT3I2cAddr;
extern const byte eepromI2cAddr;
extern byte lteRestartCounter;
extern volatile byte mqttRecMsg;
extern unsigned long startMillis;
extern unsigned long previousTime;
extern bool lteBeginFailed;
extern int sampleCount;
extern byte loadState;
extern bool requestOn200;
extern bool ledState;
extern bool lteRestarted;
extern bool gotNewLoadStatus;
extern byte dataStoreCounter;

#endif
