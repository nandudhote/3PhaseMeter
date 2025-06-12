#ifndef INTERNALDRIVERS_H  // IF NOT DEFINED, DEFINE THE PRE-PROCESSOR BELOW
#define INTERNALDRIVERS_H  // DEFINE THE PRE-PROCESSOR

#include "Arduino.h"  // INCLUDE ARDUINO LIBRARY
#include <EEPROM.h>   // INCLUDE EEPROM LIBRARY

// STRUCTURE TO 3 PHASE VOLTAGES
struct voltages {
  float Phase1, Phase2, Phase3;
};

// STRUCTURE TO 3 PHASE CURRENTS
struct currents {
  float Phase1, Phase2, Phase3, Total_I;
};

// STRUCTURE TO 3 PHASE INSTANTENEOUS POWER
struct powers {
  float Phase1, Phase2, Phase3;
};

// STRUCTURE TO 3 PHASE POWER IN KWH
struct kwhPower {
  double Phase1, Phase2, Phase3;
};

// CLASS DEFINATIONS
class internalDrivers {
public:
  internalDrivers();
  void gpioInit();
  void i2cInit();
  void spiInit();
  float readVoltage(const char pin, float calPar1, float calPar2);
  unsigned int internalDrivers::getMax(const char readingPin);
  float readCurrent(const byte i2cAddr, float calPar1, float calPar2);
  powers calculatePower(float voltageOne, float voltageTwo, float voltageThree, float currentOne, float currentTwo, float currentThree);
  kwhPower calculatePowerInKWH(kwhPower powersInKwh, float powerOne, float powerTwo, float powerThree);
  voltages averageVol(voltages vol, int averageCount);
  currents averageCur(currents cur, int averageCount);
  bool lteBeginWithSSL(String mqttServer, int mqttPort, String mqttUserName, String mqttPassword, String deviceId, String subTopic);
  bool lteBeginWithTCP(String mqttServer, int mqttPort, String mqttUserName, String mqttPassword, String deviceId, String subTopic);
  bool publishMsgOverLTENetwork(String pubTopic, String devID, float phase1Vol, float phase2Vol, float phase3Vol, float phase1Cur, float phase2Cur, float phase3Cur, double phase1Pow, double phase2Pow, double phase3Pow, bool loadstate);
  void subscribeMsgOverLTENetwork();
  bool confirmMsgSentOrNotOverLteNetwork(byte data);
  void sendATCommand(String command);
  void resetLTENetwork();
  void StoreKwhPowerInEEPROM(double PowerOneInKwh, double PowerTwoInKwh, double PowerThreeInKwh);
  kwhPower readDataFromEEPROM();
  void writeByteInEEPROM(int memAddress, byte data);
  byte readByteFromEEPROM(int memAddress);
  void writeFloatInEEPROM(int address, long value);
  long readFloatFromEEPROM(int address);
  void toggleStatusAndMeasurementLED();
};

#endif
