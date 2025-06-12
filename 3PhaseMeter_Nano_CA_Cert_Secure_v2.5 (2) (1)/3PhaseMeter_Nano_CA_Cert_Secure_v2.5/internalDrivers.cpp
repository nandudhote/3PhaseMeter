#include "HardwareSerial.h"
#include "Arduino.h"
#include "internalDrivers.h"
#include "Config.h"
#include <Wire.h>
#include <EEPROM.h>
#include <SPI.h>

// /*FOR DEBUG ONLY*/
#include <SoftwareSerial.h>
const char rxPin = 15;
const char txPin = 14;
SoftwareSerial DEBUGSERIAL(rxPin, txPin);  // RX, TX
// /*DEBUG CODE END*/

internalDrivers::internalDrivers() {
}

void internalDrivers::gpioInit() {
  pinMode(_volDivPinOne, INPUT);
  pinMode(_volDivPinTwo, INPUT);
  pinMode(_volDivPinThree, INPUT);
  pinMode(_lteResetPin, OUTPUT);
  pinMode(_contactorPin, OUTPUT);
  pinMode(_statusLEDPin, OUTPUT);
  pinMode(_measurementLEDPin, OUTPUT);
  // analogReference(INTERNAL1V1);  // This is for to set 1.1v reference voltage for ADC // For arduino mega
  analogReference(INTERNAL);  // This is for to set 1.1v reference voltage for ADC // for arduino nano
  digitalWrite(_lteResetPin, LOW);
  DEBUGSERIAL.begin(9600);  // FOR DEBUGGING
  DEBUGSERIAL.println("Debugging Started");
}

void internalDrivers::i2cInit() {
  Wire.begin();
}

void internalDrivers::spiInit() {
  pinMode(MISO, OUTPUT);  // Configure MISO as output
  SPCR |= _BV(SPE);       //Turn on SPI in Slave Mode
  SPI.attachInterrupt();  // Enable SPI interrupt
}

// ISR for SPI communication
ISR(SPI_STC_vect) {
  mqttRecMsg = SPDR;  // Read the received data from the SPI Data Register
  DEBUGSERIAL.print("mqttRecMsg: ");
  DEBUGSERIAL.println(mqttRecMsg);
  /*Trigger contactor according to receive msg*/
  if (mqttRecMsg == 101) {
    digitalWrite(_contactorPin, HIGH);
    loadState = 1;
    gotNewLoadStatus = true;
  } else if (mqttRecMsg == 100) {
    digitalWrite(_contactorPin, LOW);
    loadState = 0;
    gotNewLoadStatus = true;
  } else if (mqttRecMsg == 200) {
    requestOn200 = true;
  } else {
    ;
  }
}

float internalDrivers::readVoltage(const char pin, float calPar1, float calPar2) {
  float calibratedvoltage, avgVolt = 0.0;
  for (byte i = 0; i < 50; i++) {
    uint32_t voltageReading = getMax(pin);
    voltageReading = voltageReading * 1100 / 1023;
    voltageReading /= sqrt(2);
    avgVolt += voltageReading;
  }
  calibratedvoltage = calPar1 * (avgVolt / 50.0) - calPar2;
  calibratedvoltage = (calibratedvoltage < 0.0) ? 0.0 : calibratedvoltage;  // return voltage zero if its less than zero
  return calibratedvoltage;
}

unsigned int internalDrivers::getMax(const char readingPin) {
  uint16_t max_v = 0, maxReading = 0;
  for (byte reading = 0; reading < 5; reading++) {
    for (uint8_t i = 0; i < 50; i++) {
      uint16_t r = analogRead(readingPin);  // read from analog channel 3 (A3)
      if (max_v < r) max_v = r;
      delayMicroseconds(200);
    }
    if (max_v > maxReading) maxReading = max_v;
  }
  return maxReading;
}

float internalDrivers::readCurrent(const byte i2cAddr, float calPar1, float calPar2) {
  // 0x0A // 0x10 // 0x52
  float Current = 0.0;
  unsigned int duration = 2000;

  Wire.requestFrom((uint8_t)i2cAddr, (uint8_t)2, (uint8_t) false);  // request 6 bytes from slave device #8
  startMillis = millis();
  while (millis() - startMillis < duration) {
    if (Wire.available()) {
      Current = Merger(Wire.read(), Wire.read(), calPar1, calPar2);
      return Current;
    }
  }
  return Current;
}

float internalDrivers::Merger(float high, float low, float calpar1, float calpar2) {
  float mergedCur = (high * 100) + low;
  float calibratedCur = (calpar1 * mergedCur) - calpar2;
  return (calibratedCur < 0.0) ? 0.0 : calibratedCur;
}

powers internalDrivers::calculatePower(float voltageOne, float voltageTwo, float voltageThree, float currentOne, float currentTwo, float currentThree) {
  powers power;
  power.Phase1 = voltageOne * currentOne;
  power.Phase2 = voltageTwo * currentTwo;
  power.Phase3 = voltageThree * currentThree;
  return power;
}

kwhPower internalDrivers::calculatePowerInKWH(kwhPower powersInKwh, float powerOne, float powerTwo, float powerThree) {
  // kwhPower powersInKwh;
  powersInKwh.Phase1 += (powerOne / 60.0) / 1000.0;  // taking one sample in every 60 seconds
  powersInKwh.Phase2 += (powerTwo / 60.0) / 1000.0;
  powersInKwh.Phase3 += (powerThree / 60.0) / 1000.0;
  return powersInKwh;
}

voltages internalDrivers::averageVol(voltages vol, int averageCount) {
  vol.Phase1 /= averageCount;
  vol.Phase2 /= averageCount;
  vol.Phase3 /= averageCount;
  DEBUGSERIAL.print("averageCount: ");
  DEBUGSERIAL.println(averageCount);
  return vol;
}

currents internalDrivers::averageCur(currents cur, int averageCount) {
  cur.Phase1 /= averageCount;
  cur.Phase2 /= averageCount;
  cur.Phase3 /= averageCount;
  return cur;
}

bool internalDrivers::lteBegin(String mqttServer, int mqttPort, String mqttUserName, String mqttPassword, String deviceId, String subTopic) {
  sendATCommand("ATE0");                                         // Disable echos , this is mandetory to call before init the MQTT config
  sendATCommand("AT+CSSLCFG=0");                                 // set ssl context ID to zero
  sendATCommand("AT+CCERTLIST");                                 // check downloaded certificates lists # This is not mandetory, just for debugging in case of any issue
  sendATCommand("AT+CSSLCFG=\"sslversion\",0,4");                // select all ssl versions
  sendATCommand("AT+CSSLCFG=\"authmode\",0,2");                  // enable server and client certification verification
  sendATCommand("AT+CSSLCFG=\"cacert\",0,\"a.pem\"");            // Attach rootCA cerificate # certificate already downloaded in module
  sendATCommand("AT+CSSLCFG=\"clientcert\",0,\"b.pem\"");        // Attach client cerificate # certificate already downloaded in module
  sendATCommand("AT+CSSLCFG=\"clientkey\",0,\"c.pem\"");         // Attach key cerificate # certificate already downloaded in module
  sendATCommand("AT+CSSLCFG=0");                                 // check whether all parameters set properly or not
  sendATCommand("AT+CMQTTSTART");                                // Start Mqtt
  sendATCommand("AT+CMQTTACCQ=0,\"+ String(deviceId) + \", 1");  // set client name
  // sendATCommand("AT+CMQTTUSERCFG=0,1,\"" + deviceId + "\",\"" + mqttUserName + "\",\"" + mqttPassword + "\",0,0,\"\"");
  sendATCommand("AT+CMQTTCONNECT=0,\"tcp://" + mqttServer + ":" + mqttPort + "\",90,1");  // Connect to broker
  if (!confirmMsgSentOrNotOverLteNetwork(1)) return false;                                // confirm whether device connected to broker properly or not
  sendATCommand("AT+CMQTTSUBTOPIC=0," + String(subTopic.length()) + ",1");                // Set sub topic
  sendATCommand(subTopic);                                                                // send sub topic
  sendATCommand("AT+CMQTTSUB=0," + String(subTopic.length()) + ",1");                     // Set sub topic
  sendATCommand(subTopic);                                                                // send sub topic
  if (!confirmMsgSentOrNotOverLteNetwork(2)) return false;                                // confirm wether module subscribed topic properly or not
  return true;
}

bool internalDrivers::publishMsgOverLTENetwork(String pubTopic, String devID, float phase1Vol, float phase2Vol, float phase3Vol, float phase1Cur, float phase2Cur, float phase3Cur, double phase1Pow, double phase2Pow, double phase3Pow, bool loadstate) {
  String transmittingString = devID + ":" + String(phase1Vol) + ":" + String(phase2Vol) + ":" + String(phase3Vol) + ":" + String(phase1Cur) + ":" + String(phase2Cur) + ":" + String(phase3Cur) + ":" + String(phase1Pow) + ":" + String(phase2Pow) + ":" + String(phase3Pow) + ":" + String(loadstate);
  sendATCommand("AT+CMQTTTOPIC=0," + String(pubTopic.length()));              //AT Command for Setting up the publishTopic Topic Name
  sendATCommand(pubTopic);                                                    //Topic Name
  sendATCommand("AT+CMQTTPAYLOAD=0," + String(transmittingString.length()));  //Payload length
  sendATCommand(transmittingString);                                          //Payload message
  sendATCommand("AT+CMQTTPUB=0,1,60");                                        //Acknowledgment
  bool responseStatus = confirmMsgSentOrNotOverLteNetwork(3);                 // check whether msg sent properly not
  return responseStatus;
}

bool internalDrivers::confirmMsgSentOrNotOverLteNetwork(byte data) {
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {  // Check for 3 seconds
    if (mqttRecMsg == data) {
      return true;  // Return true immediately if data matches
    }
  }
  return false;  // Return false if no match after 3 seconds
}

void internalDrivers::sendATCommand(String command) {
  DEBUGSERIAL.print("Sending: ");
  DEBUGSERIAL.println(command);

  Serial.println(command.c_str());  // Send command to the module
  delay(1000);                      // Small delay for response handling

  // while (Serial.available()) {
  //   String response = Serial.readString();  // Read module response
  //   DEBUGSERIAL.print("Response: ");
  //   DEBUGSERIAL.println(response);
  // }
}

void internalDrivers::resetLTENetwork() {
  Serial.println("AT+CMQTTDISC=0,60");  // disconnect the Mqtt
  Serial.println("AT+CRESET");          // reset the module
  lteRestarted = true;
  // digitalWrite(_lteResetPin, HIGH);
  // delay(100);
  // digitalWrite(_lteResetPin, LOW);
  // delay(100);
}

void internalDrivers::StoreKwhPowerInEEPROM(double PowerOneInKwh, double PowerTwoInKwh, double PowerThreeInKwh) {
  writeFloatInEEPROM(powerOneInKwhEEPROMAdd, long(PowerOneInKwh * 100));
  writeFloatInEEPROM(powerTwoInKwhEEPROMAdd, long(PowerTwoInKwh * 100));
  writeFloatInEEPROM(powerThreeInKwhEEPROMAdd, long(PowerThreeInKwh * 100));
}

kwhPower internalDrivers::readDataFromEEPROM() {
  kwhPower kwhPowers;
  kwhPowers.Phase1 = (readFloatFromEEPROM(powerOneInKwhEEPROMAdd) / 100.0);
  kwhPowers.Phase2 = (readFloatFromEEPROM(powerTwoInKwhEEPROMAdd) / 100.0);
  kwhPowers.Phase3 = (readFloatFromEEPROM(powerThreeInKwhEEPROMAdd) / 100.0);
  loadState = readByteFromEEPROM(loadStateEEPROMAdd);
  // loadState = (loadState >= 1) ? true : false;
  digitalWrite(_contactorPin, loadState);
  return kwhPowers;
}

void internalDrivers::writeByteInEEPROM(int memAddress, byte data) {
  Wire.beginTransmission(eepromI2cAddr);
  Wire.write((memAddress >> 8) & 0xFF);  // High byte of memory address
  Wire.write(memAddress & 0xFF);         // Low byte of memory address
  Wire.write(data);                      // Data to write
  Wire.endTransmission();
  delay(10);  // Writing takes time
}

byte internalDrivers::readByteFromEEPROM(int memAddress) {
  byte data = 0;
  Wire.beginTransmission(eepromI2cAddr);
  Wire.write((memAddress >> 8) & 0xFF);  // High byte of memory address
  Wire.write(memAddress & 0xFF);         // Low byte of memory address
  Wire.endTransmission();

  Wire.requestFrom(int(eepromI2cAddr), int(1));  // Request 1 byte from EEPROM
  if (Wire.available()) {
    data = Wire.read();
  }
  return data;
}

/*FUNCTION TO WRITE 4-BYTE FLOAT TO EEPROM*/
void internalDrivers::writeFloatInEEPROM(int address, long value) {
  Wire.beginTransmission(eepromI2cAddr);
  Wire.write((address >> 8) & 0xFF);  // High byte of address
  Wire.write(address & 0xFF);         // Low byte of address
  Wire.write((value >> 24) & 0xFF);   // Byte 3 (most significant byte)
  Wire.write((value >> 16) & 0xFF);   // Byte 2
  Wire.write((value >> 8) & 0xFF);    // Byte 1
  Wire.write(value & 0xFF);           // Byte 0 (least significant byte)
  Wire.endTransmission();
  delay(10);  // Increased delay to allow enough time for EEPROM write
}

/*FUNCTION TO READ 4-BYTE FLOAT FROM EEPROM*/
long internalDrivers::readFloatFromEEPROM(int address) {
  long value = 0;

  Wire.beginTransmission(eepromI2cAddr);
  Wire.write((address >> 8) & 0xFF);  // High byte of address
  Wire.write(address & 0xFF);         // Low byte of address
  Wire.endTransmission();

  Wire.requestFrom(int(eepromI2cAddr), int(4));  // Request 4 bytes for the long value
  if (Wire.available() == 4) {
    value |= (long)Wire.read() << 24;  // Byte 3
    value |= (long)Wire.read() << 16;  // Byte 2
    value |= (long)Wire.read() << 8;   // Byte 1
    value |= (long)Wire.read();        // Byte 0
  }
  return value;
}
