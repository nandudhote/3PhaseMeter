#include "HardwareSerial.h"
#include "Arduino.h"
#include "internalDrivers.h"
#include "Config.h"
#include <Wire.h>
#include <EEPROM.h>
#include <SPI.h>

// CONSTRUCTOR OF CLASS // CONSTRUNCTOR USE TO SET REQUIRED MEMORY FOR CLASS OBJECTS // ITS DO REQUIRED INITILISATION FOR CLASS DEFINATION
internalDrivers::internalDrivers() {
}

/*
  FUNCTION NAME: GPIOINIT
  FUNCTION INPUT: NONE
  FUNCTION RETURNS: NONE
  FUNCTION DESCRIPTION: FUNCTION USED TO SET MODE OF GPIO AS INPUT OR OUTPUT
*/
void internalDrivers::gpioInit() {
  pinMode(_volDivPinOne, INPUT);        // PHASE 1 VOLTAGE READ PIN AS INPUT
  pinMode(_volDivPinTwo, INPUT);        // PHASE 2 VOLTAGE READ PIN AS INPUT
  pinMode(_volDivPinThree, INPUT);      // PHASE 3 VOLTAGE READ PIN AS INPUT
  pinMode(_lteResetPin, OUTPUT);        // LTE MUDULE RESET RELAY CONTROL PIN SET AS OUTPUT
  pinMode(_contactorPin, OUTPUT);       // CONTACTOR CONTROL RELAY PIN SET AS OUTPUT
  pinMode(_statusLEDPin, OUTPUT);       // STATUS LED PIN SET AS OUTPUT
  pinMode(_measurementLEDPin, OUTPUT);  // MEASUREMENT LED PIN SET AS OUTPUT
  pinMode(MISO, OUTPUT);                // CONFIGURE SPI MISO PIN AS OUTPUT // MISO VARIABLE NAME IN PINS_ARDUINO.H FILE NOT IN CONFIG.H
  // analogReference(INTERNAL1V1);         // This is for to set 1.1v reference voltage for ADC // For arduino mega
  analogReference(INTERNAL);        // SET 1.1V REF VOLTAGE FOR ADC TO GET BETTER ADC RESOLUTION
  digitalWrite(_lteResetPin, LOW);  // TURN ON THE LTE MODULE RELAY BY DEFAULT // WE WILL NOT USE HARDWARE RESET // AS SOFTWARE RESET BETTER IN CASE OF ANY COMMUNICATION FAIL
}

/*
  FUNCTION NAME: I2CINIT
  FUNCTION INPUT: NONE 
  FUNCTION RETURNS: NONE
  FUNCTION DESCRIPTION: CALL WIRE.H LIBRARY BEGIN() FUNCTION TO INITILISE THE I2C // WHY NOT CALL THIS ONE LINE IN .INO FILE SETUP FUNCTION? FOR BETTER CODE REDIABILITY 
*/
void internalDrivers::i2cInit() {
  Wire.begin();  // SET I2C IN MASTER MODE
}

/*
  FUNCTION NAME: SPIINIT
  FUNCTION INPUT: NONE
  FUNCTION RETURNS: NONE
  FUNCTION DESCRIPTION: INITILISE THE SPI SLAVE MODE AND ENABLE SPI INTTERUPT
*/
void internalDrivers::spiInit() {
  SPCR |= _BV(SPE);       // ACTIVATE SPI SLAVE MODE
  SPI.attachInterrupt();  // ENABLE SPI INTTERUPT
}

/*
  FUNCTIONS NAME: ISR
  FUNCTION INPUT: INTERRUPT VECTOR TABLE // SPI SERIAL TRANSFER COMPLETE
  FUNCTION RETURNS: NONE
  FUNCTION DESCRIPTION: WHENEVER DATA RECEIVE OVER SPI BUS, MCU JUMP ON THIS FUNCTION, THEN READ THE DATA AND PERFORM THE ACTION ACCORDINGLY
*/
ISR(SPI_STC_vect) {
  mqttRecMsg = SPDR;                    // LOAD DATA FROM SPI DATA REGISTER TO MQTTRECMSG VARIABLE
  if (mqttRecMsg == 101) {              // IF RECEIVED DATA IS 101 THEN TURN ON THE CONTACTOR
    digitalWrite(_contactorPin, HIGH);  // TURN ON CONTACTOR RELAY
    loadState = 1;                      // SET LOADSTATE FLAG TO 1 // THIS FLAG DATA SEND OVER MQTT
    gotNewLoadStatus = true;            // SET THIS FLAG TO TRUE FOR STORE DATA IN EEPROM // CALLED THIN IN .INO FILE // NOT CALLING EEPROM WRITE FUNCTION HERE TO AVOID INTERRUPT LATTENCY
  } else if (mqttRecMsg == 100) {       // IF RECEIVED DATA IS 100 THEN TURN OFF THE CONSTACTOR
    digitalWrite(_contactorPin, LOW);   // TURN OFF CONTACTOR RELAY
    loadState = 0;                      // SET LOADSTATE FLAG TO 0 // THIS FLAG DATA SEND OVER MQTT
    gotNewLoadStatus = true;            // SET THIS FLAG TO TRUE FOR STORE DATA IN EEPROM // CALLED THIN IN .INO FILE // NOT CALLING EEPROM WRITE FUNCTION HERE TO AVOID INTERRUPT LATTENCY
  } else if (mqttRecMsg == 200) {       // IF RECIEVED DATA IS 200 THEN ENABLE REQUESTON200 FLAG
    requestOn200 = true;                // ENABLE THIS FLAG TO SEND DATA OVER MQTT AS REQUIRED BY USER
  } else {                              // DO NOTHING IF ANY OTHER DATA RECEIVED
    ;
  }
}

/*
  FUNCTION NAME: READVOLTAGE
  FUNCTION INPUT: ADC PIN, CALIBRATION VALUES
  FUNCTION RETURN: CALIBRATED VOLTAGE
  FUNCTION DESCRIPTION: READ 50 CYCLES OF ADC VALUE AND APPLY FORMULE FOR CALIBRATION TO GET CALIBRATED VOLATGE // AS PER OUR ANALYSIS ALWAYS GETS VALUE IN LINEAR PATTERN
*/
float internalDrivers::readVoltage(const char pin, float calPar1, float calPar2) {
  float avgVolt = 0.0;                                                      // VARIABLE INITILISE WITH ZERO VALUE TO STORE 50 CYCLES TOTAL VALUE
  for (byte i = 0; i < 50; i++) {                                           // TABLE SAMPLES 50 TIMES
    avgVolt += getMax(pin);                                                 // GET MAXIMUM VALUE FROM INPUT BECAUSE ITS AC SIGNAL AND WANT TO MEASURE THE HIGHEST AMPLITUDE
  }                                                                         //
  float calibratedvoltage = calPar1 * (avgVolt / 50.0) - calPar2;           // APPLY CALIBRATED FARMULA
  calibratedvoltage = (calibratedvoltage < 0.0) ? 0.0 : calibratedvoltage;  // RETURN ZERO IF CALCULATED VALUE IS NEGA
  return calibratedvoltage;                                                 // RETURN THE CALIBRATED VALUE
}

/*
  FUNCTION NAME: GETMAX
  FUNCTION INPUT: ADC PIN
  FUNCTION RETURN: MAXIMUM VALUE
  FUNCTION DESCRIPTION: CONFIRM THE MAX VALUE FROM 250 ADC SAMPLES TO FIND CORRECT MAX VOLTAGE AMPLITUDE
*/
unsigned int internalDrivers::getMax(const char readingPin) {
  uint16_t max_v = 0, maxReading = 0;
  for (byte reading = 0; reading < 5; reading++) {  // TAKE TOP MAX 5 VALUE SAMPLES AND FIND TOP MAX ONE
    for (uint8_t i = 0; i < 50; i++) {              // FIND MAX VALUE FROM 50 ADC VALUE SAMPLES
      uint16_t r = analogRead(readingPin);          // READ ADC PIN
      if (max_v < r) max_v = r;                     // STORE MAX VALUE IN MAX_V VARIABLE
      delayMicroseconds(200);                       // 200US DELAY BETWEEN TWO SAMPLES
    }                                               //
    if (max_v > maxReading) maxReading = max_v;     // FIND MAX ADC VALUE FROM TOP 5 MAX VALUE
  }                                                 //
  return maxReading;                                // REURN MAX VALUE
}

/*
  FUNCTION NAME: READCURRENT
  FUNCTION INPUT: I2C ADDRESS, CALIBRATED VALUE
  FUNCTION RETURN: CALIBRATED CURRENT
  FUNCTION DESCRIPTION: SEND REQUEST ON I2C TO N76E003 MCU WHICH FIND THE MAX ADC VALUE AND THEN APPLY CALIBRATION FARMULA TO FIND CALIBRATED CURRENT
*/
float internalDrivers::readCurrent(const byte i2cAddr, float calPar1, float calPar2) {
  // 0x0A // 0x10 // 0x52
  float Current = 0.0;
  unsigned int duration = 2000;                                       // 2 SECONDS TIMEOUT DURATION TO GET RESPONSE FROM N76E003 MCU
  Wire.requestFrom((uint8_t)i2cAddr, (uint8_t)2, (uint8_t) false);    //REQUEST TWO BYTE CT ADC DATA FROM N76E003 MCU
  startMillis = millis();                                             // START TIMER
  while (millis() - startMillis < duration) {                         // WAIT TILL 2 SECONDS TO GET DATA, IF DATA RECEIVED BEFORE THIS THEN BREAK THE LOOP
    if (Wire.available()) {                                           // CHECK WHETHER DATA AVAILABLE ON I2C BUFFER
      unsigned int mergedCurAdc = (Wire.read() * 100) + Wire.read();  // MEARGE THE RECEIVED TWO BYTE DATA, N76E003 MCU BREAK THE 4 BYTE DATA IN 2 - 2 BYTE. EXA: CALCULATED ADC VALUE FROM CT PIN IS 1623, THEN RECEIVED VALUE WILL 16 AND 23
      float calibratedCur = (calPar1 * mergedCurAdc) - calPar2;       // CALIBRATE THE CURRENT
      return (calibratedCur < 0.0) ? 0.0 : calibratedCur;             // REUTN THE CALIBRATED CURRENT
    }                                                                 //
  }                                                                   //
  return Current;                                                     // RETURN 0 CURRENT IN CASE OF READING FAILED (SUGGESTION FOR BETTER: INSTEAD OF PASSING ZERO RETURN PREVIOUS CORRECT VALUE)
}

/*
  FUNCTION NAME: CALCULATEPOWER
  FUNCTION INPUT: 3 PHASES VOLTAGE AND CURRENT
  FUNCTION RETURN: POWER
  FUNCTION DESCRIPTION: CALCULATE THE POWER OF ALL THREE PHASES BY MULTIPLYING VOLTAGE AND CURRENT
*/
powers internalDrivers::calculatePower(float voltageOne, float voltageTwo, float voltageThree, float currentOne, float currentTwo, float currentThree) {
  powers power;                                // CREATE OBJECT FOR POWER FUNCTION
  power.Phase1 = voltageOne * currentOne;      // CALCULATE PHASE 1 POWER
  power.Phase2 = voltageTwo * currentTwo;      // CALCULATE PHASE 2 POWER
  power.Phase3 = voltageThree * currentThree;  // CALCULATE PHASE 3 POWER
  return power;                                // RETURN CALCULATE POWER STRUCTURE // USING STRUCTURE TO RETURN MULTIPLE VALUES
}

/*
  FUNCTION NAME: CALCULATEPOWERINKWH 
  FUNCTION INPUT: KWHPOWER STRUCTURE, POWER OF PHASE 1, PHASE 2, PHASE 3
  FUNCTION RETURN: LWH CALCULATE POWER
  FUNCTION DESCRIPTION: CALCULATE THE POWER IN KWH
*/
kwhPower internalDrivers::calculatePowerInKWH(kwhPower powersInKwh, float powerOne, float powerTwo, float powerThree) {
  powersInKwh.Phase1 += (powerOne / 60.0) / 1000.0;    // CALCULATE IN EVERY 60 SECONDS THATS WHY DEVIDING BY 60 AND FOR CONVERTING IN KW DEVIDE BY 1000
  powersInKwh.Phase2 += (powerTwo / 60.0) / 1000.0;    // CALCULATE IN EVERY 60 SECONDS THATS WHY DEVIDING BY 60 AND FOR CONVERTING IN KW DEVIDE BY 1000
  powersInKwh.Phase3 += (powerThree / 60.0) / 1000.0;  // CALCULATE IN EVERY 60 SECONDS THATS WHY DEVIDING BY 60 AND FOR CONVERTING IN KW DEVIDE BY 1000
  return powersInKwh;                                  // RETURN CALCULATE POWER IN KWH
}

/*
  FUNCTION NAME: AVERAGEVOL
  FUNCTION INPUT: VOLTAGE STRUCTURE, AVERAGE SAMPLE COUNT
  FUNCTION RETURN: AVERAGE VOLTAGE
  FUNCTION DESCRIPTION: CALCULATE THE VOLTAGE FROM TOTAL ADDED VOLTAGE BY DIVIDING NUMBER OF SAMPLES 
*/
voltages internalDrivers::averageVol(voltages vol, int averageCount) {
  vol.Phase1 /= averageCount;  // DIVIDE TOTAL VOLTAGE BY NUMBER OF TIMES VOLTAGE ADDED
  vol.Phase2 /= averageCount;  // DIVIDE TOTAL VOLTAGE BY NUMBER OF TIMES VOLTAGE ADDED
  vol.Phase3 /= averageCount;  // DIVIDE TOTAL VOLTAGE BY NUMBER OF TIMES VOLTAGE ADDED
  return vol;                  // RETURN VOLTAGE STRUCTURE
}

/*
  FUNCTION NAME: AVERAGECUR
  FUNCTION INPUT: CURRENT STRUCTURE, AVERAGE SAMPLE COUNT
  FUNCTION RETURN: AVERAGE CURRENT
  FUCTION DESCRIPTION: CALCULATE THE CURRENT FROM TOTAL ADDED VOLTAGE BY DIVIDING NUMBER OF SAMPLES 
*/
currents internalDrivers::averageCur(currents cur, int averageCount) {
  cur.Phase1 /= averageCount;
  cur.Phase2 /= averageCount;
  cur.Phase3 /= averageCount;
  return cur;
}

/*
  FUNCTION NAME: LTEBEGINWITHSSL
  FUNCTION INPUT: MQTT BROKER, MQTT PORT, BROKER USERNAME AND PASSWORD, DEVICE ID, MQTT SUBSCRIBE TOPIC
  FUNCTION RETURN: WHETHER MESSAGE HAS SENT PROPERLY OR NOT
  FUCTION DESCRIPTION: LTE MODULE ESTABLISH THE CONNECTION WITH MQTT MODULE WITH CA CERTIFICATES AND RETURN THE STATUS OF WHETHER MESSAGE HAS SENT SUCCESSFULLY OR NOT
                      PRE-CONSIDERATION: THE CA CERTIFICATE, CLIENT CERTIFICATE AND KEY CERTIFICATES ARE DOWNLOADED IN MODULE
                      TO UNDERSTAND THE BELOW CONNANDS PLEASE READ AT COMMANDS MANUAL OF SIMCOMA7670 LTE MODULE - https://www.ktron.in/wp-content/uploads/2023/03/A76XX-Series_AT_Command_Manual_V1.06-4.pdf
*/
bool internalDrivers::lteBeginWithSSL(String mqttServer, int mqttPort, String mqttUserName, String mqttPassword, String deviceId, String subTopic) {
  sendATCommand("ATE0");                                                                  // DISABLE ECHOS , THIS IS MANDATORY TO CALL BEFORE INIT THE MQTT CONFIG
  sendATCommand("AT+CSSLCFG=0");                                                          // SET SSL CONTEXT ID TO ZERO // YOU CAN SET 0 OR 1 ANY // WE SELECTED DEFAULT ONE
  sendATCommand("AT+CCERTLIST");                                                          // CHECK DOWNLOADED CERTIFICATES LISTS # THIS IS NOT MANDATORY, JUST FOR DEBUGGING IN CASE OF ANY ISSUE
  sendATCommand("AT+CSSLCFG=\"sslversion\",0,4");                                         // SELECT ALL SSL VERSIONS // SELECTED THE ALL OPTION
  sendATCommand("AT+CSSLCFG=\"authmode\",0,2");                                           // ENABLE SERVER AND CLIENT CERTIFICATION VERIFICATION // FOR SUCCESSFULL CONNECTION MUST HAVE SERVER AND CLIENT VERIFICATION CERTIFICATIONS // SOMETIME WE CAN HAVE ONLY CLIENT VERIFICATION CERTIFICATE, THAT WILL NOT WORK
  sendATCommand("AT+CSSLCFG=\"cacert\",0,\"a.pem\"");                                     // ATTACH ROOT CA CERTIFICATE // CERTIFICATE ALREADY DOWNLOADED IN MODULE // FILE DOWNLOADED IN MODULE WITH NAME OF a.pem
  sendATCommand("AT+CSSLCFG=\"clientcert\",0,\"b.pem\"");                                 // ATTACH CLIENT CERTIFICATE // CERTIFICATE ALREADY DOWNLOADED IN MODULE // FILE DOWNLOADED IN MODULE WITH NAME OF b.pem
  sendATCommand("AT+CSSLCFG=\"clientkey\",0,\"c.pem\"");                                  // ATTACH KEY CERTIFICATE // CERTIFICATE ALREADY DOWNLOADED IN MODULE // FILE DOWNLOADED IN MODULE WITH NAME OF c.pem
  sendATCommand("AT+CSSLCFG=0");                                                          // CHECK WHETHER ALL PARAMETERS SET PROPERLY OR NOT // THIS COMMAND RETURNS THE ABOVE SET PARAMETERS STATUS
  sendATCommand("AT+CMQTTSTART");                                                         // START MQTT
  sendATCommand("AT+CMQTTACCQ=0,\"+ String(deviceId) + \", 1");                           // SET CLIENT NAME // 1 FOR SSL CONNECTION // 0 FOR TCP CONNECTIONS
  sendATCommand("AT+CMQTTCONNECT=0,\"tcp://" + mqttServer + ":" + mqttPort + "\",90,1");  // CONNECT TO BROKER // 90 SECONDS ARE CONNECTION TIMEOUT // 1 FOR SSL CONNECTION // 0 FOR TCP CONNECTIONS
  if (!confirmMsgSentOrNotOverLteNetwork(1)) return false;                                // CONFIRM WHETHER DEVICE CONNECTED TO BROKER PROPERLY OR NOT // AFTER SUCCESSFULL CONNECTION MODULE WILL RETURN CMQTTCONNECT=0,0
  sendATCommand("AT+CMQTTSUBTOPIC=0," + String(subTopic.length()) + ",1");                // SET SUBSCRIBE TOPIC
  sendATCommand(subTopic);                                                                // SEND SUBSCRIBE TOPIC
  sendATCommand("AT+CMQTTSUB=0," + String(subTopic.length()) + ",1");                     // SET SUBSCRIBE TOPIC
  sendATCommand(subTopic);                                                                // SEND SUBSCRIBE TOPIC
  if (!confirmMsgSentOrNotOverLteNetwork(2)) return false;                                // CONFIRM WETHER MODULE SUBSCRIBED TOPIC PROPERLY OR NOT // RETURN FALSE IF NOT SUBSCRIBED SUCCESSFULLY
  return true;                                                                            // RETUTN TRUE IF ALL PROCESS ARE DONE PROPERLY
}

/*
  FUNCTION NAME: LTEBEGINWITHTCP
  FUNCTION INPUT: MQTT BROKER, MQTT PORT, BROKER USERNAME AND PASSWORD, DEVICE ID, MQTT SUBSCRIBE TOPIC
  FUNCTION RETURN:  WHETHER MESSAGE HAS SENT PROPERLY OR NOT
  FUCTION DESCRIPTION: LTE MODULE ESTABLISH THE CONNECTION WITH MQTT MODULE WITHOUT CA CERTIFICATES AND RETURN THE STATUS OF WHETHER MESSAGE HAS SENT SUCCESSFULLY OR NOT. DO NOT NEED TO DOWNLOAD CERTIFICATES IN MODULE
                      TO UNDERSTAND THE BELOW CONNANDS PLEASE READ AT COMMANDS MANUAL OF SIMCOMA7670 LTE MODULE - https://www.ktron.in/wp-content/uploads/2023/03/A76XX-Series_AT_Command_Manual_V1.06-4.pdf
*/
bool internalDrivers::lteBeginWithTCP(String mqttServer, int mqttPort, String mqttUserName, String mqttPassword, String deviceId, String subTopic) {
  sendATCommand("ATE0");                                         // DISABLE ECHOS , THIS IS MANDATORY TO CALL BEFORE INIT THE MQTT CONFIG
  sendATCommand("AT+CMQTTSTART");                                // START MQTT
  sendATCommand("AT+CMQTTACCQ=0,\"+ String(deviceId) + \", 0");  // SET CLIENT NAME // 1 FOR SSL CONNECTION // 0 FOR TCP CONNECTIONS
  // sendATCommand("AT+CMQTTUSERCFG=0,1,\"" + deviceId + "\",\"" + mqttUserName + "\",\"" + mqttPassword + "\",0,0,\"\"");
  sendATCommand("AT+CMQTTCONNECT=0,\"tcp://" + mqttServer + ":" + mqttPort + "\",90,1");  // CONNECT TO BROKER // 90 SECONDS ARE CONNECTION TIMEOUT // 1 FOR SSL CONNECTION // 0 FOR TCP CONNECTIONS
  if (!confirmMsgSentOrNotOverLteNetwork(1)) return false;                                // CONFIRM WHETHER DEVICE CONNECTED TO BROKER PROPERLY OR NOT // AFTER SUCCESSFULL CONNECTION MODULE WILL RETURN CMQTTCONNECT=0,0
  sendATCommand("AT+CMQTTSUBTOPIC=0," + String(subTopic.length()) + ",1");                // SET SUBSCRIBE TOPIC
  sendATCommand(subTopic);                                                                // SEND SUBSCRIBE TOPIC
  sendATCommand("AT+CMQTTSUB=0," + String(subTopic.length()) + ",1");                     // SET SUBSCRIBE TOPIC
  sendATCommand(subTopic);                                                                // SEND SUBSCRIBE TOPIC
  if (!confirmMsgSentOrNotOverLteNetwork(2)) return false;                                // CONFIRM WETHER MODULE SUBSCRIBED TOPIC PROPERLY OR NOT // RETURN FALSE IF NOT SUBSCRIBED SUCCESSFULLY
  return true;                                                                            // RETUTN TRUE IF ALL PROCESS ARE DONE PROPERLY
}

/*
  FUNCTION NAME: PUBLISHMSGOVERLTENETWORK
  FUNCTION INPUT: MQTT PUBLISH TOPIC, DEVICE ID, PHASE1 VOLTAGE, PHASE2 VOLTAGE, PHASE3 VOLTAGE, PHASE1 CURRENT, PHASE2 CURRENT, PHASE3 CURRENT, PHASE1 KWH POWER, PHASE2 KWH POWER, PHASE3 KWH POWER, LOAD STATUS
  FUNCTION RETURN: WHETHER MESSAGE SENT SUCCESSFULLY OR NOT // TRUE = SUCCESSFULLY SENT // FALSE = FAILED TO SEND
  FUCTION DESCRIPTION: SEND MEASURED 3 PHASE VOLTAGE, CURRENT, KWH POWER, LOAD STATUS OVER MQTT 
*/
bool internalDrivers::publishMsgOverLTENetwork(String pubTopic, String devID, float phase1Vol, float phase2Vol, float phase3Vol, float phase1Cur, float phase2Cur, float phase3Cur, double phase1Pow, double phase2Pow, double phase3Pow, bool loadstate) {
  String transmittingString = devID + ":" + String(phase1Vol) + ":" + String(phase2Vol) + ":" + String(phase3Vol) + ":" + String(phase1Cur) + ":"
                              + String(phase2Cur) + ":" + String(phase3Cur) + ":" + String(phase1Pow) + ":"
                              + String(phase2Pow) + ":" + String(phase3Pow) + ":" + String(loadstate);  // MERGE ALL SENDING PARAMETERS IN ONE STRING
  sendATCommand("AT+CMQTTTOPIC=0," + String(pubTopic.length()));                                        // SEND COMMAND TO LOAD PUBLISH TOPIC
  sendATCommand(pubTopic);                                                                              // SEND PUBLISH TOPIC
  sendATCommand("AT+CMQTTPAYLOAD=0," + String(transmittingString.length()));                            // SEND COMMAND TO LOAD MESSAGE
  sendATCommand(transmittingString);                                                                    // PAYLOAD MESSAGE
  sendATCommand("AT+CMQTTPUB=0,1,60");                                                                  // PUBLISH THE MESSAGE
  return confirmMsgSentOrNotOverLteNetwork(3);                                                          // RETURN STATUS WHETHER MESSAGE SENT SUCCESSFULLY OR NOT // TRUE = SUCCESSFULLY SENT // FALSE = FAILED TO SEND
}

/*
  FUNCTION NAME: CONFIRMMSGSENDORNOTOVERLTENETWORK
  FUNCTION INPUT: CONFIRMATION CODE
  FUNCTION RETURN: TRUE = RESPONSE CODE  MATCHED // FALSE = RESPONSE CODE NOT MATCHED
  FUCTION DESCRIPTION: ONCE COMMAND SENT TO LTE MODULE, MODULE RESPONSE OVER THIS COMMANDS, TO READ MESSAGE FROM LTE MODULE USED ANOTHER ARDUINO PRO MINI MCU. PRO MINI MCU DECODE THE RECEIVED MESSAGE FROM LTE MODULE, THE DECODED CODES ARE BELOW,
                      1 - CONNECTED TO MQTT BROKER SUCCESSFULLY
                      2 - SUBSCRIBED MQTT TOPIC SUCCESSFULLY
                      3 - MESSGE SENT OVER MQTT NETWORK SUCCESSFULLY
                      CONNECTIONS: ARDUINO NANO TX - LTE MODULE TX
                                    ARDUINO PRO MINI - LTE MODULE RX
                                    ARDUINO NANO SPI - ARDUINO PRO MINI SPI
*/
bool internalDrivers::confirmMsgSentOrNotOverLteNetwork(byte data) {
  unsigned long startTime = millis();    // START MILLI SECONDS TIMER
  while (millis() - startTime < 3000) {  // BE HERE TILL 3 SECONDS IF DID NOT GET RESPONSE FROM PRO MINI MCU
    if (mqttRecMsg == data) {            // CHECK IF RECEIVED MESSAGE MATCH WITH GIVEN CODE OR NOT
      return true;                       // RETURN TRUE IMMEDIATELY IF DATA MATCHES
    }                                    //
  }                                      //
  return false;                          // RETURN FALSE IF NO MATCH AFTER 3 SECONDS
}

/*
  FUNCTION NAME: SENDATCOMMAND
  FUNCTION INPUT: AT COMMAND OR MESSAGE
  FUNCTION RETURN: NONE
  FUCTION DESCRIPTION: SEND AT COMMAND OR MESSAGE TO LTE COMMAND OVER SERIAL PORT (UART)
*/
void internalDrivers::sendATCommand(String command) {
  Serial.println(command.c_str());  // SEND COMMAND TO MODULE
  delay(1000);                      // SMALL DELAY FOR RESPONSE HANDLING
}

/*
  FUNCTION NAME: RESETLTENETWORK
  FUNCTION INPUT: NONE
  FUNCTION RETURN: NONE
  FUCTION DESCRIPTION: IF DEVICE LOST THE MQTT CONNECTION THEN RESTART THE MODULE BY SOFTWARE COMMANDS
*/
void internalDrivers::resetLTENetwork() {
  sendATCommand("AT+CMQTTDISC=0,60");  // DISCONNECT THE MQTT
  sendATCommand("AT+CRESET");          // DO SOFT RESET
  lteRestarted = true;                 // ENABLE LTERESTARTED FLAG TO REBEGIN THE MODULE
}

/*
  FUNCTION NAME: STOREKWHPOWERINEEPROM
  FUNCTION INPUT: 3 PHASE KWH POWERS 
  FUNCTION RETURN: NONE
  FUCTION DESCRIPTION: WRITE KWH POWER IN EEPROM LOCATIONS
*/
void internalDrivers::StoreKwhPowerInEEPROM(double PowerOneInKwh, double PowerTwoInKwh, double PowerThreeInKwh) {
  writeFloatInEEPROM(powerOneInKwhEEPROMAdd, long(PowerOneInKwh * 100));      // WRITE PHASE 1 POWER IN EEPROM
  writeFloatInEEPROM(powerTwoInKwhEEPROMAdd, long(PowerTwoInKwh * 100));      // WRITE PHASE 2 POWER IN EEPROM
  writeFloatInEEPROM(powerThreeInKwhEEPROMAdd, long(PowerThreeInKwh * 100));  // WRITE PHASE 3 POWER IN EEPROM
}

/*
  FUNCTION NAME: READDATAFROMEEPROM
  FUNCTION INPUT: NONE
  FUNCTION RETURN: KWH POWER STRUCTURE
  FUCTION DESCRIPTION: READ STORE DATA FROM EEPROM AFTER POWER UP THE MCU
*/
kwhPower internalDrivers::readDataFromEEPROM() {
  kwhPower kwhPowers;                                                          // CREATE OBJECT FOR STRUCTURE
  kwhPowers.Phase1 = (readFloatFromEEPROM(powerOneInKwhEEPROMAdd) / 100.0);    // READ PHASE 1 POWER
  kwhPowers.Phase2 = (readFloatFromEEPROM(powerTwoInKwhEEPROMAdd) / 100.0);    // READ PHASE 2 POWER
  kwhPowers.Phase3 = (readFloatFromEEPROM(powerThreeInKwhEEPROMAdd) / 100.0);  // READ PHASE 3 POWER
  loadState = readByteFromEEPROM(loadStateEEPROMAdd);                          // READ LOAD STATUS
  digitalWrite(_contactorPin, loadState);                                      // TRIGGER THE CONTACTOR ACCORDING TO READ LOAD STATUS
  return kwhPowers;                                                            // RETURN KWH READ STRUCTURE
}

/*
  FUNCTION NAME: WRITEBYTEINEEPROM
  FUNCTION INPUT: EEPROM ADDRESS, DATA 
  FUNCTION RETURN: NONE
  FUCTION DESCRIPTION: STORE GIVEN DATA TO GIVEN EEPROM LOCATION. LIMITATIONS: CAN ACCEPT AND STORE ONLY ONE BYTE DATA
*/
void internalDrivers::writeByteInEEPROM(int memAddress, byte data) {
  Wire.beginTransmission(eepromI2cAddr);  // SEND EEPROM IC ADDRESS OVER I2C NETWORK TO START COMMUNICATION
  Wire.write((memAddress >> 8) & 0xFF);   // SEND HIGH BYTE OF MEMORY ADDRESS
  Wire.write(memAddress & 0xFF);          // SEND LOW BYTE OF MEMORY ADDRESS
  Wire.write(data);                       // SEND DATA TO WRITE IN MEMORY
  Wire.endTransmission();                 // END I2C TRANSMISSION
  delay(10);                              // SMALL DELAY TO SETTLE DOWN THE HARDWAR PROPELY
}

/*
  FUNCTION NAME: READBYTEFROMEEPROM
  FUNCTION INPUT: MEMORY ADDRESS
  FUNCTION RETURN: READ DATA FROM GIVEN MEMORY LOCATIONS
  FUCTION DESCRIPTION: READ THE DATA FROM GIVEN MEMORY ADDRESS. LIMITATIONS: CAN READ ONLY BYTE DATA ONCE
*/
byte internalDrivers::readByteFromEEPROM(int memAddress) {
  byte data = 0;
  Wire.beginTransmission(eepromI2cAddr);         // SEND EEPROM IC ADDRESS OVER I2C NETWORK TO START COMMUNICATION
  Wire.write((memAddress >> 8) & 0xFF);          // SEND HIGH BYTE OF MEMORY ADDRESS
  Wire.write(memAddress & 0xFF);                 // SEND LOW BYTE OF MEMORY ADDRESS
  Wire.endTransmission();                        // END I2C TRANSMISSION
  Wire.requestFrom(int(eepromI2cAddr), int(1));  // REQUEST ONE BYTE DATA FROM EERPOM IC
  if (Wire.available()) {                        // CHECK IF DATA AVAILABLE
    data = Wire.read();                          // READ AVAILABLE DATAm
  }                                              //
  return data;                                   // RETURN READ DATA
}

/*
  FUNCTION NAME: WRITEFLOATINEEPROM
  FUNCTION INPUT: EEPROM ADDRESS, LONG VALUE (4-BYTE DATA)
  FUNCTION RETURN: NONE
  FUNCTION DESCRIPTION: STORES 4-BYTE LONG DATA INTO THE GIVEN EEPROM MEMORY LOCATION.
  LIMITATIONS: CAN ONLY STORE 4-BYTE DATA IN A SINGLE TRANSACTION. THIS FUNCTION IS USEFULL TO STORE FLOAT DATA 
*/
void internalDrivers::writeFloatInEEPROM(int address, long value) {
  Wire.beginTransmission(eepromI2cAddr);  // SEND EEPROM IC ADDRESS OVER I2C NETWORK TO START COMMUNICATION
  Wire.write((address >> 8) & 0xFF);      // SEND HIGH BYTE OF MEMORY ADDRESS
  Wire.write(address & 0xFF);             // SEND LOW BYTE OF MEMORY ADDRESS
  Wire.write((value >> 24) & 0xFF);       // SEND BYTE 3 (MOST SIGNIFICANT BYTE)
  Wire.write((value >> 16) & 0xFF);       // SEND BYTE 2
  Wire.write((value >> 8) & 0xFF);        // SEND BYTE 1
  Wire.write(value & 0xFF);               // SEND BYTE 0 (LEAST SIGNIFICANT BYTE)
  Wire.endTransmission();                 // END I2C TRANSMISSION
  delay(10);                              // SMALL DELAY TO ALLOW ENOUGH TIME FOR EEPROM WRITE
}

/*
  FUNCTION NAME: READFLOATFROMEEPROM
  FUNCTION INPUT: MEMORY ADDRESS
  FUNCTION RETURN: LONG VALUE (4-BYTE DATA) READ FROM THE GIVEN MEMORY LOCATION
  FUNCTION DESCRIPTION: READ 4-BYTE LONG DATA FROM THE GIVEN MEMORY ADDRESS OF EEPROM.
*/
long internalDrivers::readFloatFromEEPROM(int address) {
  long value = 0;
  Wire.beginTransmission(eepromI2cAddr);         // SEND EEPROM IC ADDRESS OVER I2C NETWORK TO START COMMUNICATION
  Wire.write((address >> 8) & 0xFF);             // SEND HIGH BYTE OF MEMORY ADDRESS
  Wire.write(address & 0xFF);                    // SEND LOW BYTE OF MEMORY ADDRESS
  Wire.endTransmission();                        // END I2C TRANSMISSION
                                                 //
  Wire.requestFrom(int(eepromI2cAddr), int(4));  // REQUEST 4 BYTE DATA FROM EERPOM IC
  if (Wire.available() == 4) {                   // CHECK IF REQUESTED 4 BYTE DATA AVAILABLE OR NOT
    value |= (long)Wire.read() << 24;            // READ BYTE 3
    value |= (long)Wire.read() << 16;            // READ BYTE 2
    value |= (long)Wire.read() << 8;             // READ BYTE 1
    value |= (long)Wire.read();                  // READ BYTE 0
  }                                              //
  return value;                                  // RETURN READ LONG VALUE
}

/*
  FUNCTION NAME: TOGGLESTATUSANDMEASUREMENTLED
  FUNCTION INPUT: NONE
  FUNCTION RETURN: NONE
  FUNCTION DESCRIPTION: TOGGLES THE STATE OF STATUS AND MEASUREMENT LEDS BY FLIPPING THEIR CURRENT STATE (ON/OFF). 
*/
void internalDrivers::toggleStatusAndMeasurementLED() {
  ledState = !ledState;                        // TOGGLE THE STATE
  digitalWrite(_statusLEDPin, ledState);       // TRIGGER STATUS LED ACCORDINGLY STATUS
  digitalWrite(_measurementLEDPin, ledState);  // TRIGGER MEASUREMENT LED ACCORDINGLY STATUS
}
