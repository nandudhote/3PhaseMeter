/*
   Name Of Project: 3 phase Measurement and control system
   MCU used: Arduino Nano
   Functions: Contactor ON/OFF over MQTT using LTE module and Power measurement
   Programmer: Nilesh Patle
   Author: EVOLUZN INDIA PRIVATE LIMITED
*/

#include "Config.h"           // INCLUDE CONFIG FILE
#include "internalDrivers.h"  // INCLUDE INTERNALDRIVER FILE

// #include <SoftwareSerial.h> // FOR DEBUGGING
// const char rxPin = 5;
// const char txPin = 4;
// SoftwareSerial DEBUGSERIAL(rxPin, txPin);  // RX, TX

internalDrivers iDrivers;  // INTERNALDRIVERS LIBRARY OBJECT
voltages voltage;          // VOLTAGE STRUCTURE OBJECT
currents Current;          // CURRNT STRUCTURE OBJECT
powers Power;              // POWER STRUCTURE OBJECT
kwhPower KwhPower;         // POWER IN KWH STRUCTURE OBJECT

void setup() {
  // DEBUGSERIAL.begin(9600);                    // FOR DEBUGGING
  Serial.begin(115200);                      // LTE MODULE SERIAL PORT (LTE MODULE INTERFACED ON DEFAULT SERIAL PORT)
  iDrivers.gpioInit();                       // GPIO INITIALIZATION
  iDrivers.i2cInit();                        // I2C MASTER MODE INITIALIZATION // CT BOARD AND EXTERNAL EEPROM INTERFACED ON I2C
  iDrivers.spiInit();                        // SPI SLAVE MODE INITIALIZATION // ARDUINO PRO MINI INTERFACED TO RECEIVE DATA FROM LTE MODULE AND SEND TO MAIN ARDUINO NANO
  KwhPower = iDrivers.readDataFromEEPROM();  // READ STORED LOAD STATUS AND 3 PHASES POWER FROM EEPROM
}

void loop() {
  /*IT CHECK ANY DATA RECEIVED FOR LOAD STATUS FROM MQTT AND STORE IN EEPROM ACCORDINGLY, AVOID TO THIS OPERATION IN SPI ISR TO INCREASE INTERRUPT LATANCY*/
  if (gotNewLoadStatus) {                                       // THIS FLAG ENABLE IN SPI ISR WHEN DATA RECEIVE FOR LOAD CONTROL
    iDrivers.writeByteInEEPROM(loadStateEEPROMAdd, loadState);  // STORE THE DATA IN EEPROM
    gotNewLoadStatus = false;                                   // CLEAR THE FLAG SO EEPROM WRITING SHOULD DONE WHEN NEW DATA AVAIABLE ONLY
  }

  iDrivers.toggleStatusAndMeasurementLED();  // JUST TOGGLE THE MEASUREMENT AND STATUS LED ON EVERY LOOP INTERATION

  /* READ VOLTAGE, PASS ADC PIN AND CALIBRAION VALUE TO DUNCTION */
  voltage.Phase1 += iDrivers.readVoltage(_volDivPinOne, 1.0705, -17.3233);  // calibratedVoltage = 1.0705x - 3.3233
  voltage.Phase2 += iDrivers.readVoltage(_volDivPinTwo, 1.0705, -17.3233);  // calibratedVoltage = 1.0705x - 3.3233
  voltage.Phase3 += iDrivers.readVoltage(_volDivPinThree, 1.0705, 3.3233);  // calibratedVoltage = 1.0705x - 3.3233

  /* READ CURRENT,  PASS ADC PIN AND CALIBRAION VALUE TO DUNCTION */
  Current.Phase1 += iDrivers.readCurrent(CT1I2cAddr, 0.0018, 0.308);   // y = 0.0018x - 0.308
  Current.Phase2 += iDrivers.readCurrent(CT2I2cAddr, 0.0018, 0.3382);  // y = 0.0018x - 0.3382
  Current.Phase3 += iDrivers.readCurrent(CT3I2cAddr, 0.0018, 0.3978);  // y = 0.0018x - 0.3978
  sampleCount += 1;                                                    // THIS COUNT USE FOR AVERATE THE TOTAL ADDED VALUE WHILE SENDING AND STORING THE VOLTAGE AND CURRENT VALUE

  /* THIS CONDITION WILL EXECUTE EVERY ONE MINUTE */
  unsigned long currentTime = millis();                                                          // CALCULATE CURRENT TIME IN MILLI SECONDS
  if (currentTime - previousTime >= 60000 || (requestOn200 == true && lteRestarted == false)) {  // CHECK IF ONE MINUTES COMPLETED OR GOT REQUEST ON 200
    if (currentTime - previousTime >= 60000) {                                                   // RESTART TO MEASURE ONE MINUTES INTERVAL IF ONE MINUTES DONE FROM LAST RESET
      previousTime = currentTime;
    }
    lteRestarted = false;                                                                                                  // RESET THE FLAG WHICH ENABLED DUE TO MQTT DISCONNECTION // ITS DIABLE HERE TO GIVE 1 MINUTES TIME TO MODULE TO REINITILISE AFTER RESET
    if (lteRestartCounter >= 2 || lteBeginFailed) {                                                                        // INITIALLY lteRestartCounter IS 2. THE LOGIC IS GIVE 1 MINTS OF TIME TO LTE MODULE TO GIVE PROPER DELAY TO INITIALISE THE MODULE THEM BEGIN IT // IF MODULED FAILED TO BEGIN IT WILL REBIN IT AFTER 1 MINTS OR MODULE ENABLE TO SEND DATA OVER MQTT
      if (iDrivers.lteBeginWithSSL(ServerMQTT, MqttPort, mqttUserName, mqttUserPassword, deviceID, mqttSubscribeTopic)) {  // BEGIN THE MODULE AND RETURN TRUE FOR SUCESSFUL BEGIN AND FALSE FOR FAILED TO BEGIN
        lteRestartCounter = 0;                                                                                             // SET IT ZERO TO MODULE SHOULD NOT INITIALISE REPEATEDLY WITHOUT FAILED TO CONNECT TO MQTT
        lteBeginFailed = false;                                                                                            // SET IT FALSE ON SUCESSFUL BEGIN
      } else {                                                                                                             // FAILED TO BEGIN
        lteBeginFailed = true;                                                                                             // SET IT TRUE FOR SET BEGIN TRY DUE TO BEGIN UNSUCCSSFUL
      }
    }
    voltage = iDrivers.averageVol(voltage, sampleCount);                                                                              // AVERAGE OUT THE VOLTAGE WHICH ADDED OVER 1 MINTS DURATION
    Current = iDrivers.averageCur(Current, sampleCount);                                                                              // AVERAGE OUT THE CURRENT WHICH ADDED OVER 1 MINTS DURATION
    Power = iDrivers.calculatePower(voltage.Phase1, voltage.Phase2, voltage.Phase3, Current.Phase1, Current.Phase2, Current.Phase3);  // CALCULATE THE POWER FROM AVERGE VOLTAGE AND CURRENT
    KwhPower = iDrivers.calculatePowerInKWH(KwhPower, Power.Phase1, Power.Phase2, Power.Phase3);                                      // CALCULATE THE POWER IN KWH FORMAT
    bool networkConnectivityStatus = false;                                                                                           // THIS FLAG SHOW WHETHER DATA HAS SUCCESSFULL SENT OVER MQTT OR NOT
    if (requestOn200) {                                                                                                               // CHECK IF 200 REQUEST RECEIVED FORM SERVER FOR DATA
      networkConnectivityStatus = iDrivers.publishMsgOverLTENetwork(mqttPublishTopic, "200", voltage.Phase1, voltage.Phase2, voltage.Phase3, Current.Phase1, Current.Phase2, Current.Phase3, KwhPower.Phase1, KwhPower.Phase2, KwhPower.Phase3, loadState);
      if (networkConnectivityStatus) requestOn200 = false;  // RESET THE FLAG AS DATA SENT OVER MQTT SUCCESSFULLY
    } else {                                                // ELSE SEND DATA OVER MQTT ON 1 MINTS INTERVAL

      networkConnectivityStatus = iDrivers.publishMsgOverLTENetwork(mqttPublishTopic, deviceID, voltage.Phase1, voltage.Phase2, voltage.Phase3, Current.Phase1, Current.Phase2, Current.Phase3, KwhPower.Phase1, KwhPower.Phase2, KwhPower.Phase3, loadState);
      dataStoreCounter += 1;
      if (dataStoreCounter >= 5) {                                                          // STORE DATA IN EEPROM IN EVERY 5 MINTS TO ENHANCE THE EEPROM LIFE
        iDrivers.StoreKwhPowerInEEPROM(KwhPower.Phase1, KwhPower.Phase2, KwhPower.Phase3);  // WRITE POWER IN EEPROM
        dataStoreCounter = 0;                                                               // RESET THE COUNTER TO SEND DATA IN NEXT 5 MINTS
      }
    }
    if (networkConnectivityStatus == false) {  // IF DATA FAILED TO SEND OVER MQTT INCREASE lteRestartCounter // IF lteRestartCounter >= 2 THEN LTE MODULE WILL RESET
      lteRestartCounter += 1;                  // INCREASE COUNTER BY ONE BY FAILURE OF DATA SEND
    } else {                                   // ELSE
      lteRestartCounter = 0;                   // RESET THE COUNTER ON SUCCESSFULL DATA SENT
    }
    if (lteRestartCounter >= 2 || lteBeginFailed) {  // IF LTE MODULE FAILED TO SEND DATA OVER MQTT FOR >= 2 AND FAILED TO BEGIN THEN RESTART THE MODULE BY SOFTWARE COMMAND // HARDWARE RESET NOT REQUIRED
      iDrivers.resetLTENetwork();                    // LTE MODULE SOFT RESET
    }
    sampleCount = 0;                                                                                            // SET AVERAGE COUNT TO ZERO
    voltage.Phase1 = voltage.Phase2 = voltage.Phase3 = Current.Phase1 = Current.Phase2 = Current.Phase3 = 0.0;  // RESET THE INSTANT VOLTAGE AND CURRENT
  }
  // delay(10);
}
