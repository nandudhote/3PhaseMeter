/*
   Name Of Project: 3 phase Meter
   MCU used: SMT32
   Functions: Load ON/OFF over MQTT and Power measurement
   Programmer: Nilesh Patle
   Author: EVOLUZN INDIA PRIVATE LIMITED
*/

#include "Config.h"
#include "internalDrivers.h"

/*FOR DEBUG ONLY*/
// #include <SoftwareSerial.h>
// const char rxPin = 5;
// const char txPin = 4;
// SoftwareSerial DEBUGSERIAL(rxPin, txPin);  // RX, TX
/*DEBUG CODE END*/

internalDrivers iDrivers;
voltages voltage;
currents Current;
powers Power;
kwhPower KwhPower;

void setup() {
  // DEBUGSERIAL.begin(9600);  // FOR DEBUGGING
  Serial.begin(115200);
  iDrivers.gpioInit();
  iDrivers.i2cInit();
  iDrivers.spiInit();
  KwhPower = iDrivers.readDataFromEEPROM();
}

void loop() {
  /*it will write in EEPROM When data is changed*/
  if (gotNewLoadStatus) {
    iDrivers.writeByteInEEPROM(loadStateEEPROMAdd, loadState);
    gotNewLoadStatus = false;
  }

  /*Temporary Logic */
  ledState = (ledState) ? false : true;
  digitalWrite(_statusLEDPin, ledState);
  digitalWrite(_measurementLEDPin, ledState);

  voltage.Phase1 += iDrivers.readVoltage(_volDivPinOne, 1.2519, -20.0902);  // calibratedVoltage = 1.2519 * (adcVal) - 5.9098
  voltage.Phase2 += iDrivers.readVoltage(_volDivPinTwo, 1.2519, -22.0902);  // calibratedVoltage = 1.2519 * (adcVal) - 5.9098
  voltage.Phase3 += iDrivers.readVoltage(_volDivPinThree, 1.2519, 5.9098);  // calibratedVoltage = 1.2519 * (adcVal) - 5.9098
  Current.Phase1 += iDrivers.readCurrent(CT1I2cAddr, 0.002, 0.2626);        // y = 0.002x - 0.2626
  Current.Phase2 += iDrivers.readCurrent(CT2I2cAddr, 0.002, 0.2856);        // y = 0.002x - 0.2856
  Current.Phase3 += iDrivers.readCurrent(CT3I2cAddr, 0.002, 0.3067);        // y = 0.002x - 0.3067
  sampleCount += 1;

  unsigned long currentTime = millis();
  if (currentTime - previousTime >= 60000 || (requestOn200 == true && lteRestarted == false)) {
    if (currentTime - previousTime >= 60000) {  // restart to measure 30 seconds loop
      previousTime = currentTime;
    }
    lteRestarted = false;  // This flag enable when LTE module reset due to MQTT disconnection
    if (lteRestartCounter >= 2 || lteBeginFailed) {
      if (iDrivers.lteBegin(ServerMQTT, MqttPort, mqttUserName, mqttUserPassword, deviceID, mqttSubscribeTopic)) {
        lteRestartCounter = 0;
        lteBeginFailed = false;
      } else {
        lteBeginFailed = true;
      }
    }
    voltage = iDrivers.averageVol(voltage, sampleCount);
    Current = iDrivers.averageCur(Current, sampleCount);
    Power = iDrivers.calculatePower(voltage.Phase1, voltage.Phase2, voltage.Phase3, Current.Phase1, Current.Phase2, Current.Phase3);
    KwhPower = iDrivers.calculatePowerInKWH(KwhPower, Power.Phase1, Power.Phase2, Power.Phase3);
    bool networkConnectivityStatus = false;
    if (requestOn200) {
      networkConnectivityStatus = iDrivers.publishMsgOverLTENetwork(mqttPublishTopic, "200", voltage.Phase1, voltage.Phase2, voltage.Phase3, Current.Phase1, Current.Phase2, Current.Phase3, KwhPower.Phase1, KwhPower.Phase2, KwhPower.Phase3, loadState);
      requestOn200 = false;
    } else {
      networkConnectivityStatus = iDrivers.publishMsgOverLTENetwork(mqttPublishTopic, deviceID, voltage.Phase1, voltage.Phase2, voltage.Phase3, Current.Phase1, Current.Phase2, Current.Phase3, KwhPower.Phase1, KwhPower.Phase2, KwhPower.Phase3, loadState);
      dataStoreCounter += 1;
      if (dataStoreCounter >= 5) {
        iDrivers.StoreKwhPowerInEEPROM(KwhPower.Phase1, KwhPower.Phase2, KwhPower.Phase3);
        dataStoreCounter = 0;
      }
    }
    if (networkConnectivityStatus == false) {
      lteRestartCounter += 1;
    } else {
      lteRestartCounter = 0;
    }
    if (lteRestartCounter >= 2 || lteBeginFailed) {  // try it for 3 times
      iDrivers.resetLTENetwork();
    }
    sampleCount = 0;
    voltage.Phase1 = voltage.Phase2 = voltage.Phase3 = Current.Phase1 = Current.Phase2 = Current.Phase3 = 0.0;
  }
  delay(10);
}
