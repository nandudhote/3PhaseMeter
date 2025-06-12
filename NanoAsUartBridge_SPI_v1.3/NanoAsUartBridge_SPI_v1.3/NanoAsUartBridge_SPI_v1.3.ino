#include <SoftwareSerial.h>
#include <SPI.h>

// const char rxPin = 9;
// const char txPin = 8;
// SoftwareSerial //+DEBUGSERIAL(rxPin, txPin);  // RX, TX

const int SS_PIN = 10;  // Slave Select pin

byte counterValue = 0;  // Holds the extracted counter value as a byte
bool validateRecDataFlag = false;
String response;

void setup() {
  //+DEBUGSERIAL.begin(9600);
  Serial.begin(115200);

  SPI.begin();                          // Initialize SPI as Master
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // Sets clock for SPI communication at 2 MHz
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);  // Set Slave Select high (inactive)
  //+DEBUGSERIAL.println("SPI Master Initialized.");
}

void loop() {  // Read data from Serial
  while (Serial.available()) {
    response = Serial.readString();  // Read module response
    validateRecDataFlag = true;
  }
  if (validateRecDataFlag) {
    processLine(response);
    validateRecDataFlag = false;
  }
}

// Function to process each line of incoming data
void processLine(String line) {
  //+DEBUGSERIAL.print("Rec Data: ");
  //+DEBUGSERIAL.println(line);

  // if (strstr(line.c_str(), "OK")) {
  //   //+DEBUGSERIAL.println("Acknowledgment Received (OK)");
  // }

  // if (strstr(line.c_str(), "ERROR")) {
  //   //+DEBUGSERIAL.println("Acknowledgment Received (ERROR)");
  // }

  if (strstr(line.c_str(), "+CMQTTCONNECT: 0,0")) {
    //+DEBUGSERIAL.println("Connected to Broker Successfully!");
    signalMega(1);  // Send 1 for acknowledgment
  } else if (strstr(line.c_str(), "+CMQTTSUB: 0,0")) {
    //+DEBUGSERIAL.println("Topic Subscribed Successfully!");
    signalMega(2);  // Send 1 for acknowledgment
  } else if (strstr(line.c_str(), "+CMQTTPUB: 0,0")) {
    //+DEBUGSERIAL.println("Msg Published Successfully!");
    signalMega(3);  // Send 1 for acknowledgment
  } else if (strstr(line.c_str(), "3P:")) {
    extractCounter(line.c_str());  // Extract the counter value
    signalMega(counterValue);      // Send counter value to Mega
    counterValue = 0;              // Reset counterValue after sending
  } else {
    ;
  }
}

// Function to extract the counter value
void extractCounter(const char* payloadLine) {
  const char* start = strstr(payloadLine, "3P:");
  if (start) {
    start += 3;                        // Move past "3P:"
    counterValue = (byte)atoi(start);  // Convert extracted value to a byte
  }
}

// Function to send data to Arduino Mega
void signalMega(byte dataToSend) {
  // Start communication
  digitalWrite(SS_PIN, LOW);  // Activate the slave
  delay(20);
  byte receivedData = SPI.transfer(dataToSend);  // Send and receive data
  delay(10);
  digitalWrite(SS_PIN, HIGH);  // Deactivate the slave
}
