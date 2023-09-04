#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(57600);
  mySerial.begin(9600); //38400 für hc-05
  Serial.println("Ready to receive.");
}

void loop() {
  if (mySerial.available()) {
    char receivedChar = mySerial.read();
    Serial.print(receivedChar);
  }
}
