/*
 This is a simple chat tool to communicate with RockBLOCK. 
 The sketch reads bytes from RockBLOCK and sends them to Serial (baud rate 115200), 
 and vice-versa, reads bytes from Serial and sends them to RockBLOCK. 

 Hint: In Serial Monitor Arduino windows set line ending to "Carriage Return".

 These commands will transmit message to RockBLOCK:
 AT&K0\r
 AT+SBDWT=Hello World\r
 AT+SBDIX\r
 */

#include "SoftwareSerial.h"

#define ROCKBLOCK_RX    8
#define ROCKBLOCK_TX    9
#define ROCKBLOCK_ONOFF 10

#define SERIAL_BAUD     57600

SoftwareSerial nss(ROCKBLOCK_RX, ROCKBLOCK_TX);

void setup() {
  pinMode(ROCKBLOCK_ONOFF, OUTPUT);
  digitalWrite(ROCKBLOCK_ONOFF, HIGH);

  Serial.begin(SERIAL_BAUD);
  nss.begin(19200);

  for (int i = 10; i > 0; i--) {
    delay(1000);
    Serial.println(i);
  }  
  
  Serial.print("Hi. I'm "); 
  nss.write("AT+CGMM\r"); 
  Serial.print(nss.readString());
  nss.write("AT+CGSN\r");
  Serial.print(nss.readString());

  Serial.println("Let's talk...");
}

void loop() {
  char c = nss.read();

  if (c != -1) {
    Serial.write(c);
  }

  c = Serial.read();

  if (c != -1) {
    nss.write(c);
  }
}
