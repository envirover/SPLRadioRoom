 /*
 MAVLinkTest.ino

 MAVLink communication test for SPLRadioRoom.
 
 (C) Copyright 2017 Envirover.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <SoftwareSerial.h>

#include "MAVLinkSerial.h"

#undef F
#include "mavlink/include/standard/mavlink.h"        // Mavlink interface

// Pixhawk telemetry interface
#define AP_TELEM_RX_PIN    2  //connect to Pixhawk TELEM1 pin 2 
#define AP_TELEM_TX_PIN    3  //connect to Pixhawk TELEM1 pin 3 

#define LED_PIN            13

#define AP_TELEM_BAUD_RATE 57600
#define SERIAL_BAUD_RATE   57600


SoftwareSerial telem(AP_TELEM_RX_PIN, AP_TELEM_TX_PIN);
MAVLinkSerial  ardupilot(telem);


void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  telem.begin(AP_TELEM_BAUD_RATE);
  
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  commReceive();
}

/*
 * Debug print of mavlink_message_t message
 */
void printMavlinkMsg(const mavlink_message_t& msg) {
  Serial.println("**");
  Serial.print("msgid = "); Serial.println(msg.msgid);
  Serial.print("compid = "); Serial.println(msg.compid);
}

/*
 * Reads and processes MAVLink messages from ArduPilot.
 */
void commReceive() {
  mavlink_message_t msg;
  
  digitalWrite(LED_PIN, LOW);
  
  if (ardupilot.receiveMessage(msg)) {
    digitalWrite(LED_PIN, HIGH);

    printMavlinkMsg(msg);
  }
}


