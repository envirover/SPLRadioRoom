 /*
 MAVLinkTest.ino

 Arduino sketch for testing MAVLink communication between ArduPilot and SPLRadioRoom.

 The test reads MAVLink messages from the connected ArduPilot, integrates the messages 
 into HIGH_LATENCY messages and prints the HIGH_LATENCY messages.   
 
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
#include "HighLatencyMsg.h"

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

HighLatencyMsg highLatencyMsg(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID);

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  telem.begin(AP_TELEM_BAUD_RATE);
  
  pinMode(LED_PIN, OUTPUT);

}

void loop() {

  uint8_t req_stream_ids[] = {MAV_DATA_STREAM_EXTRA1, MAV_DATA_STREAM_EXTRA2, MAV_DATA_STREAM_EXTENDED_STATUS, 
                              MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_RAW_CONTROLLER};
  uint16_t req_message_rates[] = {2, 3, 2, 2, 2};

  for (size_t i = 0; i < sizeof(req_stream_ids)/sizeof(req_stream_ids[0]); i++) {
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(255, 1, &msg, 1, 1, req_stream_ids[i], req_message_rates[i], 1);
    ardupilot.sendMessage(msg);
    delay(10);
  }
  
  for (int i = 0; i < 100; i++) {
    commReceive();

    delay(10);
  }

  Serial.println("*** HIGH_LATENCY ***");
  highLatencyMsg.print();
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

    highLatencyMsg.update(msg);
    
    printMavlinkMsg(msg);
  }
}


