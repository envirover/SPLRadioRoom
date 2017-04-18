/*
 SPLRadioRoom.ino

 Iridium SBD telemetry for ArduPilot.
 
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

#include "IridiumSBD.h"
#include "MAVLinkSerial.h"
#include "BLEConfig.h"
#include "HighLatencyMsg.h"

#undef F
#include "mavlink/include/standard/mavlink.h"        // Mavlink interface

// Default HIGH_LATENCY message reporting period in milliseconds
#define HL_MSG_REPORT_PERIOD 300000L

#define ISBD_MAX_MT_MGS_SIZE 270

// Pixhawk telemetry interface
#define AP_TELEM_RX_PIN    2  //connect to Pixhawk TELEM1 pin 2 
#define AP_TELEM_TX_PIN    3  //connect to Pixhawk TELEM1 pin 3 

// Iridium SBD transceiver interface
#define ISBD_RX_PIN        8  //connect to ISBD TX pin
#define ISBD_TX_PIN        9  //connect to ISBD RX pin
#define ISBD_SLEEP_PIN     10 //connect to ISBD sleep pin

#define LED_PIN            13

#define AP_TELEM_BAUD_RATE 57600
#define ISBD_BAUD_RATE     19200
#define SERIAL_BAUD_RATE   57600

SoftwareSerial telem(AP_TELEM_RX_PIN, AP_TELEM_TX_PIN);
MAVLinkSerial  ardupilot(telem);

SoftwareSerial nss(ISBD_RX_PIN, ISBD_TX_PIN);
IridiumSBD isbd(nss, ISBD_SLEEP_PIN);

BLEConfig config;

HighLatencyMsg highLatencyMsg(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID);

unsigned long lastReportTime = 0;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  telem.begin(AP_TELEM_BAUD_RATE);
  
  pinMode(LED_PIN, OUTPUT);
  
  config.init();
  config.setHighLatencyMsgPeriod(HL_MSG_REPORT_PERIOD);

  // Init SBD
  nss.begin(ISBD_BAUD_RATE);

  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.setPowerProfile(1);
  isbd.begin();

  int signalQuality = -1;
  int err = isbd.getSignalQuality(signalQuality);
  if (err != 0) {
    Serial.print("SignalQuality failed: error ");
    Serial.println(err);
  } else {
    Serial.print("SignalQuality: ");
    Serial.println(signalQuality);
  }
}

void loop() {
  commReceive();

  unsigned long currentTime = millis();

  if (currentTime - lastReportTime > config.getHighLatencyMsgPeriod()) {
    highLatencyMsg.print();

    mavlink_message_t msg;
    highLatencyMsg.encode(msg);

    isbdSession(msg);

    lastReportTime = currentTime;
  }
}

/*
 * Debug print of mavlink_message_t message
 */
void printMavlinkMsg(const mavlink_message_t& msg) {
  Serial.println("**");
  Serial.print("msgid = "); Serial.println(msg.msgid);
  Serial.print("compid = "); Serial.println(msg.compid);
}

/**
 * Sends MT message to ISBD and receives MO message from the inbound message queue if any.
 * Returns true if the ISBD session succeeded.
 */
boolean isbdSendReceiveMessage(const mavlink_message_t& moMsg, mavlink_message_t& mtMsg, boolean& received) {
  uint8_t buf[ISBD_MAX_MT_MGS_SIZE];
  size_t buf_size = sizeof(buf);
  uint16_t len = 0;

  if (moMsg.len != 0 && moMsg.msgid != 0) {
    Serial.println("Sending MO message.");
    printMavlinkMsg(moMsg);

    len = mavlink_msg_to_send_buffer(buf, &moMsg);
  }

  received = false;
  
  nss.listen();
  
  if (isbd.sendReceiveSBDBinary(buf, len, buf, buf_size) != ISBD_SUCCESS) 
    return false;  

  if (buf_size > 0) {
    mavlink_status_t mavlink_status;

    for (size_t i = 0; i < buf_size; i++) {
      if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &mtMsg, &mavlink_status)) {
        received = true;

        Serial.println("MT message received.");
        printMavlinkMsg(mtMsg);

        break;
      }
    }
  }

  return true;
}

/*
 * Send the message to ISBD, recieve all the messages in the 
 * inbound message queue, if any, pass them to ArduPilot, 
 * sends ACKs back to ISBD.
 */
void isbdSession(mavlink_message_t& moMsg) {
  boolean received;
  mavlink_message_t mtMsg;
  boolean ackReceived = false;

  do {
    ackReceived = false;
    
    if (isbdSendReceiveMessage(moMsg, mtMsg, received)) {
      if (received) {
        ardupilot.sendMessage(mtMsg);

        ackReceived = ardupilot.receiveAck(mtMsg, moMsg) ||
                      ardupilot.composeUnconfirmedAck(mtMsg, moMsg);

        if (ackReceived) {
          Serial.println("ACK received form ArduPilot."); 
          printMavlinkMsg(moMsg);
        } else {
          moMsg.len = moMsg.msgid = 0;
        }
      }
    }
  } while (isbd.getWaitingMessageCount() > 0 || ackReceived);
}

/**
 * Filters out MO messages from ArduPilot. 
 */
boolean filterMessage(const mavlink_message_t& msg) {
  //TODO: Add all relevant messages 
  return false;
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
    
    if (filterMessage(msg)) {
      printMavlinkMsg(msg);

      isbdSession(msg);
    }
  }
}

/*
bool ISBDCallback() {
  mavlink_message_t msg;
 
  digitalWrite(LED_PIN, LOW);
  
  if (ardupilot.receiveMessage(msg)) {
    digitalWrite(LED_PIN, HIGH);
   
    updateHighLatencyMsg(high_latency_msg, msg);
  }

  nss.listen();
  
  return true;
}
*/
