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
#include "SPLConfig.h"
#include "HighLatencyMsg.h"

#undef F
#include "mavlink/include/standard/mavlink.h"        // Mavlink interface

// Default HIGH_LATENCY message reporting period in milliseconds
#define HL_REPORT_PERIOD_PARAM "HL_REPORT_PERIOD"

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

#define MAX_SEND_RETRIES   5

#define MAX_MISSION_COUNT  7

SoftwareSerial telem(AP_TELEM_RX_PIN, AP_TELEM_TX_PIN);
MAVLinkSerial  ardupilot(telem);

SoftwareSerial nss(ISBD_RX_PIN, ISBD_TX_PIN);
IridiumSBD isbd(nss, ISBD_SLEEP_PIN);

SPLConfig config;

HighLatencyMsg highLatencyMsg(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID);

mavlink_message_t missions[MAX_MISSION_COUNT];

unsigned long lastReportTime = 0;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  telem.begin(AP_TELEM_BAUD_RATE);
  
  pinMode(LED_PIN, OUTPUT);
  
  config.init();

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
  for (int i = 0; i < 10; i++) {
    commReceive();
  }

  nss.listen();

  uint16_t moFlag = 0, moMSN = 0, mtFlag = 0, mtMSN = 0, raFlag = 0, msgWaiting = 0;
  
  int err = isbd.getStatusExtended(moFlag, moMSN, mtFlag, mtMSN, raFlag, msgWaiting);
  
  if (err != 0) {
    Serial.print("SBDSX failed: error ");
    Serial.println(err);
  } else {
    Serial.print("Ring Alert flag: ");
    Serial.println(raFlag);
  }
  
  unsigned long currentTime = millis();

  // Start ISBD session if ring alert is received or HIGH_LATENCY report period is elapsed.
  if (raFlag || currentTime - lastReportTime > config.getReportPeriod()) {
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
 * 
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
 * Sends the message to ISBD, recieve all the messages in the 
 * inbound message queue, if any, pass them to ArduPilot, 
 * sends ACKs back to ISBD.
 */
void isbdSession(mavlink_message_t& moMsg) {
  boolean received;
  boolean ackReceived = false;
  mavlink_message_t mtMsg;

  do {
    ackReceived = false;
    
    if (isbdSendReceiveMessage(moMsg, mtMsg, received)) {
      moMsg.len = moMsg.msgid = 0;
      if (received) {
        ackReceived = handleParamSet(mtMsg, moMsg);

        if (!ackReceived) { 
           ackReceived = handleMissionWrite(mtMsg, moMsg);
           Serial.println("MISSION_ACK received."); 
        }
        
        if (!ackReceived) {
          ackReceived = ardupilot.sendReceiveMessage(mtMsg, moMsg);
          
          if (ackReceived) {
            Serial.println("ACK received from ArduPilot."); 
            printMavlinkMsg(moMsg);
          }
        }
      }
    }
  } while (isbd.getWaitingMessageCount() > 0 || ackReceived);
}

/**
 * Filters out MO messages from ArduPilot. 
 * 
 * Returns true if the message is allowed to pass though the filter.
boolean filterMessage(const mavlink_message_t& msg) {
  //TODO: Add all relevant messages 
  return false;
}
*/
 
/*
 * Reads and processes MAVLink messages from ArduPilot.
 */
void commReceive() {
  mavlink_message_t msg;
  
  digitalWrite(LED_PIN, LOW);
  
  if (ardupilot.receiveMessage(msg)) {
    digitalWrite(LED_PIN, HIGH);
    
    highLatencyMsg.update(msg);

    /*
    if (filterMessage(msg)) {
      printMavlinkMsg(msg);

      isbdSession(msg);
    }
    */
  }
}

/*
 * Updates HIGH_LATENCY message reporting period if HL_REPORT_PERIOD parameter value is set by 
 * PARAM_SET MT message.
 * 
 * Returns true if the message was handled.
 */
boolean handleParamSet(const mavlink_message_t& msg, mavlink_message_t& ack) {
  if (msg.msgid == MAVLINK_MSG_ID_PARAM_SET) {
    char param_id[17];
    mavlink_msg_param_set_get_param_id(&msg, param_id);
    if (strncmp(param_id, HL_REPORT_PERIOD_PARAM, 16) == 0) {
      float value = mavlink_msg_param_set_get_param_value(&msg);
      config.setReportPeriod(value * 1000);

      mavlink_param_value_t paramValue;
      paramValue.param_value = value;
      paramValue.param_count = 0;
      paramValue.param_index = 0;
      mavlink_msg_param_set_get_param_id(&msg, paramValue.param_id);
      paramValue.param_type = mavlink_msg_param_set_get_param_type(&msg);

      mavlink_msg_param_value_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &paramValue);

      return true;
    }
  }

  return false;
}

/*
 * Handles writing waypoints list as described  in 
 * http://qgroundcontrol.org/mavlink/waypoint_protocol
 * 
 * returns true if waypoints list was updated in ardupilot 
 */
boolean handleMissionWrite(const mavlink_message_t& msg, mavlink_message_t& ack) {
  if (msg.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {
    Serial.println("MISSION_COUNT MT message received.");
    
    uint16_t count = mavlink_msg_mission_count_get_count(&msg);

    if (count > MAX_MISSION_COUNT) {
      Serial.println("Not enough memory for storing missions.");
      
      mavlink_mission_ack_t mission_ack;
      mission_ack.target_system = msg.sysid;
      mission_ack.target_component = msg.compid;
      mission_ack.type = MAV_MISSION_NO_SPACE;
      mavlink_msg_mission_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &mission_ack);
      ack.seq = 0; //TODO: use global counter for message sequence numbers.

      return true;
    }
    
    mavlink_message_t mtMsg, moMsg;
    moMsg.len = moMsg.msgid = 0;

    Serial.println("Receiving mission items from ISBD.");
    
    uint16_t idx = 0;
    
    for (uint16_t i = 0; i < count * MAX_SEND_RETRIES && idx < count; i++) {
      boolean received = false;   
      
      if (isbdSendReceiveMessage(moMsg, mtMsg, received)) {
        if (received && mtMsg.msgid == MAVLINK_MSG_ID_MISSION_ITEM) {
          Serial.println("MISSION_ITEM MT message received.");
          memcpy(missions + idx, &mtMsg, sizeof(mavlink_message_t));
          idx++;
        }
      } else {
        delay(5000);
      }
    }

    if (idx != count) {
      Serial.println("Not all mission items received.");

      mavlink_mission_ack_t mission_ack;
      mission_ack.target_system = msg.sysid;
      mission_ack.target_component = msg.compid;
      mission_ack.type = MAV_MISSION_ERROR;
      mavlink_msg_mission_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &mission_ack);
      ack.seq = 0; //TODO: use global counter for message sequence numbers.

      return true;
    }
      
    Serial.println("Sending mission items to ArduPilot...");
    
    for (int i = 0; i < MAX_SEND_RETRIES; i++) {
      if (ardupilot.sendMessage(msg)) {
        break;
      }

      delay(10);
    }

    for (uint16_t i = 0; i < count; i++) {
      ardupilot.sendReceiveMessage(missions[i], ack);
      
      Serial.println("Mission item sent to ArduPilot.");
      
      delay(10);
    }

    if (mavlink_msg_mission_ack_get_type(&ack) == MAV_MISSION_ACCEPTED) {
      Serial.println("Mission accepted by ArduPilot.");
    } else {
      Serial.print("Mission not accepted by ArduPilot: ");
      Serial.println(mavlink_msg_mission_ack_get_type(&ack));
    }

    return true;
  }

  return false;
}

/*
bool ISBDCallback() {
  mavlink_message_t msg;
 
  digitalWrite(LED_PIN, LOW);
  
  if (ardupilot.receiveMessage(msg)) {
    digitalWrite(LED_PIN, HIGH);
   
    highLatencyMsg.update(msg);
  }

  nss.listen();
  
  return true;
}
*/
