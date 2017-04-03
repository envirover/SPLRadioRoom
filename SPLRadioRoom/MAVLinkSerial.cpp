/*
 MAVLinkSerial.cpp

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

#include "Arduino.h"
#include "MAVLinkSerial.h"

#define RECEIVE_RETRIES         10
#define RECEIVE_RETRY_DELAY     5   //ms

// COMMAND_ACK message was not received from Ardupilot
#define MAV_RESULT_UNCONFIRMED  5  

// MISSION_ACK message was not received from Ardupilot
#define MAV_MISSION_UNCONFIRMED 15

MAVLinkSerial::MAVLinkSerial(SoftwareSerial& serial) : serial(serial)
{
}

bool MAVLinkSerial::sendMessage(const mavlink_message_t& msg)
{
  uint8_t buf[263];
  
  //Copy the message to send buffer 
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  serial.listen();
  
  uint16_t n = serial.write(buf, len);
  
  serial.flush();

  return n == len;
}

bool MAVLinkSerial::receiveMessage(mavlink_message_t& msg)
{
  mavlink_status_t mavlink_status;
  
  // Receive data from stream
  serial.listen();

  while (serial.available() > 0) {
    int c = serial.read();
    
    if (c >= 0) {
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &mavlink_status)) 
        return true;
    }
  }
  
  return false;
}

bool MAVLinkSerial::receiveAck(const mavlink_message_t& msg, mavlink_message_t& ack)
{
  for (int i = 0; i < RECEIVE_RETRIES; i++) {
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
      if (receiveMessage(ack) && ack.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        uint16_t msg_command = mavlink_msg_command_long_get_command(&msg);
        uint16_t ack_command = mavlink_msg_command_ack_get_command(&ack);
        if (msg_command == ack_command) 
          ack.seq = seq++;
          return true;
      }
    } else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
      if (receiveMessage(ack) && ack.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        uint16_t msg_command = mavlink_msg_command_int_get_command(&msg);
        uint16_t ack_command = mavlink_msg_command_ack_get_command(&ack);
        if (msg_command == ack_command) 
          ack.seq = seq++;
          return true;
      }
    } else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM) {
      if (receiveMessage(ack) && ack.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
        ack.seq = seq++;
        return true;
      }
    } else {
      return false;
    }

    delay(RECEIVE_RETRY_DELAY);
  }

  return false;
}

bool MAVLinkSerial::composeUnconfirmedAck(const mavlink_message_t& msg, mavlink_message_t& ack)
{
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_ack_t command_ack;
    command_ack.command = mavlink_msg_command_long_get_command(&msg);
    command_ack.result  = MAV_RESULT_UNCONFIRMED;
    mavlink_msg_command_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &command_ack);
    ack.seq = seq++;
    return true;
  } else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
    mavlink_command_ack_t command_ack;
    command_ack.command = mavlink_msg_command_int_get_command(&msg);
    command_ack.result  = MAV_RESULT_UNCONFIRMED;
    mavlink_msg_command_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &command_ack);
    ack.seq = seq++;
    return true;
  } else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM) {
    mavlink_mission_ack_t mission_ack;
    mission_ack.target_system = msg.sysid;
    mission_ack.target_component = msg.compid;
    mission_ack.type = MAV_MISSION_UNCONFIRMED;
    mavlink_msg_mission_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &mission_ack);
    ack.seq = seq++;
    return true;
  } 
  
  return false;
}

