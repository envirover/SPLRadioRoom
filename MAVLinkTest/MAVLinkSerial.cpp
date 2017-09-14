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

MAVLinkSerial::MAVLinkSerial(SoftwareSerial& serial) : 
  serial(serial), timeout(1000), startMillis(0), seq(0)
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

  int c = timedRead();

  while (c >= 0)
  {
    //Serial.println(c);
    
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &mavlink_status)) 
        return true;

    c = timedRead();
  }

  return false;
}

bool MAVLinkSerial::sendReceiveMessage(const mavlink_message_t& msg, mavlink_message_t& ack) 
{
  for (int i = 0; i < SEND_RETRIES; i++) {
    if (sendMessage(msg)) {
      if (msg.msgid != MAVLINK_MSG_ID_COMMAND_LONG &&
          msg.msgid != MAVLINK_MSG_ID_COMMAND_INT &&
          msg.msgid != MAVLINK_MSG_ID_MISSION_ITEM &&
          msg.msgid != MAVLINK_MSG_ID_PARAM_SET)
        return false;
          
      if (receiveAck(msg, ack)) 
        return true;
    }
  }
  
  return composeFailedAck(msg, ack);
}

bool MAVLinkSerial::receiveAck(const mavlink_message_t& msg, mavlink_message_t& ack)
{
  for (int i = 0; i < RECEIVE_RETRIES; i++) {
    switch (msg.msgid) {
      case MAVLINK_MSG_ID_COMMAND_LONG:
      case MAVLINK_MSG_ID_COMMAND_INT:  
        if (receiveMessage(ack) && ack.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
          //Repackage the message to get around problems with CRC mismatch
          mavlink_command_ack_t command_ack;
          command_ack.command = mavlink_msg_command_ack_get_command(&ack);
          command_ack.result  = mavlink_msg_command_ack_get_result(&ack);
          mavlink_msg_command_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &command_ack);
          ack.seq = seq++;
          return true;
        }
        break;
      case MAVLINK_MSG_ID_MISSION_ITEM:
        if (receiveMessage(ack) && (ack.msgid == MAVLINK_MSG_ID_MISSION_ACK || ack.msgid == MAVLINK_MSG_ID_MISSION_REQUEST)) {
          //Repackage the message to get around problems with CRC mismatch
          mavlink_mission_ack_t missionAck;
          missionAck.target_system = msg.sysid;
          missionAck.target_component = msg.compid;
          missionAck.type = mavlink_msg_mission_ack_get_type(&ack);
          mavlink_msg_mission_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &missionAck);
          ack.seq = seq++;
          return true;
        }
        break;
      case MAVLINK_MSG_ID_PARAM_SET:
        if (receiveMessage(ack) && ack.msgid == MAVLINK_MSG_ID_PARAM_VALUE) {
          //Repackage the message to get around problems with CRC mismatch
          mavlink_param_value_t param_value;
          param_value.param_count = 0;
          param_value.param_index = 0;
          mavlink_msg_param_value_get_param_id(&msg, param_value.param_id);
          param_value.param_value = mavlink_msg_param_set_get_param_value(&msg);
          mavlink_msg_param_value_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &param_value);
          ack.seq = seq++;
          return true;
        }
        break;
      default:
        return false;
    }

    delay(RECEIVE_RETRY_DELAY);
  }

  return false;
}

bool MAVLinkSerial::composeFailedAck(const mavlink_message_t& msg, mavlink_message_t& ack)
{
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG:
      mavlink_command_ack_t command_ack;
      command_ack.command = mavlink_msg_command_long_get_command(&msg);
      command_ack.result  =   MAV_RESULT_FAILED;
      mavlink_msg_command_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &command_ack);
      ack.seq = seq++;
      return true;
    case MAVLINK_MSG_ID_COMMAND_INT:
      command_ack.command = mavlink_msg_command_int_get_command(&msg);
      command_ack.result  =   MAV_RESULT_FAILED;
      mavlink_msg_command_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &command_ack);
      ack.seq = seq++;
      return true;
    case MAVLINK_MSG_ID_MISSION_ITEM:
      mavlink_mission_ack_t mission_ack;
      mission_ack.target_system = msg.sysid;
      mission_ack.target_component = msg.compid;
      mission_ack.type = MAV_MISSION_ERROR;
      mavlink_msg_mission_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &mission_ack);
      ack.seq = seq++;
      return true;
    case MAVLINK_MSG_ID_PARAM_SET:
      mavlink_param_value_t param_value;
      param_value.param_count = 0;
      param_value.param_index = 0;
      mavlink_msg_param_value_get_param_id(&msg, param_value.param_id);
      param_value.param_value = mavlink_msg_param_set_get_param_value(&msg);
      mavlink_msg_param_value_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &param_value);
      ack.seq = seq++;
      return true;
    default:
      ack.len = ack.msgid = 0;
      return false;
  } 
}


// private method to read stream with timeout
int MAVLinkSerial::timedRead()
{
  int c;
  
  startMillis = millis();
  
  do {
    c = serial.read();
  
    if (c >= 0) return c;
  } while(millis() - startMillis < timeout);

  return -1;     // -1 indicates timeout
}

