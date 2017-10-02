/*
 MAVLinkSerial.h

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
#include <ctime>
#include "Serial.h"

#undef F
#include "mavlink/include/standard/mavlink.h"      

#define ARDUPILOT_SYSTEM_ID     1
#define ARDUPILOT_COMPONENT_ID  1

#define SEND_RETRIES            5
#define RECEIVE_RETRIES         10
#define RECEIVE_RETRY_DELAY     5   //ms

/**
 * MAVLinkSerial is used to send and receive MAVLink messages to/from a serial interface.
 */
class MAVLinkSerial 
{
  Serial& serial;
  unsigned long timeout;      // number of milliseconds to wait for the next char before aborting timed read
  clock_t startMillis;  // used for timeout measurement
  uint8_t seq;

  int timedRead();    // private method to read stream with timeout
  
public:

  /**
   * Constructs MAVLinkSerial instance using the specified serial interface.
   */
  MAVLinkSerial(Serial& serial);

  /**
   * Send MAVLink message to ArduPilot.
   */
  bool sendMessage(const mavlink_message_t& msg);
  
  /**
   * Receive MAVLink message from ArduPilot.
   */
  bool receiveMessage(mavlink_message_t& msg);

  /**
   * Retries sending message to ArduPilot until ACK is received. 
   * Returns true if ACK received.
   */
  bool sendReceiveMessage(const mavlink_message_t& msg, mavlink_message_t& ack);

private:

  /**
   * Receive messages from serial several times until received
   * COMMAND_ACK for COMMAND_LONG and COMMAND_INT or 
   * MISSION_ACK for MISSION_ITEM message.
   */
  bool receiveAck(const mavlink_message_t& msg, mavlink_message_t& ack);

  /**
   * Compose an unconfirmed COMMAND_ACK or MISSION_ACK message.
   * Unconfirmed ACK messages are sent to GCS if ACK message was not 
   * received from ArduPilot.
   */
  bool composeFailedAck(const mavlink_message_t& msg, mavlink_message_t& ack);

};

