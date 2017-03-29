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

#include "MAVLinkSerial.h"

MAVLinkSerial::MAVLinkSerial(SoftwareSerial& serial) : serial(serial)
{
}

bool MAVLinkSerial::sendMessage(const mavlink_message_t& msg)
{
  uint8_t buf[263];
  
  //Copy the message to send buffer 
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  //Send the message to ArduPilot
  serial.listen();
  serial.write(buf, len);

  return true;
}

bool MAVLinkSerial::receiveMessage(mavlink_message_t& msg)
{
  mavlink_status_t mavlink_status;
  
  // Receive data from stream
  serial.listen();

  while (serial.available() > 0) {
    uint8_t c = serial.read();

    // Try to get a new message 
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &mavlink_status)) 
      return true;
  }

  return false;
}

