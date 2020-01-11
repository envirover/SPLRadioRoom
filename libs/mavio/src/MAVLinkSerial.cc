/*
 MAVLinkSerial.cc

 MAVIO MAVLink I/O library.

 (C) Copyright 2017-2019 Envirover.

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

#include <limits.h>
#include <stdio.h>
#include <unistd.h>

#include "MAVLinkLogger.h"

namespace mavio {

using std::string;

MAVLinkSerial::MAVLinkSerial() : serial() {}

bool MAVLinkSerial::init(const string& path, int speed) {
  if (serial.open(path, speed) == 0) {
    return true;
  }

  // mavio::log(LOG_WARNING, "Failed to open serial device '%s'.", path.data());

  return false;
}

void MAVLinkSerial::close() { serial.close(); }

bool MAVLinkSerial::send_message(const mavlink_message_t& msg) {
  if (msg.len == 0 && msg.msgid == 0) {
    return true;
  }

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Copy the message to send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  uint16_t n = serial.write(buf, len);

  if (n == len) {
    MAVLinkLogger::log(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT ?
                       LOG_DEBUG: LOG_INFO, "MAV <<", msg);
  } else {
    MAVLinkLogger::log(LOG_WARNING, "MAV << FAILED", msg);
  }

  return n == len;
}

bool MAVLinkSerial::receive_message(mavlink_message_t& msg) {
  mavlink_status_t mavlink_status;

  int c = serial.read();

  while (c >= 0) {
    // Serial.println(c);

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &mavlink_status)) {
      MAVLinkLogger::log(LOG_DEBUG, "MAV >>", msg);
      return true;
    }

    c = serial.read();
  }

  return false;
}

}  // namespace mavio
