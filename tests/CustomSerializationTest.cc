/*
 CustomSerializationTest.h

 MAVIO MAVLink I/O library

 (C) Copyright 2021 Envirover.

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
#include "MAVLinkLib.h"

#include <cassert>
#include <iostream>

using std::cout;
using std::endl;

namespace mavio {
// Custom serialization/deerialization helpers
uint16_t message_to_send_buffer(uint8_t *buf, const mavlink_message_t *msg);
bool parse_message(const uint8_t *buf, size_t buf_size, mavlink_message_t *msg);
}  // namespace mavio

int main() {
  cout << "Custom serialization test started." << endl;

  // Test MAVLink 2.0 serialization using HIGH_LATENCY2 message
  mavlink_high_latency2_t high_latency2;
  memset(&high_latency2, 0, sizeof(high_latency2));
  high_latency2.timestamp = 1;
  high_latency2.airspeed = 1;
  high_latency2.custom2 = 1;

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t msg;
  memset(&msg, 0, sizeof(mavlink_message_t));
  mavlink_msg_high_latency2_encode(1, 0, &msg, &high_latency2);

  uint8_t len = mavlink_msg_to_send_buffer(buf, &msg);

  cout << "Original message length: " << (long)len << endl;

  len = mavio::message_to_send_buffer(buf, &msg);

  cout << "Custom message length: " << (long)len << endl;

  mavlink_message_t parsed_msg;
  memset(&parsed_msg, 0, sizeof(mavlink_message_t));
  if (mavio::parse_message(buf, len, &parsed_msg)) {
    assert(msg.checksum == parsed_msg.checksum);
    assert(memcmp(&msg.payload64, &parsed_msg.payload64, msg.len) == 0);

    mavlink_high_latency2_t high_latency2_parsed;
    memset(&high_latency2_parsed, 0, sizeof(high_latency2_parsed));
    mavlink_msg_high_latency2_decode(&parsed_msg, &high_latency2_parsed);
    assert(memcmp(&high_latency2_parsed, &high_latency2,
                  sizeof(high_latency2)) == 0);
  } else {
    cout << "Failed to parse MAVLink message." << endl;
    return 1;
  }

  // Test MAVLink 1.0 Serialization using HEARTBEAT message.
  mavlink_heartbeat_t heartbeat;
  heartbeat.custom_mode = 1;
  heartbeat.type = 0;
  heartbeat.autopilot = 0;
  heartbeat.base_mode = 0;
  heartbeat.system_status = 0;
  heartbeat.mavlink_version = 1;

  mavlink_message_t tmp_msg;
  mavlink_msg_heartbeat_encode(1, 0, &tmp_msg, &heartbeat);

  memset(&msg, 0, sizeof(mavlink_message_t));
  msg.checksum = 52057;
  msg.magic = MAVLINK_STX_MAVLINK1;
  msg.len = MAVLINK_MSG_ID_HEARTBEAT_LEN;
  msg.sysid = 1;
  msg.compid = 0;
  msg.msgid = MAVLINK_MSG_ID_HEARTBEAT;
  memcpy(_MAV_PAYLOAD_NON_CONST(&msg), _MAV_PAYLOAD(&tmp_msg), msg.len);


  len = mavio::message_to_send_buffer(buf, &msg);
  memset(&parsed_msg, 0, sizeof(mavlink_message_t));

  if (mavio::parse_message(buf, len, &parsed_msg)) {
    assert(msg.checksum == parsed_msg.checksum);
    assert(memcmp(&msg.payload64, &parsed_msg.payload64, msg.len) == 0);
  } else {
    cout << "Failed to parse MAVLink message." << endl;
    return 1;
  }

  cout << "Custom serialization test succeeded." << endl;
  return 0;
}
