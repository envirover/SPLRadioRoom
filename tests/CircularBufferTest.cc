/*
 CircularBufferTest.h

 MAVIO MAVLink I/O library

 (C) Copyright 2019 Envirover.

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
#include <cassert>
#include <iostream>

#include "CircularBuffer.h"
#include "MAVLinkLib.h"

using std::cout;
using std::endl;

constexpr size_t max_queue_size = 10;

void get_test_message(int i, mavlink_message_t* msg) {
  mavlink_heartbeat_t heartbeat;
  heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
  heartbeat.base_mode = 1;
  heartbeat.custom_mode = 0;
  heartbeat.mavlink_version = MAVLINK_VERSION;
  heartbeat.system_status = 0;
  heartbeat.type = MAV_TYPE_GENERIC;

  mavlink_msg_heartbeat_encode(255, i, msg, &heartbeat);
}

int main() {
  cout << "CircularBuffer test started." << endl;

  mavio::CircularBuffer<mavlink_message_t> queue(max_queue_size);

  mavlink_message_t msg;
  get_test_message(0, &msg);
  queue.push(msg);

  queue.reset();

  assert(queue.size() == 0);

  cout << "Queue size:" << static_cast<int>(queue.size())
       << ", Queue capacity:" << queue.capacity() << endl;

  assert(queue.size() == 0);
  assert(queue.capacity() == max_queue_size);

  cout << "Pushing messages..." << endl;

  for (uint8_t i = 0; i < 100; ++i) {
    get_test_message(i, &msg);
    queue.push(msg);

    cout << "i : " << static_cast<int>(i) << ", full : " << queue.full() << endl;
  }

  assert(queue.size() == 10);

  cout << "Retrieving messages..." << endl;

  while (!queue.empty()) {
    queue.pop(msg);
    cout << "Message retrieved: " << static_cast<int>(msg.compid)
         << ", Queue size: " << static_cast<int>(queue.size()) << endl;
  }

  assert(queue.size() == 0);

  cout << "CircularBuffer test completed." << endl;

  return 0;
}
