/*
 MAVLinkHandler.h

Telemetry for MAVLink autopilots.

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
#include <chrono>
#include <iostream>

#include "Logger.h"
#include "MAVLinkAutopilot.h"

using mavio::MAVLinkAutopilot;
using mavio::Serial;
using std::cout;
using std::endl;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

void request_data_streams(MAVLinkAutopilot& autopilot, uint16_t rate);

constexpr char default_autopilot_serial[] = "/dev/ttyusb0";
constexpr int autopilot_serial_baud_rate = 57600;

constexpr uint16_t DATA_STREAM_RATE = 2;  // Hz

constexpr struct timespec autopilot_send_interval[] = {
    {0, 10000000L}};  // 10 milliseconds
// constexpr struct timespec heart_interval[] = { { 0, 10000000L } }; // 10
// milliseconds

int main(int argc, char** argv) {
  cout << "MAVLinkAutopilot class test." << endl;
  cout << "Usage: aptest <device path>" << endl;

  for (int i = 0; i < argc; i++) {
    cout << argv[i] << endl;
  }

  mavio::openlog("aptest", LOG_UPTO(LOG_DEBUG));

  MAVLinkAutopilot autopilot;

  std::string path = default_autopilot_serial;
  int speed = autopilot_serial_baud_rate;
  std::vector<std::string> devices;
  Serial::get_serial_devices(devices);

  if (argc > 1) {
    path = argv[1];
  }

  if (!autopilot.init(path, speed, devices)) {
    cout << "MAVLinkAutopilot.init() failed." << endl;
    return 1;
  }

  cout << "MAVLinkAutopilot.get_system_id() returned "
       << static_cast<int>(autopilot.get_system_id()) << endl;

  request_data_streams(autopilot, DATA_STREAM_RATE);

  high_resolution_clock::time_point start_time = high_resolution_clock::now();
  high_resolution_clock::time_point heartbeat_time =
      high_resolution_clock::now();

  int count = 0;
  const double period = 10.0;

  while (duration_cast<milliseconds>(high_resolution_clock::now() - start_time)
                 .count() /
             1000.0 <=
         period) {
    mavlink_message_t msg;
    if (autopilot.receive_message(msg)) {
      cout << "Message reseived. msgid = " << static_cast<int>(msg.msgid)
           << endl;
      count++;
    }

    if (duration_cast<milliseconds>(high_resolution_clock::now() -
                                    heartbeat_time)
                .count() /
            1000.0 >=
        1.0) {
      heartbeat_time = high_resolution_clock::now();

      mavlink_message_t mt_msg;
      mavlink_msg_heartbeat_pack(mavio::system_id, mavio::component_id, &mt_msg,
                                 MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
      autopilot.send_message(mt_msg);
    }

    nanosleep(autopilot_send_interval, NULL);
  }

  cout << count << " messages received in " << period << " seconds." << endl;

  autopilot.close();

  cout << "MAVLinkAutopilot class test completed." << endl;

  mavio::closelog();

  return 0;
}

void request_data_streams(MAVLinkAutopilot& autopilot, uint16_t rate) {
  mavlink_message_t mt_msg;

  /*
   * Send a heartbeat first
   */
  mavlink_msg_heartbeat_pack(mavio::system_id, mavio::component_id, &mt_msg,
                             MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
  autopilot.send_message(mt_msg);

  /*
   * Request data streams from the autopilot.
   */
  uint8_t req_stream_ids[] = {
      MAV_DATA_STREAM_EXTENDED_STATUS,  // SYS_STATUS, NAV_CONTROLLER_OUTPUT,
                                        // GPS_RAW, MISSION_CURRENT
      MAV_DATA_STREAM_POSITION,         // GLOBAL_POSITION_INT
      // MAV_DATA_STREAM_RAW_CONTROLLER,
      MAV_DATA_STREAM_RAW_SENSORS,  //
      MAV_DATA_STREAM_EXTRA1,       // ATTITUDE
      MAV_DATA_STREAM_EXTRA2,       // VFR_HUD
      MAV_DATA_STREAM_EXTRA3        // MSG_BATTERY2
  };

  constexpr size_t n = sizeof(req_stream_ids) / sizeof(req_stream_ids[0]);
  
  for (size_t i = 0; i < n; ++i) {
    mavlink_msg_request_data_stream_pack(mavio::system_id, mavio::component_id,
                                         &mt_msg, autopilot.get_system_id(),
                                         mavio::ardupilot_component_id,
                                         req_stream_ids[i], rate, 1);
    autopilot.send_message(mt_msg);

    nanosleep(autopilot_send_interval, NULL);
  }
}
