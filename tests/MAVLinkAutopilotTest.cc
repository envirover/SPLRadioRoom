/*
 MAVLinkAutopilotTest.h

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

#include "Logger.h"
#include "MAVLinkAutopilot.h"
#include "timelib.h"

using std::cout;
using std::endl;

using mavio::MAVLinkAutopilot;
using mavio::Serial;
using timelib::sec2ms;
using timelib::sleep;
using timelib::Stopwatch;

void request_data_streams(MAVLinkAutopilot& autopilot, uint16_t rate);

constexpr char default_autopilot_serial[] = "/dev/ttyusb0";
constexpr int autopilot_serial_baud_rate = 57600;

constexpr uint16_t DATA_STREAM_RATE = 2;  // Hz

const std::chrono::milliseconds autopilot_send_interval(10);

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

  Stopwatch start_time;
  Stopwatch heartbeat_time;

  int count = 0;
  const std::chrono::milliseconds period = sec2ms(10.0);
  const std::chrono::milliseconds heartbeat_interval = sec2ms(1.0);

  while (start_time.elapsed_time() <= period) {
    mavlink_message_t msg;
    if (autopilot.receive_message(msg)) {
      cout << "Message reseived. msgid = " << static_cast<int>(msg.msgid)
           << endl;
      count++;
    }

    if (heartbeat_time.elapsed_time() >= heartbeat_interval) {
      heartbeat_time.reset();

      mavlink_message_t mt_msg;
      mavlink_msg_heartbeat_pack(mavio::gcs_system_id, mavio::gcs_component_id,
                                 &mt_msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
                                 0, 0, 0);
      autopilot.send_message(mt_msg);
    }

    sleep(autopilot_send_interval);
  }

  cout << count << " messages received in " << period.count() << " ms." << endl;

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
  mavlink_msg_heartbeat_pack(mavio::gcs_system_id, mavio::gcs_component_id,
                             &mt_msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0,
                             0);
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
    mavlink_msg_request_data_stream_pack(
        mavio::gcs_system_id, mavio::gcs_component_id, &mt_msg,
        autopilot.get_system_id(), mavio::ardupilot_component_id,
        req_stream_ids[i], rate, 1);
    autopilot.send_message(mt_msg);

    sleep(autopilot_send_interval);
  }
}
