/*
 MAVLinkISBDChannelTest.cc

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
#include <iostream>

#include "MAVLinkISBDChannel.h"
#include "Serial.h"
#include "timelib.h"

using std::cout;
using std::endl;

using mavio::MAVLinkISBDChannel;
using mavio::Serial;

constexpr int isbd_serial_baud_rate = 19200;
constexpr int max_send_time = 600;  // seconds

int main(int argc, char** argv) {
  cout << "MAVLinkISBDChannel class test." << endl;

  if (argc < 2) {
    cout << "Usage: isbdtest <path>" << endl;
    return 1;
  }

  std::string path = argv[1];
  std::vector<std::string> devices;
  Serial::get_serial_devices(devices);

  cout << "Testing ISBD channel " << path << "..." << endl;

  MAVLinkISBDChannel isbd_channel;

  if (isbd_channel.init(path, isbd_serial_baud_rate, devices)) {
    cout << "ISBD channel init() succeeded." << endl;

    mavlink_high_latency_t report;
    report.latitude = 1;
    report.longitude = 2;

    mavlink_message_t msg;
    uint8_t sysid = 1;
    uint8_t compid = 0;
    mavlink_msg_high_latency_encode(sysid, compid, &msg, &report);

    std::chrono::milliseconds send_time = isbd_channel.last_send_time();

    isbd_channel.send_message(msg);

    for (int i = 0; i < max_send_time; i++) {
      if (isbd_channel.last_send_time() != send_time) {
        cout << "Message sent." << endl;
        send_time = isbd_channel.last_send_time();
      }

      if (isbd_channel.message_available()) {
        if (isbd_channel.receive_message(msg)) {
          cout << "Message received. msgid = " << static_cast<int>(msg.msgid)
          << endl;
        }
      }

      timelib::sleep(timelib::sec2ms(1.0));
    }

    if (isbd_channel.last_send_time() == send_time) {
      cout << "Message was not sent." << endl;
    }

    isbd_channel.close();
  } else {
    cout << "ISBD channel init() failed." << endl;
    return 1;
  }

  return 0;
}
