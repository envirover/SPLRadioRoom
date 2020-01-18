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
#include "Logger.h"

using std::cout;
using std::endl;

using mavio::MAVLinkISBDChannel;
using mavio::Serial;

constexpr int isbd_serial_baud_rate = 19200;
constexpr int max_test_time = 600;  // seconds

/**
 * Tests sending message to ISBD transceiver.
 *
 * Returns 0 if the test was successful.
 */
int test_send(MAVLinkISBDChannel& isbd_channel) {
  mavlink_high_latency_t report;
  memset(&report, 0, sizeof(mavlink_high_latency_t));

  mavlink_message_t msg;
  uint8_t sysid = 1;
  uint8_t compid = 0;
  mavlink_msg_high_latency_encode(sysid, compid, &msg, &report);

  std::chrono::milliseconds send_time = isbd_channel.last_send_time();

  isbd_channel.send_message(msg);

  for (int i = 0; i < max_test_time; i++) {
    int quality = 0;
    isbd_channel.get_signal_quality(quality);
    mavio::log(LOG_INFO, "Signal quality: %d",  quality);

    if (isbd_channel.last_send_time() != send_time) {
      mavio::log(LOG_INFO, "Message sent.");
      return 0;
    }

    timelib::sleep(timelib::sec2ms(1.0));
  }

  mavio::log(LOG_INFO, "Message was not sent.");
  return 1;
}

/**
 * Tests receiving message from ISBD transceiver.
 * A message must be sent to the transceiver before the test.
 *
 * Returns 0 if the test was successful.
 */
int test_receive(MAVLinkISBDChannel& isbd_channel) {
  mavlink_message_t msg;

  for (int i = 0; i < max_test_time; i++) {
    int quality = 0;
    isbd_channel.get_signal_quality(quality);
    mavio::log(LOG_INFO, "Signal quality: %d",  quality);

    if (isbd_channel.message_available()) {
      if (isbd_channel.receive_message(msg)) {
        mavio::log(LOG_INFO, "Message received. msgid = %d", msg.msgid);
        return 0;
      }
    }

    timelib::sleep(timelib::sec2ms(1.0));
  }

  return 1;
}

int main(int argc, char** argv) {
  cout << "MAVLinkISBDChannel class test." << endl;

  cout << "Usage: isbdtest <device path> <send|receive>" << endl;

  std::string path = "/dev/ttyUSB0";
  if (argc > 1) {
    path = argv[1];
  }

  std::string test = "send";
  if (argc > 2) {
    test = argv[2];
  }

  std::vector<std::string> devices;
  Serial::get_serial_devices(devices);

  cout << "Device: " << path << endl;
  cout << "Test: " << test << endl;

  MAVLinkISBDChannel isbd_channel;

  int ret = 0;

  if (isbd_channel.init(path, isbd_serial_baud_rate, devices)) {
    cout << "ISBD channel init() succeeded." << endl;

    if (test == "send") {
      ret = test_send(isbd_channel);
    } else if (test == "receive") {
      ret = test_receive(isbd_channel);
    } else {
      cout << "Invalid test." << endl;
      ret = 1;
    }

    isbd_channel.close();
  } else {
    cout << "ISBD channel init() failed." << endl;
    ret = 1;
  }

  return ret;
}
