/*
 MAVLinkISBDChannel.cc

 MAVIO MAVLink I/O library.

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

#include "MAVLinkISBDChannel.h"

namespace mavio {

using std::string;
using std::vector;
using std::chrono::high_resolution_clock;

constexpr size_t max_isbd_channel_queue_size = 10;

constexpr struct timespec isbd_channel_poll_interval[] = {
    {0, 10000000L}};  // 10 ms

MAVLinkISBDChannel::MAVLinkISBDChannel()
    : MAVLinkChannel("isbd"),
      isbd(),
      running(false),
      send_receive_thread(),
      send_queue(max_isbd_channel_queue_size),
      receive_queue(max_isbd_channel_queue_size) {}

MAVLinkISBDChannel::~MAVLinkISBDChannel() {}

bool MAVLinkISBDChannel::init(std::string path, int speed,
                              const vector<string>& devices) {
  bool ret = !isbd.init(path, speed, devices);

  if (!running) {
    running = true;

    std::thread send_receive_th(&MAVLinkISBDChannel::send_receive_task, this);
    send_receive_thread.swap(send_receive_th);
  }

  return ret;
}

void MAVLinkISBDChannel::close() {
  if (running) {
    running = false;

    send_receive_thread.join();
  }

  isbd.close();
}

bool MAVLinkISBDChannel::send_message(const mavlink_message_t& msg) {
  if (msg.len == 0 && msg.msgid == 0) {
    return true;
  }

  send_queue.push(msg);

  return true;
}

bool MAVLinkISBDChannel::receive_message(mavlink_message_t& msg) {
  return receive_queue.pop(msg);
}

bool MAVLinkISBDChannel::message_available() { return !receive_queue.empty(); }

high_resolution_clock::time_point MAVLinkISBDChannel::last_send_time() {
  return send_queue.last_push_time();
}

high_resolution_clock::time_point MAVLinkISBDChannel::last_receive_time() {
  return receive_queue.last_push_time();
}

/**
 * If there are messages in send_queue or ring alert flag of  ISBD transceiver
 * is up, pop send_queue, run send-receive session, and push received messages
 * to receive_queue.
 */
void MAVLinkISBDChannel::send_receive_task() {
  while (running) {
    if (!send_queue.empty() || isbd.message_available()) {
      mavlink_message_t mo_msg, mt_msg;
      if (!send_queue.pop(mo_msg)) {
        mo_msg.len = 0;
        mo_msg.msgid = 0;
      }

      bool received = false;
      if (isbd.send_receive_message(mo_msg, mt_msg, received)) {
        if (received) {
          receive_queue.push(mt_msg);
        }
      }
    }

    nanosleep(isbd_channel_poll_interval, NULL);
  }
}

}  // namespace mavio
