/*
 MAVLinkTCPChannel.cc

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

#include "MAVLinkTCPChannel.h"

#include "timelib.h"

namespace mavio {

using timelib::sleep;

constexpr size_t max_tcp_channnel_queue_size = 1024;

const std::chrono::milliseconds tcp_channel_send_interval(10);
const std::chrono::milliseconds tcp_channel_receive_interval(10);

MAVLinkTCPChannel::MAVLinkTCPChannel()
    : MAVLinkChannel("tcp"),
      running(false),
      send_thread(),
      receive_thread(),
      socket(),
      send_queue(max_tcp_channnel_queue_size),
      receive_queue(max_tcp_channnel_queue_size),
      send_time(0),
      receive_time(0) {}

MAVLinkTCPChannel::~MAVLinkTCPChannel() { close(); }

bool MAVLinkTCPChannel::init(const std::string address, uint16_t port) {
  bool ret = socket.init(address, port);

  if (!running) {
    running = true;

    std::thread send_th(&MAVLinkTCPChannel::send_task, this);
    send_thread.swap(send_th);

    std::thread receive_th(&MAVLinkTCPChannel::receive_task, this);
    receive_thread.swap(receive_th);
  }

  return ret;
}

void MAVLinkTCPChannel::close() {
  if (running) {
    running = false;

    receive_thread.join();
    send_thread.join();
  }

  socket.close();
}

bool MAVLinkTCPChannel::send_message(const mavlink_message_t& msg) {
  send_queue.push(msg);

  return true;
}

bool MAVLinkTCPChannel::receive_message(mavlink_message_t& msg) {
  return receive_queue.pop(msg);
}

bool MAVLinkTCPChannel::message_available() { return !receive_queue.empty(); }

std::chrono::milliseconds MAVLinkTCPChannel::last_send_time() {
  return send_time;
}

std::chrono::milliseconds MAVLinkTCPChannel::last_receive_time() {
  return receive_time;
}

void MAVLinkTCPChannel::send_task() {
  while (running) {
    mavlink_message_t msg;

    if (send_queue.pop(msg)) {
      if (socket.send_message(msg)) {
        send_time = timelib::time_since_epoch();
      }
    }

    sleep(tcp_channel_send_interval);
  }
}

void MAVLinkTCPChannel::receive_task() {
  while (running) {
    mavlink_message_t msg;

    if (socket.receive_message(msg)) {
      receive_time = timelib::time_since_epoch();
      receive_queue.push(msg);
    }

    sleep(tcp_channel_receive_interval);
  }
}

}  // namespace mavio
