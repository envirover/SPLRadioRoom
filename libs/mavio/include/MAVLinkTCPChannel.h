/*
 MAVLinkTCPChannel.h

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

#ifndef LIBS_MAVIO_INCLUDE_MAVLINKTCPCHANNEL_H_
#define LIBS_MAVIO_INCLUDE_MAVLINKTCPCHANNEL_H_

#include "CircularBuffer.h"
#include "MAVLinkChannel.h"
#include "MAVLinkLib.h"
#include "MAVLinkTCP.h"

#include <atomic>
#include <string>
#include <thread>
#include <chrono>

namespace mavio {

/**
 * Asyncronous sends/receives MAVLink messages to/from a TCP/IP socket.
 */
class MAVLinkTCPChannel : public MAVLinkChannel {
 public:
  /**
   * Constructs an instance of MAVLinkTcpClient.
   */
  MAVLinkTCPChannel();

  /**
   * Closes connection and frees the resources.
   */
  virtual ~MAVLinkTCPChannel();

  /**
   * Connects to the TCP/IP socket at the specified address and port.
   *
   * Returns true if the connection was successful.
   */
  bool init(const std::string address, uint16_t port);

  /**
   * Closes the connection if it was open.
   */
  void close();

  /**
   * Sends the specified MAVLink message to the socket.
   *
   * Returns true if the message was sent successfully.
   */
  bool send_message(const mavlink_message_t& msg);

  /**
   * Receives MAVLink message from the socket.
   *
   * Returns true if a message was received.
   */
  bool receive_message(mavlink_message_t& msg);

  /**
   * Checks if data is available in the socket input buffer.
   *
   * Returns true if data is available.
   */
  bool message_available();

  /**
   * Returns time of the last successfully sent message.
   */
  std::chrono::milliseconds last_send_time();

  /**
   * Returns time of the last successfully received message.
   */
  std::chrono::milliseconds last_receive_time();

 private:
  /**
   * While running is true, retrieves messages from send_queue and sends them to
   * serial.
   */
  void send_task();

  /**
   *  While running is true, receives messages from serial and pushes them to
   * receive_queue.
   */
  void receive_task();

  // send and receive threads are running while this flag is true
  std::atomic<bool> running;
  // Thread of send_task
  std::thread send_thread;
  // Thread of receive_task
  std::thread receive_thread;
  // MAVLink TCP socket connection
  MAVLinkTCP socket;
  // Queue that buffers messages to be sent to the socket
  CircularBuffer<mavlink_message_t> send_queue;
  // Queue that buffers messages received from the socket
  CircularBuffer<mavlink_message_t> receive_queue;
  std::chrono::milliseconds send_time;  // Last send epoch time
  std::chrono::milliseconds receive_time;  // Last receive epoch time
};

}  // namespace mavio

#endif  // LIBS_MAVIO_INCLUDE_MAVLINKTCPCHANNEL_H_
