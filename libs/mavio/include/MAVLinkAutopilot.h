/*
 MAVLinkAutopilot.h
 
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

#ifndef LIBS_MAVIO_INCLUDE_MAVLINKAUTOPILOT_H_
#define LIBS_MAVIO_INCLUDE_MAVLINKAUTOPILOT_H_

#include "CircularBuffer.h"
#include "MAVLinkChannel.h"
#include "MAVLinkLib.h"
#include "MAVLinkSerial.h"

#include <atomic>
#include <thread>
#include <vector>

namespace mavio {

constexpr uint8_t gcs_system_id = 255;   // GCS system Id
constexpr uint8_t gcs_component_id = 1;  // GCS component Id
constexpr uint8_t ardupilot_component_id = 0;

/**
 * Asynchronously sends and receivs MAVLink messages to autopilot over serial
 * interface.
 */
class MAVLinkAutopilot : public MAVLinkChannel {
 public:
  /**
   * Constructs MAVLinkSerial instance.
   */
  MAVLinkAutopilot();

  /**
   * Initializes connection to the serial device and starts send and receive
   * tasks.
   *
   * @param path serial device path
   * @param speed serial device speed
   * @param devices list of all serial devices
   * @return true if initialization succeeded
   */
  bool init(const std::string& path, int speed,
            const std::vector<std::string>& devices);

  /**
   * Stops send and receive tasks and closes the serial device.
   */
  void close() override;

  /**
   * Returns the path of serial device set by init(...) call.
   */
  inline std::string get_path() const { return serial.get_path(); }

  /**
   * Returns the autopilot system id.
   */
  inline uint8_t get_system_id() const { return system_id; }

  /**
   * Send MAVLink message to autopilot.
   *
   * @param msg MAVLink message
   * @return always returns true
   */
  bool send_message(const mavlink_message_t& msg) override;

  /**
   * Receive MAVLink message from autopilot.
   *
   * @param msg reference to MAVLink message object
   * @return true if MAVLink message was received
   */
  bool receive_message(mavlink_message_t& msg) override;

  /**
   * Returns true if there is a message in the receive queue.
   */
  bool message_available() override;

  /**
   * Returns time of the last successfully sent message.
   */
  std::chrono::milliseconds last_send_time() override;

  /**
   * Returns time of the last successfully received message.
   */
  std::chrono::milliseconds last_receive_time() override;

 private:
  /**
   * Connects to the serial device.
   */
  bool connect(const std::string& path, int speed,
               const std::vector<std::string>& devices);

  /*
   * Checks if MAVLink autopilot is available on the specified serial device.
   *
   * If an autopilot was detected, returns the autopilot's system id,
   * otherwise returns 0.
   */
  uint8_t detect_autopilot(const std::string device);

  /**
   * Sends REQUEST_AUTOPILOT_CAPABILITIES message to the autopilot and
   * reads AUTOPILOT_VERSION message replied by the autopilot.
   *
   * Returns true if AUTOPILOT_VERSION message was received.
   */
  bool request_autopilot_version(
      uint8_t& autopilot, uint8_t& mav_type, uint8_t& sys_id,
      mavlink_autopilot_version_t& autopilot_version);

  /**
   * Retrieves firmware version string from the specified AUTOPILOT_VERSION
   * message.
   *
   * Returns the firmware version string.
   */
  static char* get_firmware_version(
      const mavlink_autopilot_version_t& autopilot_version, char* buff,
      size_t buff_size);

  /**
   * While running is true, retrieves messages from send_queue and sends
   * them to serial.
   */
  void send_task();

  /**
   * While running is true, receives messages from serial and pushes them to
   * receive_queue.
   */
  void receive_task();

  std::atomic<bool> running;
  std::thread send_thread;     // Thread of send_task
  std::thread receive_thread;  // Thread of receive_task
  MAVLinkSerial serial;        // Serial interface
  // Queue that buffers messages sent to autopilot
  CircularBuffer<mavlink_message_t> send_queue;
  // Queue that buffers messages received from autopilot
  CircularBuffer<mavlink_message_t> receive_queue;
  uint8_t system_id;  // Autopilot system Id
  std::chrono::milliseconds send_time;  // Last send epoch time
  std::chrono::milliseconds receive_time;  // Last receive epoch time
};

}  // namespace mavio

#endif  // LIBS_MAVIO_INCLUDE_MAVLINKAUTOPILOT_H_
