/*
 CameraHandler.h

 Telemetry for MAVLink autopilots.

 (C) Copyright 2020 Envirover.

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

#ifndef SRC_CAMERAHANDLER_H_
#define SRC_CAMERAHANDLER_H_

#include "CircularBuffer.h"
#include "MAVLinkChannel.h"
#include "MAVLinkLib.h"

#include <atomic>
#include <string>
#include <thread>
#include <vector>

namespace radioroom {

/**
 * Executes system commands on MAVLink camera protocol messages such as
 * photo and video capture commands.
 */
class CameraHandler : public mavio::MAVLinkChannel {
  /**
   * Executes system command in a dedicated thread on MAVLink command messages.
   */
  class CmdExecutor {
   public:
    CmdExecutor(const std::string name,
                mavio::CircularBuffer<mavlink_message_t>& ack_queue);

    /**
     * Initializes CmdExecutor instance if cmd string is not empty.
     */
    bool init(const std::string cmd);

    /**
     * Stops the thread.
     */
    void close();

    /**
     * Submits MAVLink command message for execution.
     *
     * returns true if the command will be handled by the executor.
     */
    bool submit(const mavlink_message_t& msg);

   private:
    void task();

    void replace_parameters(const mavlink_message_t& msg,
                            std::string& cmd) const;

    void execute_cmd(const mavlink_message_t& msg);

    std::string name;
    std::string command;
    mavio::CircularBuffer<mavlink_message_t>& out_queue;
    mavio::CircularBuffer<mavlink_message_t> in_queue;
    std::thread handler_thread;
    std::atomic<bool> running;
    std::atomic<bool> executing;
  };

 public:
  /**
   * Constructs CameraHandler instance.
   */
  CameraHandler();

  /**
   * Initializes CameraHandler instance.
   */
  bool init();

  /**
   * Coloses CameraHandler instance.
   */
  void close() override;

  /**
   * Send MAVLink message to CameraHandler.
   *
   * @param msg MAVLink message
   * @return returns true if command will be handled by the handler.
   */
  bool send_message(const mavlink_message_t& msg) override;

  /**
   * Receive MAVLink message from CameraHandler.
   *
   * @param msg reference to MAVLink message object
   * @return true if MAVLink message was received.
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
  std::atomic<bool> running;

  // Command executors
  CmdExecutor on_video_start_capture_executor;
  CmdExecutor on_video_stop_capture_executor;
  CmdExecutor on_image_start_capture_executor;
  CmdExecutor on_image_stop_capture_executor;
  CmdExecutor on_do_digicam_control_executor;

  // Queue that buffers messages received from the handlers
  mavio::CircularBuffer<mavlink_message_t> receive_queue;

  mavlink_message_t arm_disarm_cmd;
};

}  // namespace radioroom

#endif  // SRC_CAMERAHANDLER_H_
