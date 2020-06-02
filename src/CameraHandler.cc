/*
 CameraHandler.cc

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

#include "CameraHandler.h"

#include <regex>

#include "Config.h"
#include "MAVLinkLogger.h"
#include "timelib.h"

namespace radioroom {

constexpr size_t max_handler_queue_size = 1;
constexpr size_t max_receive_queue_size = 1024;

const std::chrono::milliseconds executor_sleep_interval(10);

std::string replace(std::string str, const std::string& from,
                    const std::string& to) {
  size_t start_pos = 0;
  while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos +=
        to.length();  // Handles case where 'to' is a substring of 'from'
  }
  return str;
}

CameraHandler::CmdExecutor::CmdExecutor(
    const std::string name, mavio::CircularBuffer<mavlink_message_t>& ack_queue)
    : name(name),
      command(),
      out_queue(ack_queue),
      in_queue(max_handler_queue_size),
      handler_thread(),
      running(false),
      executing(false) {}

bool CameraHandler::CmdExecutor::init(const std::string cmd) {
  if (!running && !cmd.empty()) {
    command = cmd;

    running = true;

    handler_thread = std::thread{&CmdExecutor::task, this};

    mavio::log(LOG_INFO, "Handler '%s' initialized.", name.c_str());

    return true;
  }

  return false;
}

void CameraHandler::CmdExecutor::close() {
  if (running) {
    running = false;

    handler_thread.join();

    mavio::log(LOG_INFO, "Handler '%s' closed.", name.c_str());
  }
}

bool CameraHandler::CmdExecutor::submit(const mavlink_message_t& msg) {
  // Dismiss message if the executor is not initialized or the handler
  // is already being executed.
  if (running && !executing) {
    in_queue.push(msg);
    return true;
  }

  return false;
}

void CameraHandler::CmdExecutor::task() {
  while (running) {
    mavlink_message_t msg;

    if (in_queue.pop(msg)) {
      executing = true;
      execute_cmd(msg);
      executing = false;
    }

    timelib::sleep(executor_sleep_interval);
  }
}

void CameraHandler::CmdExecutor::replace_parameters(
    const mavlink_message_t& msg, std::string& cmd) const {
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    cmd = replace(cmd, "{{param1}}",
                  std::to_string(mavlink_msg_command_long_get_param1(&msg)));
    cmd = replace(cmd, "{{param2}}",
                  std::to_string(mavlink_msg_command_long_get_param2(&msg)));
    cmd = replace(cmd, "{{param3}}",
                  std::to_string(mavlink_msg_command_long_get_param3(&msg)));
    cmd = replace(cmd, "{{param4}}",
                  std::to_string(mavlink_msg_command_long_get_param4(&msg)));
    cmd = replace(cmd, "{{param5}}",
                  std::to_string(mavlink_msg_command_long_get_param5(&msg)));
    cmd = replace(cmd, "{{param6}}",
                  std::to_string(mavlink_msg_command_long_get_param6(&msg)));
    cmd = replace(cmd, "{{param7}}",
                  std::to_string(mavlink_msg_command_long_get_param7(&msg)));
  } else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
    cmd = replace(cmd, "{{param1}}",
                  std::to_string(mavlink_msg_command_int_get_param1(&msg)));
    cmd = replace(cmd, "{{param2}}",
                  std::to_string(mavlink_msg_command_int_get_param2(&msg)));
    cmd = replace(cmd, "{{param3}}",
                  std::to_string(mavlink_msg_command_int_get_param3(&msg)));
    cmd = replace(cmd, "{{param4}}",
                  std::to_string(mavlink_msg_command_int_get_param4(&msg)));
  }
}

void CameraHandler::CmdExecutor::execute_cmd(const mavlink_message_t& msg) {
  mavio::log(LOG_INFO, "Handler '%s' execution started.", name.c_str());

  std::string cmd = command;

  replace_parameters(msg, cmd);

  int ret = ::system(cmd.c_str());

  mavlink_command_ack_t command_ack;

  if (ret < 0) {
    mavio::log(LOG_ERR, "Handler '%s' execution failed. %s", name.c_str(),
               strerror(errno));
    command_ack.result = MAV_RESULT_FAILED;
  } else {
    mavio::log(LOG_INFO, "Handler '%s' execution completed.", name.c_str());
    command_ack.result = MAV_RESULT_ACCEPTED;
  }

  mavlink_message_t ack_msg;
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    command_ack.command = mavlink_msg_command_long_get_command(&msg);
    mavlink_msg_command_ack_encode(
        mavlink_msg_command_long_get_target_system(&msg),
        mavlink_msg_command_long_get_target_component(&msg), &ack_msg,
        &command_ack);
  } else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
    command_ack.command = mavlink_msg_command_int_get_command(&msg);
    mavlink_msg_command_ack_encode(
        mavlink_msg_command_int_get_target_system(&msg),
        mavlink_msg_command_int_get_target_component(&msg), &ack_msg,
        &command_ack);
  }

  if (command_ack.command != MAV_CMD_COMPONENT_ARM_DISARM) {
    out_queue.push(ack_msg);
  }
}

CameraHandler::CameraHandler()
    : MAVLinkChannel("handler"),
      running(false),
      on_armed_executor("on_armed", receive_queue),
      on_disarmed_executor("on_disarmed", receive_queue),
      on_video_start_capture_executor("on_video_start_capture", receive_queue),
      on_video_stop_capture_executor("on_video_stop_capture", receive_queue),
      on_image_start_capture_executor("on_image_start_capture", receive_queue),
      on_image_stop_capture_executor("on_image_stop_capture", receive_queue),
      on_do_digicam_control_executor("on_do_digicam_control", receive_queue),
      receive_queue(max_handler_queue_size) {}

bool CameraHandler::init() {
  if (!running) {
    running = true;

    on_armed_executor.init(config.get_on_armed());
    on_disarmed_executor.init(config.get_on_disarmed());
    on_video_start_capture_executor.init(config.get_on_video_start_capture());
    on_video_stop_capture_executor.init(config.get_on_video_stop_capture());
    on_image_start_capture_executor.init(config.get_on_image_start_capture());
    on_image_stop_capture_executor.init(config.get_on_image_stop_capture());
    on_do_digicam_control_executor.init(config.get_on_do_digicam_control());
  }

  return true;
}

void CameraHandler::close() {
  if (running) {
    running = false;

    on_armed_executor.close();
    on_disarmed_executor.close();
    on_video_start_capture_executor.close();
    on_video_start_capture_executor.close();
    on_image_start_capture_executor.close();
    on_image_stop_capture_executor.close();
    on_do_digicam_control_executor.close();
  }
}

bool CameraHandler::send_message(const mavlink_message_t& msg) {
  uint16_t command;

  // Handle arm/disarm command only if it was accepted by the autopilot.
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
    command = mavlink_msg_command_ack_get_command(&msg);
    uint8_t result = mavlink_msg_command_ack_get_result(&msg);

    if (command == MAV_CMD_COMPONENT_ARM_DISARM &&
        result == MAV_RESULT_ACCEPTED) {
      float arm;

      if (arm_disarm_cmd.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        arm = mavlink_msg_command_long_get_param1(&arm_disarm_cmd);
      } else {
        arm = mavlink_msg_command_int_get_param1(&arm_disarm_cmd);
      }

      if (arm == 0.0) {
        on_disarmed_executor.submit(arm_disarm_cmd);
      } else {
        on_armed_executor.submit(arm_disarm_cmd);
      }

      return true;
    }

    return false;
  }

  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    command = mavlink_msg_command_long_get_command(&msg);
  } else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
    command = mavlink_msg_command_int_get_command(&msg);
  } else {
    return false;
  }

  switch (command) {
    case MAV_CMD_COMPONENT_ARM_DISARM:
      arm_disarm_cmd = msg;
      return false;
    case MAV_CMD_VIDEO_START_CAPTURE:
      return on_video_start_capture_executor.submit(msg);
    case MAV_CMD_VIDEO_STOP_CAPTURE:
      return on_video_stop_capture_executor.submit(msg);
    case MAV_CMD_IMAGE_START_CAPTURE:
      return on_image_start_capture_executor.submit(msg);
    case MAV_CMD_IMAGE_STOP_CAPTURE:
      return on_image_stop_capture_executor.submit(msg);
    case MAV_CMD_DO_DIGICAM_CONTROL:
      return on_do_digicam_control_executor.submit(msg);
  }

  return false;
}

bool CameraHandler::receive_message(mavlink_message_t& msg) {
  return receive_queue.pop(msg);
}

bool CameraHandler::message_available() { return !receive_queue.empty(); }

std::chrono::milliseconds CameraHandler::last_send_time() {
  return std::chrono::milliseconds(0);
}

std::chrono::milliseconds CameraHandler::last_receive_time() {
  return std::chrono::milliseconds(0);
}

}  // namespace radioroom
