/*
 MAVLinkHandler.cc

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

 Created on: Oct 17, 2017
     Author: Pavel Bobov
 */

#include "MAVLinkHandler.h"

#include "Config.h"
#include "MAVLinkLogger.h"

using std::string;
using std::vector;

using mavio::log;
using mavio::MAVLinkChannel;
using mavio::Serial;
using radioroom::Config;

namespace radioroom {

constexpr int autopilot_send_retries = 5;
constexpr uint16_t data_stream_rate = 2;  // Hz

const std::chrono::milliseconds autopilot_send_interval(10);
const std::chrono::milliseconds heartbeat_period(1000);
const std::chrono::milliseconds autopilot_send_retry_timeout(250);

constexpr char hl_report_period_param[] = "HL_REPORT_PERIOD";

MAVLinkHandler::MAVLinkHandler()
    : autopilot(),
      camera_handler(),
      isbd_channel(),
      tcp_channel(),
      heartbeat_timer(),
      primary_report_timer(),
      secondary_report_timer(),
      missions_received(0),
      param_set_param_id(""),
      wp_num(0),
      retry_timer(),
      retry_msg(),
      retry_timeout(0),
      retry_count(0) {}

/**
 * Initializes autopilot and comm channels.
 *
 * Automatically detects the correct serial devices if autopilot and ISBD
 * transceiver do not respond on the devices specified by the configuration
 * properties.
 */
bool MAVLinkHandler::init() {
  if (!config.get_tcp_enabled() && !config.get_isbd_enabled()) {
    log(LOG_ERR, "Invalid configuration: no enabled comm channels.");
    return false;
  }

  vector<string> devices;

  if (config.get_auto_detect_serials()) {
    Serial::get_serial_devices(devices);
  }

  if (!autopilot.init(config.get_autopilot_serial(),
                      config.get_autopilot_serial_speed(), devices)) {
    log(LOG_ERR,
        "UV Radio Room initialization failed: cannot connect to autopilot.");
    return false;
  }

  request_data_streams();

  // Exclude the serial device used by autopilot from the device list used
  // for ISBD transceiver serial device auto-detection.
  for (vector<string>::iterator iter = devices.begin(); iter != devices.end();
       ++iter) {
    if (*iter == autopilot.get_path()) {
      devices.erase(iter);
      break;
    }
  }

  if (config.get_tcp_enabled()) {
    if (tcp_channel.init(config.get_tcp_host(), config.get_tcp_port())) {
      log(LOG_INFO, "TCP channel initialized.");
    } else {
      log(LOG_WARNING, "TCP channel initialization failed.");
    }
  }

  if (config.get_isbd_enabled()) {
    string isbd_serial = config.get_isbd_serial();

    if (isbd_serial == autopilot.get_path() && devices.size() > 0) {
      log(LOG_WARNING,
          "Autopilot detected at serial device '%s' that was assigned "
          "to ISBD transceiver by the configuration settings.",
          autopilot.get_path().data());

      isbd_serial = devices[0];
    }

    if (isbd_channel.init(isbd_serial, config.get_isbd_serial_speed(),
                          devices)) {
      log(LOG_INFO, "ISBD channel initialized.");
    } else {
      log(LOG_WARNING, "ISBD channel initialization failed.");
    }
  }

  if (camera_handler.init()) {
    log(LOG_INFO, "Camera handler initialized.");
  }

  log(LOG_INFO, "UV Radio Room initialization succeeded.");
  return true;
}

/**
 * Closes all opened connections.
 */
void MAVLinkHandler::close() {
  log(LOG_INFO, "Closing TCP channel...");
  tcp_channel.close();

  log(LOG_INFO, "Closing ISBD channel...");
  isbd_channel.close();

  log(LOG_INFO, "Closing autopilot connection...");
  autopilot.close();

  log(LOG_INFO, "Closing camera handler...");
  camera_handler.close();
}

/**
 * One iteration of the message handler.
 *
 * NOTE: Must not use blocking calls.
 */
void MAVLinkHandler::loop() {
  mavlink_message_t msg;

  if (autopilot.receive_message(msg)) {
    handle_mo_message(msg, active_channel());

    switch (msg.msgid) {
      case MAVLINK_MSG_ID_COMMAND_ACK: {
        // Handle commands acknowleged by autopilot.
        camera_handler.send_message(msg);
        break;
      }
        // case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: {
        //   // Request mission item for the reached seq.
        //   reached_item_seq = mavlink_msg_mission_item_reached_get_seq(&msg);

        //   log(LOG_INFO,
        //       "Mission item reached (seq=%d). Requesting the item from "
        //       "autopilot...",
        //       reached_item_seq);

        //   mavlink_mission_request_int_t mission_request_int;
        //   mission_request_int.seq = reached_item_seq;

        //   mavlink_message_t mission_request_int_msg;
        //   mavlink_msg_mission_request_int_encode(
        //       mavio::gcs_system_id, mavio::gcs_component_id,
        //       &mission_request_int_msg, &mission_request_int);

        //   autopilot.send_message(mission_request_int_msg);
        //   set_retry_send_timer(mission_request_int_msg,
        //                        autopilot_send_retry_timeout,
        //                        autopilot_send_retries);
        //   break;
        // }
      case MAVLINK_MSG_ID_MISSION_CURRENT: {
        uint16_t current_wp_num = mavlink_msg_mission_current_get_seq(&msg);

        if (current_wp_num > wp_num) {
          log(LOG_INFO,
              "Current mission item changed to %d. Requesting the reached "
              "items from the autopilot...",
              current_wp_num);

          mavlink_mission_request_int_t mission_request_int;
          mission_request_int.seq = ++wp_num;

          mavlink_message_t mission_request_int_msg;
          mavlink_msg_mission_request_int_encode(
              mavio::gcs_system_id, mavio::gcs_component_id,
              &mission_request_int_msg, &mission_request_int);

          autopilot.send_message(mission_request_int_msg);
          set_retry_send_timer(mission_request_int_msg,
                               autopilot_send_retry_timeout,
                               autopilot_send_retries);
        } else if (current_wp_num < wp_num) {
          log(LOG_INFO,
              "Current mission was reset to %d.",
              current_wp_num);
          wp_num = current_wp_num;
        }

        report.update(msg);
        break;
      }
      case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        uint16_t item_seq = mavlink_msg_mission_item_int_get_seq(&msg);

        if (item_seq == wp_num) {
          cancel_retry_send_timer(MAVLINK_MSG_ID_MISSION_REQUEST_INT);

          log(LOG_INFO,
              "Received the reached mission item (seq=%d) from autopilot.",
              item_seq);

          // Forward command from the reached mission item to the camera handler
          mavlink_command_long_t command_long;
          command_long.command = mavlink_msg_mission_item_int_get_command(&msg);
          command_long.param1 = mavlink_msg_mission_item_int_get_param1(&msg);
          command_long.param2 = mavlink_msg_mission_item_int_get_param2(&msg);
          command_long.param3 = mavlink_msg_mission_item_int_get_param3(&msg);
          command_long.param4 = mavlink_msg_mission_item_int_get_param4(&msg);
          command_long.param5 = mavlink_msg_mission_item_int_get_x(&msg);
          command_long.param6 = mavlink_msg_mission_item_int_get_y(&msg);
          command_long.param7 = mavlink_msg_mission_item_int_get_z(&msg);

          mavlink_message_t command_long_msg;
          mavlink_msg_command_long_encode(autopilot.get_system_id(),
                                          mavio::ardupilot_component_id,
                                          &command_long_msg, &command_long);

          camera_handler.send_message(command_long_msg);
        }
        break;
    }
  }

  if (camera_handler.receive_message(msg)) {
    handle_mo_message(msg, active_channel());
  }

  // Handle messages received from the comm channels
  if (config.get_tcp_enabled()) {
    if (tcp_channel.receive_message(msg)) {
      handle_mt_message(msg, tcp_channel);
    }
  }

  if (config.get_isbd_enabled()) {
    if (isbd_channel.receive_message(msg)) {
      handle_mt_message(msg, isbd_channel);
    }
  }

  send_report();

  send_heartbeat();

  check_retry_send_timer();
}

MAVLinkChannel& MAVLinkHandler::active_channel() {
  if (config.get_tcp_enabled() && !config.get_isbd_enabled()) {
    return tcp_channel;
  }

  if (!config.get_tcp_enabled() && config.get_isbd_enabled()) {
    return isbd_channel;
  }

  // Both TCP and ISBD channels are enabled.
  // Select channel that successfully sent or received message the last.
  std::chrono::milliseconds tcp_last_transmit =
      std::max(tcp_channel.last_send_time(), tcp_channel.last_receive_time());
  std::chrono::milliseconds isbd_last_transmit =
      std::max(isbd_channel.last_send_time(), isbd_channel.last_receive_time());

  if (tcp_last_transmit >= isbd_last_transmit) {
    return tcp_channel;
  }

  return isbd_channel;
}

// Forward ACK messages to the specified channel.
// Use missions array to get mission items on MISSION_REQUEST.
// Pass all other messages to update_report_msg().
void MAVLinkHandler::handle_mo_message(const mavlink_message_t& msg,
                                       MAVLinkChannel& channel) {
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_COMMAND_ACK: {
      channel.send_message(msg);
      break;
    }
    case MAVLINK_MSG_ID_PARAM_VALUE: {
      char param_id[17];
      mavlink_msg_param_value_get_param_id(&msg, param_id);

      // Filter out unsolicited PARAM_VALUE messages.
      // Only send messages that acknowledge received PARAM_SET messages.
      if (strncmp(param_id, param_set_param_id.c_str(), 16) == 0) {
        channel.send_message(msg);
        param_set_param_id = "";
      }

      break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST: {
      uint16_t seq = mavlink_msg_mission_request_get_seq(&msg);

      if (seq < missions_received) {
        autopilot.send_message(missions[seq]);
        set_retry_send_timer(missions[seq], autopilot_send_retry_timeout,
                             autopilot_send_retries);
      } else {
        log(LOG_ERR, "Mission request seq=%d is out of bounds.", seq);
      }

      break;
    }
    case MAVLINK_MSG_ID_MISSION_ACK: {
      cancel_retry_send_timer(MAVLINK_MSG_ID_MISSION_ITEM);

      uint8_t mission_result = mavlink_msg_mission_ack_get_type(&msg);

      if (mission_result == MAV_MISSION_ACCEPTED) {
        log(LOG_INFO, "Mission accepted by autopilot.");
      } else {
        log(LOG_WARNING, "Mission not accepted by autopilot (result=%d).",
            mission_result);
      }

      channel.send_message(msg);
      break;
    }
    default: {
      report.update(msg);
      break;
    }
  }  // switch
}

/**
 * Handles writing waypoints list as described  in
 * http://qgroundcontrol.org/mavlink/waypoint_protocol
 *
 * If message specified by msg parameter is of type MISSION_COUNT,
 * the method retrieves all the mission items from the channel, sends them
 * to the autopilot, and sends MISSION_ACK to the channel. Otherwise the
 * method does nothing and just returns false.
 */
void MAVLinkHandler::handle_mt_message(const mavlink_message_t& msg,
                                       MAVLinkChannel& channel) {
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_MISSION_COUNT: {
      uint16_t mission_count = mavlink_msg_mission_count_get_count(&msg);
      if (mission_count <= max_mission_count) {
        mission_count_msg = msg;
        // Set msgid of all mission items to 0 to flag non-initialized items
        for (size_t i = 0; i < mission_count; i++) {
          missions[i].msgid = 0;
        }
      } else {  // Too many mission items - reject the mission
        mavlink_mission_ack_t mission_ack;
        mission_ack.target_system = msg.sysid;
        mission_ack.target_component = msg.compid;
        mission_ack.type = MAV_MISSION_NO_SPACE;

        mavlink_message_t ack_msg;
        mavlink_msg_mission_ack_encode(autopilot.get_system_id(),
                                       mavio::ardupilot_component_id, &ack_msg,
                                       &mission_ack);
        channel.send_message(ack_msg);
      }

      missions_received = 0;
      break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM: {
      uint16_t mission_count =
          mavlink_msg_mission_count_get_count(&mission_count_msg);
      uint16_t seq = mavlink_msg_mission_item_get_seq(&msg);
      if (seq < mission_count) {
        if (missions[seq].msgid == 0) {  // new item
          missions_received++;
        }

        missions[seq] = msg;

        log(LOG_INFO, "%d mission items received.", missions_received);

        if (missions_received == mission_count) {
          // All mission items received
          log(LOG_INFO, "Sending mission items to autopilot...");
          autopilot.send_message(mission_count_msg);
          set_retry_send_timer(mission_count_msg, autopilot_send_retry_timeout,
                               autopilot_send_retries);
        }
      } else {
        log(LOG_WARNING, "Ignored mission item from rejected mission.");
      }
      break;
    }
    case MAVLINK_MSG_ID_PARAM_SET: {
      char param_id[17];
      mavlink_msg_param_set_get_param_id(&msg, param_id);
      param_set_param_id = param_id;

      if (strncmp(param_id, hl_report_period_param, 16) == 0) {
        float value = mavlink_msg_param_set_get_param_value(&msg);
        config.set_isbd_report_period(value);

        mavlink_param_value_t paramValue;
        paramValue.param_value = value;
        paramValue.param_count = 0;
        paramValue.param_index = 0;
        mavlink_msg_param_set_get_param_id(&msg, paramValue.param_id);
        paramValue.param_type = mavlink_msg_param_set_get_param_type(&msg);

        mavlink_message_t param_value_msg;
        mavlink_msg_param_value_encode(autopilot.get_system_id(),
                                       mavio::ardupilot_component_id,
                                       &param_value_msg, &paramValue);
        channel.send_message(param_value_msg);

        log(LOG_INFO, "Report period changed to %f seconds.", value);
      } else {
        autopilot.send_message(msg);
      }
      break;
    }
    case MAVLINK_MSG_ID_COMMAND_LONG:
      // Send the command to camera handler
      if (!camera_handler.send_message(msg)) {
        // Send the command to autopilot if it was not handled by camera handler
        autopilot.send_message(msg);
      }
      break;
    default: {
      autopilot.send_message(msg);
      break;
    }
  }
}

bool MAVLinkHandler::send_report() {
  std::chrono::milliseconds report_period =
      timelib::sec2ms(config.get_tcp_report_period());

  if (config.get_tcp_enabled() && !config.get_isbd_enabled()) {
    report_period = timelib::sec2ms(config.get_tcp_report_period());
  } else if (!config.get_tcp_enabled() && config.get_isbd_enabled()) {
    report_period = timelib::sec2ms(config.get_isbd_report_period());
  } else if (config.get_tcp_report_period() > config.get_isbd_report_period()) {
    report_period = timelib::sec2ms(config.get_isbd_report_period());
  }

  if (primary_report_timer.elapsed_time() >= report_period) {
    primary_report_timer.reset();

    // if ((report_mask & mavlink_msg_mask_high_latency) !=
    //     mavlink_msg_mask_high_latency) {
    //   log(LOG_WARNING, "Report message is incomplete. Mask = %x.",
    //   report_mask);
    // }

    mavlink_message_t report_msg;
    report.get_message(report_msg);

    // Select the channel to send report
    if (config.get_tcp_enabled() && !config.get_isbd_enabled()) {
      return tcp_channel.send_message(report_msg);
    }

    if (!config.get_tcp_enabled() && config.get_isbd_enabled()) {
      return isbd_channel.send_message(report_msg);
    }

    // Both channels are enabled.
    // Select primary and secondary channels and secondary report period.
    MAVLinkChannel& primary_channel = tcp_channel;
    MAVLinkChannel& secondary_channel = isbd_channel;
    std::chrono::milliseconds secondary_report_period =
        timelib::sec2ms(config.get_isbd_report_period());

    if (config.get_tcp_report_period() > config.get_isbd_report_period()) {
      primary_channel = isbd_channel;
      secondary_channel = tcp_channel;
      secondary_report_period = timelib::sec2ms(config.get_tcp_report_period());
    }

    // Send report to secondary channel if secondary report period elapsed
    // and messages were not successfully sent over the primary channel
    // over that period.
    if (secondary_report_timer.elapsed_time() >= secondary_report_period) {
      if (timelib::time_since_epoch() - primary_channel.last_send_time() >=
          secondary_report_period) {
        secondary_report_timer.reset();
        return secondary_channel.send_message(report_msg);
      }
    }

    return primary_channel.send_message(report_msg);
  }

  return false;
}

bool MAVLinkHandler::send_heartbeat() {
  if (heartbeat_timer.elapsed_time() >= heartbeat_period) {
    heartbeat_timer.reset();

    // Channel is healthy if it is enabled and succesfully sent report or
    // another MO message within its report period times 2.
    std::chrono::milliseconds time = timelib::time_since_epoch();

    bool tcp_healthy = config.get_tcp_enabled() &&
                       (time - tcp_channel.last_send_time()) <=
                           2 * timelib::sec2ms(config.get_tcp_report_period());

    bool isbd_healthy =
        config.get_isbd_enabled() &&
        (time - isbd_channel.last_send_time()) <=
            2 * timelib::sec2ms(config.get_isbd_report_period());

    if (tcp_healthy || isbd_healthy) {
      mavlink_message_t heartbeat_msg;
      mavlink_msg_heartbeat_pack(mavio::gcs_system_id, mavio::gcs_component_id,
                                 &heartbeat_msg, MAV_TYPE_GCS,
                                 MAV_AUTOPILOT_INVALID, 0, 0, 0);
      return autopilot.send_message(heartbeat_msg);
    }
  }

  return false;
}

void MAVLinkHandler::request_data_streams() {
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

  size_t req_stream_count = sizeof(req_stream_ids) / sizeof(req_stream_ids[0]);

  for (size_t i = 0; i < req_stream_count; ++i) {
    mavlink_msg_request_data_stream_pack(
        mavio::gcs_system_id, mavio::gcs_component_id, &mt_msg,
        autopilot.get_system_id(), mavio::ardupilot_component_id,
        req_stream_ids[i], data_stream_rate, 1);

    autopilot.send_message(mt_msg);

    timelib::sleep(autopilot_send_interval);
  }
}

void MAVLinkHandler::set_retry_send_timer(
    const mavlink_message_t& msg, const std::chrono::milliseconds& timeout,
    int retries) {
  retry_timer.reset();
  retry_msg = msg;
  retry_timeout = timeout;
  retry_count = retries;
}

void MAVLinkHandler::cancel_retry_send_timer(int msgid) {
  if (retry_msg.msgid == msgid) {
    retry_count = 0;
  }
}

void MAVLinkHandler::check_retry_send_timer() {
  if (retry_count > 0 && retry_timer.elapsed_time() >= retry_timeout) {
    autopilot.send_message(retry_msg);
    retry_count--;
    retry_timer.reset();
  }
}

}  // namespace radioroom
