/*
 MAVLinkHandler.cc

BVLOS telemetry for MAVLink autopilots.

 (C) Copyright 2018 Envirover.

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

#include "MAVLinkLogger.h"
#include <unistd.h>
#include <syslog.h>
#include <vector>
#include <algorithm>

#define AUTOPILOT_SEND_INTERVAL 10000   //microseconds
#define ISBD_RETRY_INTERVAL     5000000 //microseconds
#define TCP_RETRY_INTERVAL      5000000 //microseconds

#define MAX_SEND_RETRIES   5

// Masks of MAVLink messages used to compose single HIGH_LATENCY message
#define MAVLINK_MSG_MASK_HEARTBEAT              0x0001
#define MAVLINK_MSG_MASK_SYS_STATUS             0x0002
#define MAVLINK_MSG_MASK_GPS_RAW_INT            0x0004
#define MAVLINK_MSG_MASK_ATTITUDE               0x0008
#define MAVLINK_MSG_MASK_GLOBAL_POSITION_INT    0x0010
#define MAVLINK_MSG_MASK_MISSION_CURRENT        0x0020
#define MAVLINK_MSG_MASK_NAV_CONTROLLER_OUTPUT  0x0040
#define MAVLINK_MSG_MASK_VFR_HUD                0x0080
#define MAVLINK_MSG_MASK_BATTERY2               0x0100 // optional

#define MAVLINK_MSG_MASK_HIGH_LATENCY           0x00FF

#define DATA_STREAM_RATE   2 //Hz

inline int16_t radToCentidegrees(float rad) {
  return rad / M_PI * 18000;
}

bool missions_comp(mavlink_message_t msg1, mavlink_message_t msg2)
{
    int seq1 = mavlink_msg_mission_item_get_seq(&msg1);
    int seq2 = mavlink_msg_mission_item_get_seq(&msg2);
    return seq1 < seq2;
}

MAVLinkHandler::MAVLinkHandler() :
    autopilot(), isbd_channel(), tcp_channel(), report_time()
{
}

/**
 * Handles HL_REPORT_PERIOD_PARAM parameter setting.
 */
bool MAVLinkHandler::handle_param_set(const mavlink_message_t& msg, mavlink_message_t& ack)
{
    if (msg.msgid != MAVLINK_MSG_ID_PARAM_SET)
        return false;

    char param_id[17];
    mavlink_msg_param_set_get_param_id(&msg, param_id);

    if (strncmp(param_id, HL_REPORT_PERIOD_PARAM, 16) == 0) {
        float value = mavlink_msg_param_set_get_param_value(&msg);
        config.set_isbd_report_period(value);
        config.set_tcp_report_period(value);

        mavlink_param_value_t paramValue;
        paramValue.param_value = value;
        paramValue.param_count = 0;
        paramValue.param_index = 0;
        mavlink_msg_param_set_get_param_id(&msg, paramValue.param_id);
        paramValue.param_type = mavlink_msg_param_set_get_param_type(&msg);

        mavlink_msg_param_value_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &paramValue);

        syslog(LOG_INFO, "Report period changed to %f seconds.", config.get_isbd_report_period());
        return true;
    } else {
        return autopilot.send_receive_message(msg, ack);
    }

    return false;
}

/**
 * Handles mission write transaction.
 */
bool MAVLinkHandler::handle_mission_write(MAVLinkChannel& channel, const mavlink_message_t& msg, mavlink_message_t& ack)
{
    if (msg.msgid != MAVLINK_MSG_ID_MISSION_COUNT)
        return false;

    uint16_t count = mavlink_msg_mission_count_get_count(&msg);

    vector<mavlink_message_t> missions(count);

    mavlink_message_t mt_msg;

    syslog(LOG_INFO, "Receiving %d mission items from the comm link.", count);

    uint16_t idx = 0;

    for (uint16_t i = 0; i < count * MAX_SEND_RETRIES && idx < count; i++) {
        if (channel.receive_message(mt_msg)) {
            if (mt_msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM) {
                //syslog(LOG_DEBUG, "MISSION_ITEM MT message received.");
                missions[idx++] = mt_msg;
            }
        } else {
            usleep(ISBD_RETRY_INTERVAL);
        }
    }

    if (idx != count) {
        syslog(LOG_WARNING, "Not all mission items received.");

        mavlink_mission_ack_t mission_ack;
        mission_ack.target_system = msg.sysid;
        mission_ack.target_component = msg.compid;
        mission_ack.type = MAV_MISSION_ERROR;
        mavlink_msg_mission_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &mission_ack);

        return true;
    }

    std::sort(missions.begin(), missions.end(), missions_comp);

    return send_missions_to_autopilot(msg, missions, ack);
}

/**
 * Sends the specified missions to autopilot.
 */
bool MAVLinkHandler::send_missions_to_autopilot(const mavlink_message_t& mission_count, const vector<mavlink_message_t>& missions, mavlink_message_t& ack)
{
    if (mission_count.msgid != MAVLINK_MSG_ID_MISSION_COUNT)
        return false;

    syslog(LOG_INFO, "Sending mission items to autopilot...");

    for (int i = 0; i < MAX_SEND_RETRIES; i++) {
        if (autopilot.send_message(mission_count)) {
            break;
        }

        usleep(AUTOPILOT_SEND_INTERVAL);
    }

    for (uint16_t i = 0; i < missions.size(); i++) {
        autopilot.send_receive_message(missions[i], ack);

        usleep(AUTOPILOT_SEND_INTERVAL);
    }

    if (mavlink_msg_mission_ack_get_type(&ack) == MAV_MISSION_ACCEPTED) {
        syslog(LOG_INFO, "Missions accepted by autopilot.");
    } else {
        syslog(LOG_WARNING, "Missions not accepted by autopilot: %d", mavlink_msg_mission_ack_get_type(&ack));
    }

    return true;
}

/**
 * Send the specified mo_msg to the specified channel.
 * Receive and handle all messages waiting in the MT queue.
 * Send ACKs for received messages from autopilot to ISBD.
 */
bool MAVLinkHandler::comm_session(MAVLinkChannel& channel, mavlink_message_t& mo_msg)
{
    syslog(LOG_INFO, "Comm session started for %s channel.", channel.get_channel_id().data());

    if (!channel.send_message(mo_msg)) {
        return false;
    }

    while (channel.message_available()) {
        mavlink_message_t mt_msg;

        if (channel.receive_message(mt_msg)) {
            mo_msg.len = mo_msg.msgid = 0;
            bool ack_received = false;

            switch(mt_msg.msgid) {
                case MAVLINK_MSG_ID_PARAM_SET:
                    ack_received = handle_param_set(mt_msg, mo_msg);
                    break;
                case MAVLINK_MSG_ID_MISSION_COUNT:
                    ack_received = handle_mission_write(channel, mt_msg, mo_msg);
                    break;
                default:
                    /*
                     * Send a heartbeat first
                     */
                    mavlink_message_t heartbeat;
                    mavlink_msg_heartbeat_pack(mt_msg.sysid, mt_msg.compid, &heartbeat, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
                    autopilot.send_message(heartbeat);

                    //Forward unhandled messages to the autopilot.
                    ack_received = autopilot.send_receive_message(mt_msg, mo_msg);
            }

            if (ack_received) {
                channel.send_message(mo_msg);
            }
        }
    }

    syslog(LOG_INFO, "Comm session ended.");

    return true;
}

/**
 * Initializes autopilot and comm channels.
 *
 * Automatically detect the correct serial devices if autopilot and ISBD transceiver
 * do not respond on the devices specified by the configuration properties.
 *
 */
bool MAVLinkHandler::init()
{
    vector<string> devices;

    if (config.get_auto_detect_serials()) {
        Serial::get_serial_devices(devices);
    }

    if (!autopilot.init(config.get_autopilot_serial(),  config.get_autopilot_serial_speed(), devices)) {
        return false;
    }

    // Exclude the serial device used by autopilot from the device list used
    // for ISBD transceiver serial device auto-detection.
    for (std::vector<string>::iterator iter = devices.begin(); iter != devices.end(); ++iter) {
        if (*iter == autopilot.get_path()) {
            devices.erase(iter);
            break;
        }
    }

    if (config.get_tcp_enabled() && !tcp_channel.init(config.get_tcp_host(), config.get_tcp_port())) {
        return false;
    }

    if (config.get_isbd_enabled()) {
        string isbd_serial = config.get_isbd_serial();

        if (isbd_serial == autopilot.get_path() && devices.size() > 0) {
            syslog(LOG_WARNING,
                   "Autopilot detected at serial device '%s' that was assigned to ISBD transceiver by the configuration settings.",
                   autopilot.get_path().data());

            isbd_serial = devices[0];
        }

        if (!isbd_channel.init(isbd_serial, config.get_isbd_serial_speed(), devices)) {
            return false;
        }
    }

    if (!config.get_tcp_enabled() && !config.get_isbd_enabled()) {
        syslog(LOG_ERR, "Invalid configuration: no enabled comm channels.");
        return false;
    }

    request_data_streams();

    return true;
}

/**
 * Closes all opened connections.
 */
void MAVLinkHandler::close()
{
    tcp_channel.close();
    isbd_channel.close();
    autopilot.close();
}

/**
 * One iteration of the message handler.
 */
void MAVLinkHandler::loop()
{
    if (config.get_tcp_report_period() <= config.get_isbd_report_period()) {
        tcp_loop();
        isbd_loop();
    } else {
        isbd_loop();
        tcp_loop();
    }
}

// Start TCP comm session first if the TCP channel is enabled and
// either data is available to receive or TCP report period is elapsed.
void MAVLinkHandler::tcp_loop() {
    if (!config.get_tcp_enabled()) {
        return;
    }

    bool message_available = tcp_channel.message_available();

    if (message_available) {
        mavlink_message_t msg;
        msg.len   = 0;
        msg.msgid = 0;

        comm_session(tcp_channel, msg);
    }

    if (message_available || report_time.elapsed_time() >= config.get_tcp_report_period()) {
        high_resolution_clock::time_point period_start_time = report_time.time();

        mavlink_message_t msg;

        get_high_latency_msg(msg);

        if (comm_session(tcp_channel, msg)) {
            // Reset the stopwatch if the comm session succeeded.
            report_time.reset(period_start_time);
        }
    }
}

/*
 * Start ISBD comm session if the ISBD channel is enabled and
 * either data is available to receive or ISBD report period is elapsed.
 * Typically, configuration ISBD report period should be greater than
 * TCP report period, so ISBD report period will elapse only when
 * TCP sessions failed.
 */
void MAVLinkHandler::isbd_loop() {
    if (!config.get_isbd_enabled()) {
        return;
    }

    bool message_available = isbd_channel.message_available();

    if (message_available) {
        mavlink_message_t msg;
        msg.len   = 0;
        msg.msgid = 0;
        comm_session(isbd_channel, msg);
    }

    if (message_available || report_time.elapsed_time() >= config.get_isbd_report_period()) {
        high_resolution_clock::time_point period_start_time = report_time.time();

        mavlink_message_t msg;

        get_high_latency_msg(msg);

        if (comm_session(isbd_channel, msg)) {
            // Reset the stopwatch if the comm session succeeded.
            report_time.reset(period_start_time);
        }
    }
}

void MAVLinkHandler::request_data_streams()
{
    mavlink_message_t mt_msg;

    /*
     * Send a heartbeat first
     */
    mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &mt_msg, MAV_TYPE_GCS,
                               MAV_AUTOPILOT_INVALID, 0, 0, 0);
    autopilot.send_message(mt_msg);

    /*
     * Request data streams from the autopilot.
     */
    uint8_t req_stream_ids[] = {
        MAV_DATA_STREAM_EXTENDED_STATUS, // SYS_STATUS, NAV_CONTROLLER_OUTPUT, GPS_RAW, MISSION_CURRENT
        MAV_DATA_STREAM_POSITION,        // GLOBAL_POSITION_INT
        //MAV_DATA_STREAM_RAW_CONTROLLER,
        MAV_DATA_STREAM_RAW_SENSORS,     //
        MAV_DATA_STREAM_EXTRA1,          // ATTITUDE
        MAV_DATA_STREAM_EXTRA2,          // VFR_HUD
        MAV_DATA_STREAM_EXTRA3           // MSG_BATTERY2
    };

    for (size_t i = 0; i < sizeof(req_stream_ids)/sizeof(req_stream_ids[0]); i++) {
        mavlink_msg_request_data_stream_pack(SYSTEM_ID, COMPONENT_ID, &mt_msg,
                                             1, 1, req_stream_ids[i], DATA_STREAM_RATE, 1);

        autopilot.send_message(mt_msg);

        usleep(AUTOPILOT_SEND_INTERVAL);
    }
}

/*
 * Retrieves MAVLink messages from autopilot and composes a HIGH_LATENCY message from them.
 */
void MAVLinkHandler::get_high_latency_msg(mavlink_message_t& msg)
{
    syslog(LOG_INFO, "Preparing HIGH_LATENCY message...");

    /**
     * Reads and processes MAVLink messages from autopilot.
     */
    mavlink_high_latency_t high_latency;
    uint16_t mask = 0;

    memset(&high_latency, 0, sizeof(high_latency));

    // Rate of HEARTBEAT message cannot be requested. Usually it's 1 Hz.
    // So at least 1 second is required to compose HIGH_LATENCY message.
    double data_stream_period = 1.0;

    Stopwatch stopwatch;

    for (int i = 0; stopwatch.elapsed_time() < data_stream_period; i++) {
        mavlink_message_t mt_msg;

        if (autopilot.receive_message(mt_msg)) {
            update_high_latency_msg(mt_msg, high_latency, mask);
        }
    }

    if ((mask & MAVLINK_MSG_MASK_HIGH_LATENCY) == MAVLINK_MSG_MASK_HIGH_LATENCY) {
      syslog(LOG_INFO, "HIGH_LATENCY message prepared.");
    } else {
      syslog(LOG_WARNING, "HIGH_LATENCY message is incomplete. Mask = %x.", mask);
    }

    mavlink_msg_high_latency_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &msg, &high_latency);
}

/**
 * Integrates data from the specified MAVLink message into the HIGH_LATENCY message.
 */
bool MAVLinkHandler::update_high_latency_msg(const mavlink_message_t& msg, mavlink_high_latency_t& high_latency, uint16_t& mask)
{
  switch (msg.msgid) {
  case MAVLINK_MSG_ID_HEARTBEAT:    //0
    high_latency.base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
    high_latency.custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
    mask |= MAVLINK_MSG_MASK_HEARTBEAT;
    return true;
  case MAVLINK_MSG_ID_SYS_STATUS:   //1
    high_latency.battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg);
    high_latency.temperature = mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000;
    mask |= MAVLINK_MSG_MASK_SYS_STATUS;
    return true;
  case MAVLINK_MSG_ID_GPS_RAW_INT:    //24
    //high_latency.latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
    //high_latency.longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
    //high_latency.altitude_amsl = mavlink_msg_gps_raw_int_get_alt(&msg) / 1000;
    high_latency.groundspeed = mavlink_msg_gps_raw_int_get_vel(&msg) / 100;
    high_latency.gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
    high_latency.gps_nsat = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
    mask |= MAVLINK_MSG_MASK_GPS_RAW_INT;
    return true;
  case MAVLINK_MSG_ID_ATTITUDE:   //30
    high_latency.heading = (radToCentidegrees(mavlink_msg_attitude_get_yaw(&msg)) + 36000) % 36000;
    high_latency.roll = radToCentidegrees(mavlink_msg_attitude_get_roll(&msg));
    high_latency.pitch = radToCentidegrees(mavlink_msg_attitude_get_pitch(&msg));
    mask |= MAVLINK_MSG_MASK_ATTITUDE;
    return true;
  case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:    //33
    high_latency.latitude = mavlink_msg_global_position_int_get_lat(&msg);
    high_latency.longitude = mavlink_msg_global_position_int_get_lon(&msg);
    high_latency.altitude_amsl = mavlink_msg_global_position_int_get_alt(&msg) / 1000;
    high_latency.altitude_sp = mavlink_msg_global_position_int_get_relative_alt(&msg) / 1000;
    mask |= MAVLINK_MSG_MASK_GLOBAL_POSITION_INT;
    return true;
  case MAVLINK_MSG_ID_MISSION_CURRENT:    //42
    high_latency.wp_num = mavlink_msg_mission_current_get_seq(&msg);
    mask |= MAVLINK_MSG_MASK_MISSION_CURRENT;
    return true;
  case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:    //62
    high_latency.wp_distance = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
    high_latency.heading_sp = mavlink_msg_nav_controller_output_get_nav_bearing(&msg) * 100;
    mask |= MAVLINK_MSG_MASK_NAV_CONTROLLER_OUTPUT;
    return true;
  case MAVLINK_MSG_ID_VFR_HUD:    //74
    high_latency.airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
    high_latency.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
    //high_latency.heading = mavlink_msg_vfr_hud_get_heading(&msg) * 100;
    high_latency.climb_rate = mavlink_msg_vfr_hud_get_climb(&msg);
    high_latency.throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
    mask |= MAVLINK_MSG_MASK_VFR_HUD;
    return true;
  case MAVLINK_MSG_ID_BATTERY2:    //147
    uint16_t batt2_voltage = mavlink_msg_battery2_get_voltage(&msg);
    high_latency.temperature_air = batt2_voltage / 1000;
    mask |= MAVLINK_MSG_MASK_BATTERY2;
    return true;
  }

  return false;
}
