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

#include "Config.h"
#include "MAVLinkHandler.h"
#include "MAVLinkLogger.h"
#include <syslog.h>

using namespace std;
using namespace std::chrono;
using namespace mavio;

constexpr int      MAX_SEND_RETRIES = 5;
constexpr uint16_t DATA_STREAM_RATE = 2; //Hz

constexpr struct timespec autopilot_send_interval[] = { { 0, 10000000L } }; // 10 milliseconds

constexpr double heartbeat_period = 1.0; // seconds

// Masks of MAVLink messages used to compose single HIGH_LATENCY message
constexpr uint16_t MAVLINK_MSG_MASK_HEARTBEAT             = 0x0001;
constexpr uint16_t MAVLINK_MSG_MASK_SYS_STATUS            = 0x0002;
constexpr uint16_t MAVLINK_MSG_MASK_GPS_RAW_INT           = 0x0004;
constexpr uint16_t MAVLINK_MSG_MASK_ATTITUDE              = 0x0008;
constexpr uint16_t MAVLINK_MSG_MASK_GLOBAL_POSITION_INT   = 0x0010;
constexpr uint16_t MAVLINK_MSG_MASK_MISSION_CURRENT       = 0x0020;
constexpr uint16_t MAVLINK_MSG_MASK_NAV_CONTROLLER_OUTPUT = 0x0040;
constexpr uint16_t MAVLINK_MSG_MASK_VFR_HUD               = 0x0080;
constexpr uint16_t MAVLINK_MSG_MASK_BATTERY2              = 0x0100; // optional

constexpr uint16_t MAVLINK_MSG_MASK_HIGH_LATENCY = 0x00FF;

constexpr char hl_report_period_param[] = "HL_REPORT_PERIOD";

extern Config config;

inline int16_t rad_to_centidegrees(float rad)
{
    return rad * 18000.0 / 3.14159265358979323846;
}

// Returns elapsed time in seconds after the specified time
inline double elapsed_time(high_resolution_clock::time_point time)
{
    return duration_cast<milliseconds>(high_resolution_clock::now() - time).count() / 1000.0;
}

MAVLinkHandler::MAVLinkHandler() : autopilot(), isbd_channel(), tcp_channel(),
                                   heartbeat_timer(),
                                   primary_report_timer(), secondary_report_timer(),
                                   report_mask(0), missions_received(0)
{
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
    if (!config.get_tcp_enabled() && !config.get_isbd_enabled()) {
        syslog(LOG_ERR, "Invalid configuration: no enabled comm channels.");
        return false;
    }

    vector<string> devices;

    if (config.get_auto_detect_serials()) {
        Serial::get_serial_devices(devices);
    }

    if (!autopilot.init(config.get_autopilot_serial(), config.get_autopilot_serial_speed(), devices)) {
        syslog(LOG_ERR, "UV Radio Room initialization failed: cannot connect to autopilot.");
        return false;
    }

    request_data_streams();

    // Exclude the serial device used by autopilot from the device list used
    // for ISBD transceiver serial device auto-detection.
    for (std::vector<string>::iterator iter = devices.begin(); iter != devices.end(); ++iter) {
        if (*iter == autopilot.get_path()) {
            devices.erase(iter);
            break;
        }
    }

    if (config.get_tcp_enabled()) {
        if (tcp_channel.init(config.get_tcp_host(), config.get_tcp_port())) {
            syslog(LOG_INFO, "TCP channel initialized.");
        } else {
            syslog(LOG_WARNING, "TCP channel initialization failed.");
        }
    }

    if (config.get_isbd_enabled()) {
        string isbd_serial = config.get_isbd_serial();

        if (isbd_serial == autopilot.get_path() && devices.size() > 0) {
            syslog(LOG_WARNING,
                "Autopilot detected at serial device '%s' that was assigned to ISBD transceiver by the configuration settings.",
                autopilot.get_path().data());

            isbd_serial = devices[0];
        }

        if (isbd_channel.init(isbd_serial, config.get_isbd_serial_speed(), devices)) {
            syslog(LOG_INFO, "ISBD channel initialized.");
        } else {
            syslog(LOG_WARNING, "ISBD channel initialization failed.");
        }
    }

    syslog(LOG_INFO, "UV Radio Room initialization succeeded.");
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
 * 
 * NOTE: Must not use blocking calls.
 */
void MAVLinkHandler::loop()
{
    MAVLinkChannel&   channel = active_channel();
    mavlink_message_t msg;

    if (autopilot.receive_message(msg)) {
        handle_mo_message(msg, channel);
    }

    // Handle messages received from the comm channels
    if (config.get_tcp_enabled()) {
        if (tcp_channel.receive_message(msg)) {
            handle_mt_message(msg, channel);
        }
    }

    if (config.get_isbd_enabled()) {
        if (isbd_channel.receive_message(msg)) {
            handle_mt_message(msg, channel);
        }
    }

    send_report();

    send_heartbeat();
}

MAVLinkChannel& MAVLinkHandler::active_channel()
{
    if (config.get_tcp_enabled() && !config.get_isbd_enabled()) {
        return tcp_channel;
    }

    if (!config.get_tcp_enabled() && config.get_isbd_enabled()) {
        return isbd_channel;
    }

    // Both TCP and ISBD channels are enabled.
    // Select channel that successfully sent or received message the last.
    high_resolution_clock::time_point tcp_last_transmit  = max<high_resolution_clock::time_point>(tcp_channel.last_send_time(),
        tcp_channel.last_receive_time());
    high_resolution_clock::time_point isbd_last_transmit = max<high_resolution_clock::time_point>(isbd_channel.last_send_time(),
        isbd_channel.last_receive_time());

    if (tcp_last_transmit >= isbd_last_transmit) {
        return tcp_channel;
    }

    return isbd_channel;
}

void MAVLinkHandler::handle_mo_message(const mavlink_message_t& msg, MAVLinkChannel& channel)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_COMMAND_ACK:
    case MAVLINK_MSG_ID_PARAM_VALUE: {
        channel.send_message(msg);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST: {
        uint16_t seq = mavlink_msg_mission_request_get_seq(&msg);

        if (0 < seq && seq <= missions_received) {
            autopilot.send_message(missions[seq - 1]);
        } else {
            syslog(LOG_ERR, "Mission request seq=%d is out of bounds.", seq);
        }

        break;
    }
    case MAVLINK_MSG_ID_MISSION_ACK: {
        uint8_t mission_result = mavlink_msg_mission_ack_get_type(&msg);

        if (mission_result == MAV_MISSION_ACCEPTED) {
            syslog(LOG_INFO, "Mission accepted by autopilot.");
        } else {
            syslog(LOG_WARNING, "Mission not accepted by autopilot (result=%d).", mission_result);
        }

        channel.send_message(msg);
        break;
    }
    default: {
        update_report_msg(msg);
        break;
    }
    } // switch
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
void MAVLinkHandler::handle_mt_message(const mavlink_message_t& msg, MAVLinkChannel& channel)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_MISSION_COUNT: {
        uint16_t mission_count = mavlink_msg_mission_count_get_count(&msg);
        if (mission_count <= max_mission_count) {
            mission_count_msg = msg;
            // Set seq of all mission items to 0 to flag non-initialized items
            for (size_t i = 0; i < mission_count; i++) {
                missions[i].seq = 0;
            }
        } else { // Too many mission items - reject the mission
            mavlink_mission_ack_t mission_ack;
            mission_ack.target_system    = msg.sysid;
            mission_ack.target_component = msg.compid;
            mission_ack.type             = MAV_MISSION_NO_SPACE;

            mavlink_message_t ack_msg;
            mavlink_msg_mission_ack_encode(autopilot.get_system_id(), ARDUPILOT_COMPONENT_ID, &ack_msg, &mission_ack);
            channel.send_message(ack_msg);
        }

        missions_received = 0;
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM: {
        uint16_t mission_count = mavlink_msg_mission_count_get_count(&mission_count_msg);
        uint16_t seq           = mavlink_msg_mission_item_get_seq(&msg);
        if (0 < seq && seq <= mission_count) {
            if (missions[seq - 1].seq == 0) { // new item
                missions_received++;
            }

            missions[seq - 1] = msg;

            if (missions_received == mission_count) {
                // All mission items received
                syslog(LOG_INFO, "Sending mission items to autopilot...");
                autopilot.send_message(mission_count_msg);
            }
        } else {
            syslog(LOG_WARNING, "Ignored mission item from rejected mission.");
        }
        break;
    }
    case MAVLINK_MSG_ID_PARAM_SET: {
        char param_id[17];
        mavlink_msg_param_set_get_param_id(&msg, param_id);

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
            mavlink_msg_param_value_encode(autopilot.get_system_id(), ARDUPILOT_COMPONENT_ID, &param_value_msg, &paramValue);
            channel.send_message(param_value_msg);

            syslog(LOG_INFO, "Report period changed to %f seconds.", value);
            break;
        } else {
            autopilot.send_message(msg);
        }
    }
    default: {
        autopilot.send_message(msg);
        break;
    }
    }
}

bool MAVLinkHandler::send_report()
{
    double report_period = config.get_tcp_report_period();

    if (config.get_tcp_enabled() && !config.get_isbd_enabled()) {
        report_period = config.get_tcp_report_period();
    } else if (!config.get_tcp_enabled() && config.get_isbd_enabled()) {
        report_period = config.get_isbd_report_period();
    } else if (config.get_tcp_report_period() > config.get_isbd_report_period()) {
        report_period = config.get_isbd_report_period();
    }

    if (primary_report_timer.elapsed_time() >= report_period) {
        primary_report_timer.reset();

        if ((report_mask & MAVLINK_MSG_MASK_HIGH_LATENCY) != MAVLINK_MSG_MASK_HIGH_LATENCY) {
            syslog(LOG_WARNING, "Report message is incomplete. Mask = %x.", report_mask);
        }

        mavlink_message_t report_msg;
        mavlink_msg_high_latency_encode(autopilot.get_system_id(), ARDUPILOT_COMPONENT_ID, &report_msg, &report);

        // Select the channel to send report
        if (config.get_tcp_enabled() && !config.get_isbd_enabled()) {
            return tcp_channel.send_message(report_msg);
        }

        if (!config.get_tcp_enabled() && config.get_isbd_enabled()) {
            return isbd_channel.send_message(report_msg);
        }

        // Both channels are enabled.
        // Select primary and secondary channels and secondary report period.
        MAVLinkChannel& primary_channel         = tcp_channel;
        MAVLinkChannel& secondary_channel       = isbd_channel;
        double          secondary_report_period = config.get_isbd_report_period();

        if (config.get_tcp_report_period() > config.get_isbd_report_period()) {
            primary_channel         = isbd_channel;
            secondary_channel       = tcp_channel;
            secondary_report_period = config.get_tcp_report_period();
        }

        // Send report to secondary channel if secondary report period elapsed
        // and messages were not successfully sent over the primary channel
        // over that period.
        if (secondary_report_timer.elapsed_time() >= secondary_report_period) {
            if (elapsed_time(primary_channel.last_send_time()) >= secondary_report_period) {
                secondary_report_timer.reset();
                return secondary_channel.send_message(report_msg);
            }
        }

        return primary_channel.send_message(report_msg);
    }

    return false;
}

bool MAVLinkHandler::send_heartbeat()
{
    if (heartbeat_timer.elapsed_time() >= heartbeat_period) {
        heartbeat_timer.reset();

        // Channel is healthy if it is enabled and succesfully sent report or
        // another MO message within its report period times 2.
        bool tcp_healthy  = config.get_tcp_enabled() && (elapsed_time(tcp_channel.last_send_time()) <= 2 * config.get_tcp_report_period());
        bool isbd_healthy = config.get_isbd_enabled() && (elapsed_time(isbd_channel.last_send_time()) <= 2 * config.get_isbd_report_period());

        if (tcp_healthy || isbd_healthy) {
            mavlink_message_t heartbeat_msg;
            mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &heartbeat_msg,
                MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
            return autopilot.send_message(heartbeat_msg);
        }
    }

    return false;
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
        MAV_DATA_STREAM_POSITION, // GLOBAL_POSITION_INT
        //MAV_DATA_STREAM_RAW_CONTROLLER,
        MAV_DATA_STREAM_RAW_SENSORS, //
        MAV_DATA_STREAM_EXTRA1, // ATTITUDE
        MAV_DATA_STREAM_EXTRA2, // VFR_HUD
        MAV_DATA_STREAM_EXTRA3 // MSG_BATTERY2
    };

    for (size_t i = 0; i < sizeof(req_stream_ids) / sizeof(req_stream_ids[0]); i++) {
        mavlink_msg_request_data_stream_pack(SYSTEM_ID, COMPONENT_ID, &mt_msg,
            autopilot.get_system_id(), ARDUPILOT_COMPONENT_ID,
            req_stream_ids[i], DATA_STREAM_RATE, 1);

        autopilot.send_message(mt_msg);

        nanosleep(autopilot_send_interval, NULL);
    }
}

/**
 * Integrates data from the specified MAVLink message into the HIGH_LATENCY message.
 */
bool MAVLinkHandler::update_report_msg(const mavlink_message_t& msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: //0
        report.base_mode   = mavlink_msg_heartbeat_get_base_mode(&msg);
        report.custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
        report_mask |= MAVLINK_MSG_MASK_HEARTBEAT;
        return true;
    case MAVLINK_MSG_ID_SYS_STATUS: //1
        report.battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg);
        report.temperature       = mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000;
        report_mask |= MAVLINK_MSG_MASK_SYS_STATUS;
        return true;
    case MAVLINK_MSG_ID_GPS_RAW_INT: //24
        //report_msg.latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
        //report_msg.longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
        //report_msg.altitude_amsl = mavlink_msg_gps_raw_int_get_alt(&msg) / 1000;
        report.groundspeed  = mavlink_msg_gps_raw_int_get_vel(&msg) / 100;
        report.gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
        report.gps_nsat     = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
        report_mask |= MAVLINK_MSG_MASK_GPS_RAW_INT;
        return true;
    case MAVLINK_MSG_ID_ATTITUDE: //30
        report.heading = (rad_to_centidegrees(mavlink_msg_attitude_get_yaw(&msg)) + 36000) % 36000;
        report.roll    = rad_to_centidegrees(mavlink_msg_attitude_get_roll(&msg));
        report.pitch   = rad_to_centidegrees(mavlink_msg_attitude_get_pitch(&msg));
        report_mask |= MAVLINK_MSG_MASK_ATTITUDE;
        return true;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: //33
        report.latitude      = mavlink_msg_global_position_int_get_lat(&msg);
        report.longitude     = mavlink_msg_global_position_int_get_lon(&msg);
        report.altitude_amsl = mavlink_msg_global_position_int_get_alt(&msg) / 1000;
        report.altitude_sp   = mavlink_msg_global_position_int_get_relative_alt(&msg) / 1000;
        report_mask |= MAVLINK_MSG_MASK_GLOBAL_POSITION_INT;
        return true;
    case MAVLINK_MSG_ID_MISSION_CURRENT: //42
        report.wp_num = mavlink_msg_mission_current_get_seq(&msg);
        report_mask |= MAVLINK_MSG_MASK_MISSION_CURRENT;
        return true;
    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: //62
        report.wp_distance = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
        report.heading_sp  = mavlink_msg_nav_controller_output_get_nav_bearing(&msg) * 100;
        report_mask |= MAVLINK_MSG_MASK_NAV_CONTROLLER_OUTPUT;
        return true;
    case MAVLINK_MSG_ID_VFR_HUD: //74
        report.airspeed    = mavlink_msg_vfr_hud_get_airspeed(&msg);
        report.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
        //high_latency.heading = mavlink_msg_vfr_hud_get_heading(&msg) * 100;
        report.climb_rate = mavlink_msg_vfr_hud_get_climb(&msg);
        report.throttle   = mavlink_msg_vfr_hud_get_throttle(&msg);
        report_mask |= MAVLINK_MSG_MASK_VFR_HUD;
        return true;
    case MAVLINK_MSG_ID_BATTERY2: //147
        uint16_t batt2_voltage = mavlink_msg_battery2_get_voltage(&msg);
        report.temperature_air = batt2_voltage / 1000;
        report_mask |= MAVLINK_MSG_MASK_BATTERY2;
        return true;
    }

    return false;
}
