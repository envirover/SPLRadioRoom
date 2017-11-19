/*
 SPLRadioRoom.cpp

Iridium SBD telemetry for MAVLink autopilots.

 (C) Copyright 2017 Envirover.

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

#include "SPLRadioRoom.h"
#include "MAVLinkLogger.h"
#include <unistd.h>
#include <vector>
#include <syslog.h>


SPLRadioRoom::SPLRadioRoom() :
    autopilot(), isbd(), high_latency(), seq(0), last_report_time(0)
{
    memset(&high_latency, 0, sizeof(high_latency));
}

SPLRadioRoom::~SPLRadioRoom()
{
}


bool SPLRadioRoom::handle_param_set(const mavlink_message_t& msg, mavlink_message_t& ack)
{
    if (msg.msgid == MAVLINK_MSG_ID_PARAM_SET) {
        char param_id[17];
        mavlink_msg_param_set_get_param_id(&msg, param_id);
        if (strncmp(param_id, HL_REPORT_PERIOD_PARAM, 16) == 0) {
            float value = mavlink_msg_param_set_get_param_value(&msg);
            config.set_report_period(value);

            mavlink_param_value_t paramValue;
            paramValue.param_value = value;
            paramValue.param_count = 0;
            paramValue.param_index = 0;
            mavlink_msg_param_set_get_param_id(&msg, paramValue.param_id);
            paramValue.param_type = mavlink_msg_param_set_get_param_type(&msg);

            mavlink_msg_param_value_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &paramValue);

            return true;
        }
    }

    return false;
}

bool SPLRadioRoom::handle_mission_write(const mavlink_message_t& msg, mavlink_message_t& ack)
{
    vector<mavlink_message_t> missions;

    if (msg.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {
        //syslog(LOG_DEBUG, "MISSION_COUNT MT message received.");

        uint16_t count = mavlink_msg_mission_count_get_count(&msg);

        missions.resize(count);

        mavlink_message_t mt_msg, mo_msg;
        mo_msg.len = mo_msg.msgid = 0;

        syslog(LOG_INFO, "Receiving %d mission items from ISBD.", count);

        uint16_t idx = 0;

        for (uint16_t i = 0; i < count * MAX_SEND_RETRIES && idx < count; i++) {
            bool received = false;

            if (isbd.send_receive_message(mo_msg, mt_msg, received)) {
                if (received && mt_msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM) {
                    //syslog(LOG_DEBUG, "MISSION_ITEM MT message received.");
                    missions[idx] = mt_msg;
                    idx++;
                }
            } else {
                usleep(5000000);
            }
        }

        if (idx != count) {
            syslog(LOG_WARNING, "Not all mission items received.");

            mavlink_mission_ack_t mission_ack;
            mission_ack.target_system = msg.sysid;
            mission_ack.target_component = msg.compid;
            mission_ack.type = MAV_MISSION_ERROR;
            mavlink_msg_mission_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &mission_ack);
            ack.seq = 0; //TODO: use global counter for message sequence numbers.

            return true;
        }

        syslog(LOG_INFO, "Sending mission items to ArduPilot...");

        for (int i = 0; i < MAX_SEND_RETRIES; i++) {
            if (autopilot.send_message(msg)) {
                break;
            }

            usleep(10000);
        }

        for (uint16_t i = 0; i < count; i++) {
            autopilot.send_receive_message(missions[i], ack);

            usleep(10000);
        }

        if (mavlink_msg_mission_ack_get_type(&ack) == MAV_MISSION_ACCEPTED) {
            syslog(LOG_INFO, "Mission accepted by ArduPilot.");
        } else {
            syslog(LOG_WARNING, "Mission not accepted by ArduPilot: %d", mavlink_msg_mission_ack_get_type(&ack));
        }

        return true;
    }

    return false;
}

void SPLRadioRoom::isbd_session(mavlink_message_t& mo_msg)
{
    syslog(LOG_INFO, "ISBD session started.");

    bool received;
    bool ack_received = false;
    mavlink_message_t mt_msg;

    do {
        ack_received = false;

        if (isbd.send_receive_message(mo_msg, mt_msg, received)) {
            mo_msg.len = mo_msg.msgid = 0;

            if (received) {
                ack_received = handle_param_set(mt_msg, mo_msg);

                if (!ack_received) {
                    ack_received = handle_mission_write(mt_msg, mo_msg);
                    syslog(LOG_INFO, "MISSION_ACK received.");
                }

                if (!ack_received) {
                    ack_received = autopilot.send_receive_message(mt_msg, mo_msg);
                }
            }
        }
    } while (isbd.get_waiting_wessage_count() > 0 || ack_received);

    syslog(LOG_INFO, "ISBD session ended.");
}

void SPLRadioRoom::comm_receive()
{
    mavlink_message_t msg;

    if (autopilot.receive_message(msg)) {
        update_high_latency_msg(msg);
    }
}

/**
 * Open serial devices for autopilot and ISBD transceiver.
 *
 * Automatically detect the correct serial devices if autopilot and ISBD transceiver
 * do not respond on the devices specified by the configuration properties.
 *
 */
bool SPLRadioRoom::init()
{
    vector<string> devices;
    if (config.get_auto_detect_serials()) {
        Serial::get_serial_devices(devices);
    }

    bool autopilot_connected = autopilot.init(config.get_autopilot_serial(),
                                              config.get_autopilot_serial_speed(),
                                              devices);

    // Exclude the serial device used by autopilot from the device list used
    // for ISBD transceiver serial device auto-detection.
    for (std::vector<string>::iterator iter = devices.begin(); iter != devices.end(); ++iter) {
        if (*iter == autopilot.get_path()) {
            devices.erase(iter);
            break;
        }
    }

    string isbd_serial = config.get_isbd_serial();

    if (autopilot_connected && isbd_serial == autopilot.get_path() && devices.size() > 0) {
        syslog(LOG_WARNING,
               "Autopilot detected at serial device '%s' that was assigned to ISBD transceiver by the configuration settings.",
               autopilot.get_path().data());

        isbd_serial = devices[0];
    }

    bool isbd_connected = isbd.init(isbd_serial,
                                    config.get_isbd_serial_speed(),
                                    devices);

    return autopilot_connected && isbd_connected;
}

void SPLRadioRoom::close()
{
    isbd.close();
    autopilot.close();
}

void SPLRadioRoom::loop()
{
    // Request data streams
    uint8_t req_stream_ids[] = {MAV_DATA_STREAM_EXTRA1, MAV_DATA_STREAM_EXTRA2, MAV_DATA_STREAM_EXTENDED_STATUS,
                                MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_RAW_CONTROLLER
                               };
    uint16_t req_message_rates[] = {2, 3, 2, 2, 2};

    for (size_t i = 0; i < sizeof(req_stream_ids)/sizeof(req_stream_ids[0]); i++) {
        mavlink_message_t msg;
        mavlink_msg_request_data_stream_pack(255, 1, &msg, 1, 1, req_stream_ids[i], req_message_rates[i], 1);
        if (!autopilot.send_message(msg)) {
            syslog(LOG_WARNING, "Failed to send message to autopilot.");
        }
        usleep(10000);
    }

    for (int i = 0; i < 100; i++) {
        comm_receive();
        usleep(10000);
    }

    uint16_t ra_flag = 0;

    isbd.get_ring_alert_flag(ra_flag);

    clock_t current_time = clock();

    unsigned long elapsed_time = (current_time - last_report_time) / CLOCKS_PER_SEC;

    // Start ISBD session if ring alert is received or HIGH_LATENCY report period is elapsed.
    if (ra_flag || elapsed_time > config.get_report_period()) {
        mavlink_message_t msg;
        mavlink_msg_high_latency_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &msg, &high_latency);
        //msg.seq = seq++;

        isbd_session(msg);

        last_report_time = current_time;
    }
}

inline int16_t radToCentidegrees(float rad) {
  return rad / M_PI * 18000;
}

bool SPLRadioRoom::update_high_latency_msg(const mavlink_message_t& msg)
{
  switch (msg.msgid) {
  case MAVLINK_MSG_ID_HEARTBEAT:    //0
    high_latency.base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
    high_latency.custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
    return true;
  case MAVLINK_MSG_ID_SYS_STATUS:   //1
    high_latency.battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg);
    return true;
  case MAVLINK_MSG_ID_GPS_RAW_INT:    //24
    high_latency.latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
    high_latency.longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
    high_latency.altitude_amsl = mavlink_msg_gps_raw_int_get_alt(&msg) / 1000;
    high_latency.groundspeed = mavlink_msg_gps_raw_int_get_vel(&msg) / 100;
    high_latency.gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
    high_latency.gps_nsat = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
    return true;
  case MAVLINK_MSG_ID_ATTITUDE:   //30
    high_latency.heading = (radToCentidegrees(mavlink_msg_attitude_get_yaw(&msg)) + 36000) % 36000;
    high_latency.roll = radToCentidegrees(mavlink_msg_attitude_get_roll(&msg));
    high_latency.pitch = radToCentidegrees(mavlink_msg_attitude_get_pitch(&msg));
    return true;
  case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    return true;
  case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:    //33
    //high_latency.latitude = mavlink_msg_global_position_int_get_lat(&msg);
    //high_latency.longitude = mavlink_msg_global_position_int_get_lon(&msg);
    //high_latency.altitude_amsl = mavlink_msg_global_position_int_get_alt(&msg) / 1000;
    high_latency.altitude_sp = mavlink_msg_global_position_int_get_relative_alt(&msg) / 1000;
    return true;
  case MAVLINK_MSG_ID_MISSION_CURRENT:    //42
    high_latency.wp_num = mavlink_msg_mission_current_get_seq(&msg);
    return true;
  case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:    //62
    high_latency.wp_distance = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
    high_latency.heading_sp = mavlink_msg_nav_controller_output_get_nav_bearing(&msg) * 100;
    return true;
  case MAVLINK_MSG_ID_VFR_HUD:    //74
    high_latency.airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
    high_latency.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
    //high_latency.heading = mavlink_msg_vfr_hud_get_heading(&msg) * 100;
    high_latency.climb_rate = mavlink_msg_vfr_hud_get_climb(&msg);
    high_latency.throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
    return true;
  }

  return false;
}
