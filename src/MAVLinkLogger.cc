/*
 MAVLinkLogger.cc

 This file is a part of UV Radio Room project.

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

 Created on: Oct 22, 2017
     Author: Pavel Bobov
 */

#include "MAVLinkLogger.h"
#include <stdio.h>


#define MAX_LOG_MESSAGE_SIZE 1024

void MAVLinkLogger::log(int priority, const char* prefix,
        const mavlink_message_t& message) {
    char buff[1024];

    switch (message.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t msg;
        mavlink_msg_heartbeat_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s HEARTBEAT(%d), sysid=%d, compid=%d, seq=%d, type=%d, autopilot=%d, base_mode=%d, custom_mode=%d, system_status=%d, mavlink_version=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.type, msg.autopilot, msg.base_mode, msg.custom_mode, msg.system_status, msg.mavlink_version);
        break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS: {
        mavlink_sys_status_t msg;
        mavlink_msg_sys_status_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SYS_STATUS(%d), sysid=%d, compid=%d, seq=%d, onboard_control_sensors_present=%d, onboard_control_sensors_enabled=%d, onboard_control_sensors_health=%d, load=%d, voltage_battery=%d, current_battery=%d, battery_remaining=%d, drop_rate_comm=%d, errors_comm=%d, errors_count1=%d, errors_count2=%d, errors_count3=%d, errors_count4=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.onboard_control_sensors_present, msg.onboard_control_sensors_enabled, msg.onboard_control_sensors_health, msg.load, msg.voltage_battery, msg.current_battery, msg.battery_remaining, msg.drop_rate_comm, msg.errors_comm, msg.errors_count1, msg.errors_count2, msg.errors_count3, msg.errors_count4);
        break;
    }
    case MAVLINK_MSG_ID_SYSTEM_TIME: {
        mavlink_system_time_t msg;
        mavlink_msg_system_time_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SYSTEM_TIME(%d), sysid=%d, compid=%d, seq=%d, time_unix_usec=%lld, time_boot_ms=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_unix_usec, msg.time_boot_ms);
        break;
    }
    case MAVLINK_MSG_ID_PING: {
        mavlink_ping_t msg;
        mavlink_msg_ping_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s PING(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, seq=%d, target_system=%d, target_component=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.seq, msg.target_system, msg.target_component);
        break;
    }
    case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL: {
        mavlink_change_operator_control_t msg;
        mavlink_msg_change_operator_control_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s CHANGE_OPERATOR_CONTROL(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, control_request=%d, version=%d, passkey=%.25s",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.control_request, msg.version, msg.passkey);
        break;
    }
    case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK: {
        mavlink_change_operator_control_ack_t msg;
        mavlink_msg_change_operator_control_ack_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s CHANGE_OPERATOR_CONTROL_ACK(%d), sysid=%d, compid=%d, seq=%d, gcs_system_id=%d, control_request=%d, ack=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.gcs_system_id, msg.control_request, msg.ack);
        break;
    }
    case MAVLINK_MSG_ID_AUTH_KEY: {
        mavlink_auth_key_t msg;
        mavlink_msg_auth_key_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s AUTH_KEY(%d), sysid=%d, compid=%d, seq=%d, key=%.32s",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.key);
        break;
    }
    case MAVLINK_MSG_ID_SET_MODE: {
        mavlink_set_mode_t msg;
        mavlink_msg_set_mode_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SET_MODE(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, base_mode=%d, custom_mode=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.base_mode, msg.custom_mode);
        break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
        mavlink_param_request_read_t msg;
        mavlink_msg_param_request_read_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s PARAM_REQUEST_READ(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, param_id=%.16s, param_index=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.param_id, msg.param_index);
        break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
        mavlink_param_request_list_t msg;
        mavlink_msg_param_request_list_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s PARAM_REQUEST_LIST(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component);
        break;
    }
    case MAVLINK_MSG_ID_PARAM_VALUE: {
        mavlink_param_value_t msg;
        mavlink_msg_param_value_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s PARAM_VALUE(%d), sysid=%d, compid=%d, seq=%d, param_id=%.16s, param_value=%f, param_type=%d, param_count=%d, param_index=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.param_id, msg.param_value, msg.param_type, msg.param_count, msg.param_index);
        break;
    }
    case MAVLINK_MSG_ID_PARAM_SET: {
        mavlink_param_set_t msg;
        mavlink_msg_param_set_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s PARAM_SET(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, param_id=%.16s, param_value=%f, param_type=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.param_id, msg.param_value, msg.param_type);
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t msg;
        mavlink_msg_gps_raw_int_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s GPS_RAW_INT(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, fix_type=%d, lat=%d, lon=%d, alt=%d, eph=%d, epv=%d, vel=%d, cog=%d, satellites_visible=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.fix_type, msg.lat, msg.lon, msg.alt, msg.eph, msg.epv, msg.vel, msg.cog, msg.satellites_visible);
        break;
    }
    /*
    case MAVLINK_MSG_ID_GPS_STATUS: {
        mavlink_gps_status_t msg;
        mavlink_msg_gps_status_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s GPS_STATUS(%d), sysid=%d, compid=%d, seq=%d, satellites_visible=%d, satellite_prn=%d, satellite_used=%d, satellite_elevation=%d, satellite_azimuth=%d, satellite_snr=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.satellites_visible, msg.satellite_prn, msg.satellite_used, msg.satellite_elevation, msg.satellite_azimuth, msg.satellite_snr);
        break;
    }
    */
    case MAVLINK_MSG_ID_SCALED_IMU: {
        mavlink_scaled_imu_t msg;
        mavlink_msg_scaled_imu_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SCALED_IMU(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, xacc=%d, yacc=%d, zacc=%d, xgyro=%d, ygyro=%d, zgyro=%d, xmag=%d, ymag=%d, zmag=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro, msg.xmag, msg.ymag, msg.zmag);
        break;
    }
    case MAVLINK_MSG_ID_RAW_IMU: {
        mavlink_raw_imu_t msg;
        mavlink_msg_raw_imu_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s RAW_IMU(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, xacc=%d, yacc=%d, zacc=%d, xgyro=%d, ygyro=%d, zgyro=%d, xmag=%d, ymag=%d, zmag=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro, msg.xmag, msg.ymag, msg.zmag);
        break;
    }
    case MAVLINK_MSG_ID_RAW_PRESSURE: {
        mavlink_raw_pressure_t msg;
        mavlink_msg_raw_pressure_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s RAW_PRESSURE(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, press_abs=%d, press_diff1=%d, press_diff2=%d, temperature=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.press_abs, msg.press_diff1, msg.press_diff2, msg.temperature);
        break;
    }
    case MAVLINK_MSG_ID_SCALED_PRESSURE: {
        mavlink_scaled_pressure_t msg;
        mavlink_msg_scaled_pressure_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SCALED_PRESSURE(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, press_abs=%f, press_diff=%f, temperature=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.press_abs, msg.press_diff, msg.temperature);
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t msg;
        mavlink_msg_attitude_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s ATTITUDE(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, roll=%f, pitch=%f, yaw=%f, rollspeed=%f, pitchspeed=%f, yawspeed=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed);
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
        mavlink_attitude_quaternion_t msg;
        mavlink_msg_attitude_quaternion_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s ATTITUDE_QUATERNION(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, q1=%f, q2=%f, q3=%f, q4=%f, rollspeed=%f, pitchspeed=%f, yawspeed=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.q1, msg.q2, msg.q3, msg.q4, msg.rollspeed, msg.pitchspeed, msg.yawspeed);
        break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        mavlink_local_position_ned_t msg;
        mavlink_msg_local_position_ned_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s LOCAL_POSITION_NED(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, x=%f, y=%f, z=%f, vx=%f, vy=%f, vz=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz);
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t msg;
        mavlink_msg_global_position_int_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s GLOBAL_POSITION_INT(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, lat=%d, lon=%d, alt=%d, relative_alt=%d, vx=%d, vy=%d, vz=%d, hdg=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.lat, msg.lon, msg.alt, msg.relative_alt, msg.vx, msg.vy, msg.vz, msg.hdg);
        break;
    }
    case MAVLINK_MSG_ID_RC_CHANNELS_SCALED: {
        mavlink_rc_channels_scaled_t msg;
        mavlink_msg_rc_channels_scaled_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s RC_CHANNELS_SCALED(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, port=%d, chan1_scaled=%d, chan2_scaled=%d, chan3_scaled=%d, chan4_scaled=%d, chan5_scaled=%d, chan6_scaled=%d, chan7_scaled=%d, chan8_scaled=%d, rssi=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.port, msg.chan1_scaled, msg.chan2_scaled, msg.chan3_scaled, msg.chan4_scaled, msg.chan5_scaled, msg.chan6_scaled, msg.chan7_scaled, msg.chan8_scaled, msg.rssi);
        break;
    }
    case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
        mavlink_rc_channels_raw_t msg;
        mavlink_msg_rc_channels_raw_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s RC_CHANNELS_RAW(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, port=%d, chan1_raw=%d, chan2_raw=%d, chan3_raw=%d, chan4_raw=%d, chan5_raw=%d, chan6_raw=%d, chan7_raw=%d, chan8_raw=%d, rssi=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.port, msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw, msg.rssi);
        break;
    }
    /*
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
        mavlink_servo_output_raw_t msg;
        mavlink_msg_servo_output_raw_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SERVO_OUTPUT_RAW(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, port=%d, servo1_raw=%d, servo2_raw=%d, servo3_raw=%d, servo4_raw=%d, servo5_raw=%d, servo6_raw=%d, servo7_raw=%d, servo8_raw=%d, servo9_raw=%d, servo10_raw=%d, servo11_raw=%d, servo12_raw=%d, servo13_raw=%d, servo14_raw=%d, servo15_raw=%d, servo16_raw=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_usec, msg.port, msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw, msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw, msg.servo9_raw, msg.servo10_raw, msg.servo11_raw, msg.servo12_raw, msg.servo13_raw, msg.servo14_raw, msg.servo15_raw, msg.servo16_raw);
        break;
    }
    */
    case MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST: {
        mavlink_mission_request_partial_list_t msg;
        mavlink_msg_mission_request_partial_list_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MISSION_REQUEST_PARTIAL_LIST(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, start_index=%d, end_index=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.start_index, msg.end_index);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: {
        mavlink_mission_write_partial_list_t msg;
        mavlink_msg_mission_write_partial_list_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MISSION_WRITE_PARTIAL_LIST(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, start_index=%d, end_index=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.start_index, msg.end_index);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM: {
        mavlink_mission_item_t msg;
        mavlink_msg_mission_item_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MISSION_ITEM(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, seq=%d, frame=%d, command=%d, current=%d, autocontinue=%d, param1=%f, param2=%f, param3=%f, param4=%f, x=%f, y=%f, z=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.seq, msg.frame, msg.command, msg.current, msg.autocontinue, msg.param1, msg.param2, msg.param3, msg.param4, msg.x, msg.y, msg.z);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST: {
        mavlink_mission_request_t msg;
        mavlink_msg_mission_request_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MISSION_REQUEST(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, seq=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.seq);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
        mavlink_mission_set_current_t msg;
        mavlink_msg_mission_set_current_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MISSION_SET_CURRENT(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, seq=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.seq);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_CURRENT: {
        mavlink_mission_current_t msg;
        mavlink_msg_mission_current_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MISSION_CURRENT(%d), sysid=%d, compid=%d, seq=%d, seq=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.seq);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
        mavlink_mission_request_list_t msg;
        mavlink_msg_mission_request_list_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MISSION_REQUEST_LIST(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_COUNT: {
        mavlink_mission_count_t msg;
        mavlink_msg_mission_count_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MISSION_COUNT(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, count=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.count);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL: {
        mavlink_mission_clear_all_t msg;
        mavlink_msg_mission_clear_all_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MISSION_CLEAR_ALL(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: {
        mavlink_mission_item_reached_t msg;
        mavlink_msg_mission_item_reached_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MISSION_ITEM_REACHED(%d), sysid=%d, compid=%d, seq=%d, seq=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.seq);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ACK: {
        mavlink_mission_ack_t msg;
        mavlink_msg_mission_ack_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MISSION_ACK(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, type=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.type);
        break;
    }
    case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN: {
        mavlink_set_gps_global_origin_t msg;
        mavlink_msg_set_gps_global_origin_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SET_GPS_GLOBAL_ORIGIN(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, latitude=%d, longitude=%d, altitude=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.latitude, msg.longitude, msg.altitude);
        break;
    }
    case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN: {
        mavlink_gps_global_origin_t msg;
        mavlink_msg_gps_global_origin_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s GPS_GLOBAL_ORIGIN(%d), sysid=%d, compid=%d, seq=%d, latitude=%d, longitude=%d, altitude=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.latitude, msg.longitude, msg.altitude);
        break;
    }
    case MAVLINK_MSG_ID_PARAM_MAP_RC: {
        mavlink_param_map_rc_t msg;
        mavlink_msg_param_map_rc_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s PARAM_MAP_RC(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, param_id=%.16s, param_index=%d, parameter_rc_channel_index=%d, param_value0=%f, scale=%f, param_value_min=%f, param_value_max=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.param_id, msg.param_index, msg.parameter_rc_channel_index, msg.param_value0, msg.scale, msg.param_value_min, msg.param_value_max);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {
        mavlink_mission_request_int_t msg;
        mavlink_msg_mission_request_int_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MISSION_REQUEST_INT(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, seq=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.seq);
        break;
    }
    case MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA: {
        mavlink_safety_set_allowed_area_t msg;
        mavlink_msg_safety_set_allowed_area_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SAFETY_SET_ALLOWED_AREA(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, frame=%d, p1x=%f, p1y=%f, p1z=%f, p2x=%f, p2y=%f, p2z=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.frame, msg.p1x, msg.p1y, msg.p1z, msg.p2x, msg.p2y, msg.p2z);
        break;
    }
    case MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA: {
        mavlink_safety_allowed_area_t msg;
        mavlink_msg_safety_allowed_area_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SAFETY_ALLOWED_AREA(%d), sysid=%d, compid=%d, seq=%d, frame=%d, p1x=%f, p1y=%f, p1z=%f, p2x=%f, p2y=%f, p2z=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.frame, msg.p1x, msg.p1y, msg.p1z, msg.p2x, msg.p2y, msg.p2z);
        break;
    }
    /*
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV: {
        mavlink_attitude_quaternion_cov_t msg;
        mavlink_msg_attitude_quaternion_cov_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s ATTITUDE_QUATERNION_COV(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, q=%d, rollspeed=%d, pitchspeed=%d, yawspeed=%d, covariance=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_usec, msg.q, msg.rollspeed, msg.pitchspeed, msg.yawspeed, msg.covariance);
        break;
    }
    */
    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: {
        mavlink_nav_controller_output_t msg;
        mavlink_msg_nav_controller_output_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s NAV_CONTROLLER_OUTPUT(%d), sysid=%d, compid=%d, seq=%d, nav_roll=%f, nav_pitch=%f, nav_bearing=%d, target_bearing=%d, wp_dist=%d, alt_error=%f, aspd_error=%f, xtrack_error=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.nav_roll, msg.nav_pitch, msg.nav_bearing, msg.target_bearing, msg.wp_dist, msg.alt_error, msg.aspd_error, msg.xtrack_error);
        break;
    }
    /*
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV: {
        mavlink_global_position_int_cov_t msg;
        mavlink_msg_global_position_int_cov_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s GLOBAL_POSITION_INT_COV(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, estimator_type=%d, lat=%d, lon=%d, alt=%d, relative_alt=%d, vx=%d, vy=%d, vz=%d, covariance=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_usec, msg.estimator_type, msg.lat, msg.lon, msg.alt, msg.relative_alt, msg.vx, msg.vy, msg.vz, msg.covariance);
        break;
    }
    */
    /*
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV: {
        mavlink_local_position_ned_cov_t msg;
        mavlink_msg_local_position_ned_cov_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s LOCAL_POSITION_NED_COV(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, estimator_type=%d, x=%d, y=%d, z=%d, vx=%d, vy=%d, vz=%d, ax=%d, ay=%d, az=%d, covariance=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_usec, msg.estimator_type, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.ax, msg.ay, msg.az, msg.covariance);
        break;
    }
    */
    case MAVLINK_MSG_ID_RC_CHANNELS: {
        mavlink_rc_channels_t msg;
        mavlink_msg_rc_channels_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s RC_CHANNELS(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, chancount=%d, chan1_raw=%d, chan2_raw=%d, chan3_raw=%d, chan4_raw=%d, chan5_raw=%d, chan6_raw=%d, chan7_raw=%d, chan8_raw=%d, chan9_raw=%d, chan10_raw=%d, chan11_raw=%d, chan12_raw=%d, chan13_raw=%d, chan14_raw=%d, chan15_raw=%d, chan16_raw=%d, chan17_raw=%d, chan18_raw=%d, rssi=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.chancount, msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw, msg.chan9_raw, msg.chan10_raw, msg.chan11_raw, msg.chan12_raw, msg.chan13_raw, msg.chan14_raw, msg.chan15_raw, msg.chan16_raw, msg.chan17_raw, msg.chan18_raw, msg.rssi);
        break;
    }
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: {
        mavlink_request_data_stream_t msg;
        mavlink_msg_request_data_stream_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s REQUEST_DATA_STREAM(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, req_stream_id=%d, req_message_rate=%d, start_stop=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.req_stream_id, msg.req_message_rate, msg.start_stop);
        break;
    }
    case MAVLINK_MSG_ID_DATA_STREAM: {
        mavlink_data_stream_t msg;
        mavlink_msg_data_stream_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s DATA_STREAM(%d), sysid=%d, compid=%d, seq=%d, stream_id=%d, message_rate=%d, on_off=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.stream_id, msg.message_rate, msg.on_off);
        break;
    }
    case MAVLINK_MSG_ID_MANUAL_CONTROL: {
        mavlink_manual_control_t msg;
        mavlink_msg_manual_control_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MANUAL_CONTROL(%d), sysid=%d, compid=%d, seq=%d, target=%d, x=%d, y=%d, z=%d, r=%d, buttons=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target, msg.x, msg.y, msg.z, msg.r, msg.buttons);
        break;
    }
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
        mavlink_rc_channels_override_t msg;
        mavlink_msg_rc_channels_override_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s RC_CHANNELS_OVERRIDE(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, chan1_raw=%d, chan2_raw=%d, chan3_raw=%d, chan4_raw=%d, chan5_raw=%d, chan6_raw=%d, chan7_raw=%d, chan8_raw=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
        mavlink_mission_item_int_t msg;
        mavlink_msg_mission_item_int_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MISSION_ITEM_INT(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, seq=%d, frame=%d, command=%d, current=%d, autocontinue=%d, param1=%f, param2=%f, param3=%f, param4=%f, x=%d, y=%d, z=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.seq, msg.frame, msg.command, msg.current, msg.autocontinue, msg.param1, msg.param2, msg.param3, msg.param4, msg.x, msg.y, msg.z);
        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD: {
        mavlink_vfr_hud_t msg;
        mavlink_msg_vfr_hud_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s VFR_HUD(%d), sysid=%d, compid=%d, seq=%d, airspeed=%f, groundspeed=%f, heading=%d, throttle=%d, alt=%f, climb=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.airspeed, msg.groundspeed, msg.heading, msg.throttle, msg.alt, msg.climb);
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_INT: {
        mavlink_command_int_t msg;
        mavlink_msg_command_int_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s COMMAND_INT(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, frame=%d, command=%d, current=%d, autocontinue=%d, param1=%f, param2=%f, param3=%f, param4=%f, x=%d, y=%d, z=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.frame, msg.command, msg.current, msg.autocontinue, msg.param1, msg.param2, msg.param3, msg.param4, msg.x, msg.y, msg.z);
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t msg;
        mavlink_msg_command_long_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s COMMAND_LONG(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, command=%d, confirmation=%d, param1=%f, param2=%f, param3=%f, param4=%f, param5=%f, param6=%f, param7=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.command, msg.confirmation, msg.param1, msg.param2, msg.param3, msg.param4, msg.param5, msg.param6, msg.param7);
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t msg;
        mavlink_msg_command_ack_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s COMMAND_ACK(%d), sysid=%d, compid=%d, seq=%d, command=%d, result=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.command, msg.result);
        break;
    }
    case MAVLINK_MSG_ID_MANUAL_SETPOINT: {
        mavlink_manual_setpoint_t msg;
        mavlink_msg_manual_setpoint_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MANUAL_SETPOINT(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, roll=%f, pitch=%f, yaw=%f, thrust=%f, mode_switch=%d, manual_override_switch=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw, msg.thrust, msg.mode_switch, msg.manual_override_switch);
        break;
    }
    /*
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: {
        mavlink_set_attitude_target_t msg;
        mavlink_msg_set_attitude_target_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SET_ATTITUDE_TARGET(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, target_system=%d, target_component=%d, type_mask=%d, q=%d, body_roll_rate=%d, body_pitch_rate=%d, body_yaw_rate=%d, thrust=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.target_system, msg.target_component, msg.type_mask, msg.q, msg.body_roll_rate, msg.body_pitch_rate, msg.body_yaw_rate, msg.thrust);
        break;
    }
    */
    /*
    case MAVLINK_MSG_ID_ATTITUDE_TARGET: {
        mavlink_attitude_target_t msg;
        mavlink_msg_attitude_target_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s ATTITUDE_TARGET(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, type_mask=%d, q=%d, body_roll_rate=%d, body_pitch_rate=%d, body_yaw_rate=%d, thrust=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.type_mask, msg.q, msg.body_roll_rate, msg.body_pitch_rate, msg.body_yaw_rate, msg.thrust);
        break;
    }
    */
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: {
        mavlink_set_position_target_local_ned_t msg;
        mavlink_msg_set_position_target_local_ned_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SET_POSITION_TARGET_LOCAL_NED(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, target_system=%d, target_component=%d, coordinate_frame=%d, type_mask=%d, x=%f, y=%f, z=%f, vx=%f, vy=%f, vz=%f, afx=%f, afy=%f, afz=%f, yaw=%f, yaw_rate=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.target_system, msg.target_component, msg.coordinate_frame, msg.type_mask, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.afx, msg.afy, msg.afz, msg.yaw, msg.yaw_rate);
        break;
    }
    case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: {
        mavlink_position_target_local_ned_t msg;
        mavlink_msg_position_target_local_ned_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s POSITION_TARGET_LOCAL_NED(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, coordinate_frame=%d, type_mask=%d, x=%f, y=%f, z=%f, vx=%f, vy=%f, vz=%f, afx=%f, afy=%f, afz=%f, yaw=%f, yaw_rate=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.coordinate_frame, msg.type_mask, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.afx, msg.afy, msg.afz, msg.yaw, msg.yaw_rate);
        break;
    }
    case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: {
        mavlink_set_position_target_global_int_t msg;
        mavlink_msg_set_position_target_global_int_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SET_POSITION_TARGET_GLOBAL_INT(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, target_system=%d, target_component=%d, coordinate_frame=%d, type_mask=%d, lat_int=%d, lon_int=%d, alt=%f, vx=%f, vy=%f, vz=%f, afx=%f, afy=%f, afz=%f, yaw=%f, yaw_rate=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.target_system, msg.target_component, msg.coordinate_frame, msg.type_mask, msg.lat_int, msg.lon_int, msg.alt, msg.vx, msg.vy, msg.vz, msg.afx, msg.afy, msg.afz, msg.yaw, msg.yaw_rate);
        break;
    }
    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT: {
        mavlink_position_target_global_int_t msg;
        mavlink_msg_position_target_global_int_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s POSITION_TARGET_GLOBAL_INT(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, coordinate_frame=%d, type_mask=%d, lat_int=%d, lon_int=%d, alt=%f, vx=%f, vy=%f, vz=%f, afx=%f, afy=%f, afz=%f, yaw=%f, yaw_rate=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.coordinate_frame, msg.type_mask, msg.lat_int, msg.lon_int, msg.alt, msg.vx, msg.vy, msg.vz, msg.afx, msg.afy, msg.afz, msg.yaw, msg.yaw_rate);
        break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET: {
        mavlink_local_position_ned_system_global_offset_t msg;
        mavlink_msg_local_position_ned_system_global_offset_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw);
        break;
    }
    case MAVLINK_MSG_ID_HIL_STATE: {
        mavlink_hil_state_t msg;
        mavlink_msg_hil_state_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s HIL_STATE(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, roll=%f, pitch=%f, yaw=%f, rollspeed=%f, pitchspeed=%f, yawspeed=%f, lat=%d, lon=%d, alt=%d, vx=%d, vy=%d, vz=%d, xacc=%d, yacc=%d, zacc=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed, msg.lat, msg.lon, msg.alt, msg.vx, msg.vy, msg.vz, msg.xacc, msg.yacc, msg.zacc);
        break;
    }
    case MAVLINK_MSG_ID_HIL_CONTROLS: {
        mavlink_hil_controls_t msg;
        mavlink_msg_hil_controls_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s HIL_CONTROLS(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, roll_ailerons=%f, pitch_elevator=%f, yaw_rudder=%f, throttle=%f, aux1=%f, aux2=%f, aux3=%f, aux4=%f, mode=%d, nav_mode=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.roll_ailerons, msg.pitch_elevator, msg.yaw_rudder, msg.throttle, msg.aux1, msg.aux2, msg.aux3, msg.aux4, msg.mode, msg.nav_mode);
        break;
    }
    case MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW: {
        mavlink_hil_rc_inputs_raw_t msg;
        mavlink_msg_hil_rc_inputs_raw_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s HIL_RC_INPUTS_RAW(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, chan1_raw=%d, chan2_raw=%d, chan3_raw=%d, chan4_raw=%d, chan5_raw=%d, chan6_raw=%d, chan7_raw=%d, chan8_raw=%d, chan9_raw=%d, chan10_raw=%d, chan11_raw=%d, chan12_raw=%d, rssi=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw, msg.chan9_raw, msg.chan10_raw, msg.chan11_raw, msg.chan12_raw, msg.rssi);
        break;
    }
    /*
    case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS: {
        mavlink_hil_actuator_controls_t msg;
        mavlink_msg_hil_actuator_controls_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s HIL_ACTUATOR_CONTROLS(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, controls=%d, mode=%d, flags=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_usec, msg.controls, msg.mode, msg.flags);
        break;
    }
    */
    case MAVLINK_MSG_ID_OPTICAL_FLOW: {
        mavlink_optical_flow_t msg;
        mavlink_msg_optical_flow_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s OPTICAL_FLOW(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, sensor_id=%d, flow_x=%d, flow_y=%d, flow_comp_m_x=%f, flow_comp_m_y=%f, quality=%d, ground_distance=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.sensor_id, msg.flow_x, msg.flow_y, msg.flow_comp_m_x, msg.flow_comp_m_y, msg.quality, msg.ground_distance);
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE: {
        mavlink_global_vision_position_estimate_t msg;
        mavlink_msg_global_vision_position_estimate_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s GLOBAL_VISION_POSITION_ESTIMATE(%d), sysid=%d, compid=%d, seq=%d, usec=%lld, x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.usec, msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw);
        break;
    }
    case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE: {
        mavlink_vision_position_estimate_t msg;
        mavlink_msg_vision_position_estimate_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s VISION_POSITION_ESTIMATE(%d), sysid=%d, compid=%d, seq=%d, usec=%lld, x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.usec, msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw);
        break;
    }
    case MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE: {
        mavlink_vision_speed_estimate_t msg;
        mavlink_msg_vision_speed_estimate_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s VISION_SPEED_ESTIMATE(%d), sysid=%d, compid=%d, seq=%d, usec=%lld, x=%f, y=%f, z=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.usec, msg.x, msg.y, msg.z);
        break;
    }
    case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE: {
        mavlink_vicon_position_estimate_t msg;
        mavlink_msg_vicon_position_estimate_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s VICON_POSITION_ESTIMATE(%d), sysid=%d, compid=%d, seq=%d, usec=%lld, x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.usec, msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw);
        break;
    }
    case MAVLINK_MSG_ID_HIGHRES_IMU: {
        mavlink_highres_imu_t msg;
        mavlink_msg_highres_imu_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s HIGHRES_IMU(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, xacc=%f, yacc=%f, zacc=%f, xgyro=%f, ygyro=%f, zgyro=%f, xmag=%f, ymag=%f, zmag=%f, abs_pressure=%f, diff_pressure=%f, pressure_alt=%f, temperature=%f, fields_updated=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro, msg.xmag, msg.ymag, msg.zmag, msg.abs_pressure, msg.diff_pressure, msg.pressure_alt, msg.temperature, msg.fields_updated);
        break;
    }
    case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD: {
        mavlink_optical_flow_rad_t msg;
        mavlink_msg_optical_flow_rad_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s OPTICAL_FLOW_RAD(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, sensor_id=%d, integration_time_us=%d, integrated_x=%f, integrated_y=%f, integrated_xgyro=%f, integrated_ygyro=%f, integrated_zgyro=%f, temperature=%d, quality=%d, time_delta_distance_us=%d, distance=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.sensor_id, msg.integration_time_us, msg.integrated_x, msg.integrated_y, msg.integrated_xgyro, msg.integrated_ygyro, msg.integrated_zgyro, msg.temperature, msg.quality, msg.time_delta_distance_us, msg.distance);
        break;
    }
    case MAVLINK_MSG_ID_HIL_SENSOR: {
        mavlink_hil_sensor_t msg;
        mavlink_msg_hil_sensor_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s HIL_SENSOR(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, xacc=%f, yacc=%f, zacc=%f, xgyro=%f, ygyro=%f, zgyro=%f, xmag=%f, ymag=%f, zmag=%f, abs_pressure=%f, diff_pressure=%f, pressure_alt=%f, temperature=%f, fields_updated=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro, msg.xmag, msg.ymag, msg.zmag, msg.abs_pressure, msg.diff_pressure, msg.pressure_alt, msg.temperature, msg.fields_updated);
        break;
    }
    case MAVLINK_MSG_ID_SIM_STATE: {
        mavlink_sim_state_t msg;
        mavlink_msg_sim_state_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SIM_STATE(%d), sysid=%d, compid=%d, seq=%d, q1=%f, q2=%f, q3=%f, q4=%f, roll=%f, pitch=%f, yaw=%f, xacc=%f, yacc=%f, zacc=%f, xgyro=%f, ygyro=%f, zgyro=%f, lat=%f, lon=%f, alt=%f, std_dev_horz=%f, std_dev_vert=%f, vn=%f, ve=%f, vd=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.q1, msg.q2, msg.q3, msg.q4, msg.roll, msg.pitch, msg.yaw, msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro, msg.lat, msg.lon, msg.alt, msg.std_dev_horz, msg.std_dev_vert, msg.vn, msg.ve, msg.vd);
        break;
    }
    case MAVLINK_MSG_ID_RADIO_STATUS: {
        mavlink_radio_status_t msg;
        mavlink_msg_radio_status_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s RADIO_STATUS(%d), sysid=%d, compid=%d, seq=%d, rssi=%d, remrssi=%d, txbuf=%d, noise=%d, remnoise=%d, rxerrors=%d, fixed=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.rssi, msg.remrssi, msg.txbuf, msg.noise, msg.remnoise, msg.rxerrors, msg.fixed);
        break;
    }
    /*
    case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL: {
        mavlink_file_transfer_protocol_t msg;
        mavlink_msg_file_transfer_protocol_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s FILE_TRANSFER_PROTOCOL(%d), sysid=%d, compid=%d, seq=%d, target_network=%d, target_system=%d, target_component=%d, payload=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_network, msg.target_system, msg.target_component, msg.payload);
        break;
    }
    */
    case MAVLINK_MSG_ID_TIMESYNC: {
        mavlink_timesync_t msg;
        mavlink_msg_timesync_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s TIMESYNC(%d), sysid=%d, compid=%d, seq=%d, tc1=%lld, ts1=%lld",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.tc1, (long long int)msg.ts1);
        break;
    }
    case MAVLINK_MSG_ID_CAMERA_TRIGGER: {
        mavlink_camera_trigger_t msg;
        mavlink_msg_camera_trigger_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s CAMERA_TRIGGER(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, seq=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.seq);
        break;
    }
    case MAVLINK_MSG_ID_HIL_GPS: {
        mavlink_hil_gps_t msg;
        mavlink_msg_hil_gps_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s HIL_GPS(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, fix_type=%d, lat=%d, lon=%d, alt=%d, eph=%d, epv=%d, vel=%d, vn=%d, ve=%d, vd=%d, cog=%d, satellites_visible=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.fix_type, msg.lat, msg.lon, msg.alt, msg.eph, msg.epv, msg.vel, msg.vn, msg.ve, msg.vd, msg.cog, msg.satellites_visible);
        break;
    }
    case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW: {
        mavlink_hil_optical_flow_t msg;
        mavlink_msg_hil_optical_flow_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s HIL_OPTICAL_FLOW(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, sensor_id=%d, integration_time_us=%d, integrated_x=%f, integrated_y=%f, integrated_xgyro=%f, integrated_ygyro=%f, integrated_zgyro=%f, temperature=%d, quality=%d, time_delta_distance_us=%d, distance=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.sensor_id, msg.integration_time_us, msg.integrated_x, msg.integrated_y, msg.integrated_xgyro, msg.integrated_ygyro, msg.integrated_zgyro, msg.temperature, msg.quality, msg.time_delta_distance_us, msg.distance);
        break;
    }
    /*
    case MAVLINK_MSG_ID_HIL_STATE_QUATERNION: {
        mavlink_hil_state_quaternion_t msg;
        mavlink_msg_hil_state_quaternion_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s HIL_STATE_QUATERNION(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, attitude_quaternion=%d, rollspeed=%d, pitchspeed=%d, yawspeed=%d, lat=%d, lon=%d, alt=%d, vx=%d, vy=%d, vz=%d, ind_airspeed=%d, true_airspeed=%d, xacc=%d, yacc=%d, zacc=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_usec, msg.attitude_quaternion, msg.rollspeed, msg.pitchspeed, msg.yawspeed, msg.lat, msg.lon, msg.alt, msg.vx, msg.vy, msg.vz, msg.ind_airspeed, msg.true_airspeed, msg.xacc, msg.yacc, msg.zacc);
        break;
    }
    */
    case MAVLINK_MSG_ID_SCALED_IMU2: {
        mavlink_scaled_imu2_t msg;
        mavlink_msg_scaled_imu2_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SCALED_IMU2(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, xacc=%d, yacc=%d, zacc=%d, xgyro=%d, ygyro=%d, zgyro=%d, xmag=%d, ymag=%d, zmag=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro, msg.xmag, msg.ymag, msg.zmag);
        break;
    }
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST: {
        mavlink_log_request_list_t msg;
        mavlink_msg_log_request_list_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s LOG_REQUEST_LIST(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, start=%d, end=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.start, msg.end);
        break;
    }
    case MAVLINK_MSG_ID_LOG_ENTRY: {
        mavlink_log_entry_t msg;
        mavlink_msg_log_entry_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s LOG_ENTRY(%d), sysid=%d, compid=%d, seq=%d, id=%d, num_logs=%d, last_log_num=%d, time_utc=%d, size=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.id, msg.num_logs, msg.last_log_num, msg.time_utc, msg.size);
        break;
    }
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA: {
        mavlink_log_request_data_t msg;
        mavlink_msg_log_request_data_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s LOG_REQUEST_DATA(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, id=%d, ofs=%d, count=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.id, msg.ofs, msg.count);
        break;
    }
    /*
    case MAVLINK_MSG_ID_LOG_DATA: {
        mavlink_log_data_t msg;
        mavlink_msg_log_data_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s LOG_DATA(%d), sysid=%d, compid=%d, seq=%d, id=%d, ofs=%d, count=%d, data=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.id, msg.ofs, msg.count, msg.data);
        break;
    }
    */
    case MAVLINK_MSG_ID_LOG_ERASE: {
        mavlink_log_erase_t msg;
        mavlink_msg_log_erase_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s LOG_ERASE(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component);
        break;
    }
    case MAVLINK_MSG_ID_LOG_REQUEST_END: {
        mavlink_log_request_end_t msg;
        mavlink_msg_log_request_end_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s LOG_REQUEST_END(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component);
        break;
    }
    /*
    case MAVLINK_MSG_ID_GPS_INJECT_DATA: {
        mavlink_gps_inject_data_t msg;
        mavlink_msg_gps_inject_data_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s GPS_INJECT_DATA(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, len=%d, data=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.len, msg.data);
        break;
    }
    */
    case MAVLINK_MSG_ID_GPS2_RAW: {
        mavlink_gps2_raw_t msg;
        mavlink_msg_gps2_raw_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s GPS2_RAW(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, fix_type=%d, lat=%d, lon=%d, alt=%d, eph=%d, epv=%d, vel=%d, cog=%d, satellites_visible=%d, dgps_numch=%d, dgps_age=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.fix_type, msg.lat, msg.lon, msg.alt, msg.eph, msg.epv, msg.vel, msg.cog, msg.satellites_visible, msg.dgps_numch, msg.dgps_age);
        break;
    }
    case MAVLINK_MSG_ID_POWER_STATUS: {
        mavlink_power_status_t msg;
        mavlink_msg_power_status_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s POWER_STATUS(%d), sysid=%d, compid=%d, seq=%d, Vcc=%d, Vservo=%d, flags=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.Vcc, msg.Vservo, msg.flags);
        break;
    }
    /*
    case MAVLINK_MSG_ID_SERIAL_CONTROL: {
        mavlink_serial_control_t msg;
        mavlink_msg_serial_control_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SERIAL_CONTROL(%d), sysid=%d, compid=%d, seq=%d, device=%d, flags=%d, timeout=%d, baudrate=%d, count=%d, data=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.device, msg.flags, msg.timeout, msg.baudrate, msg.count, msg.data);
        break;
    }
    */
    case MAVLINK_MSG_ID_GPS_RTK: {
        mavlink_gps_rtk_t msg;
        mavlink_msg_gps_rtk_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s GPS_RTK(%d), sysid=%d, compid=%d, seq=%d, time_last_baseline_ms=%d, rtk_receiver_id=%d, wn=%d, tow=%d, rtk_health=%d, rtk_rate=%d, nsats=%d, baseline_coords_type=%d, baseline_a_mm=%d, baseline_b_mm=%d, baseline_c_mm=%d, accuracy=%d, iar_num_hypotheses=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_last_baseline_ms, msg.rtk_receiver_id, msg.wn, msg.tow, msg.rtk_health, msg.rtk_rate, msg.nsats, msg.baseline_coords_type, msg.baseline_a_mm, msg.baseline_b_mm, msg.baseline_c_mm, msg.accuracy, msg.iar_num_hypotheses);
        break;
    }
    case MAVLINK_MSG_ID_GPS2_RTK: {
        mavlink_gps2_rtk_t msg;
        mavlink_msg_gps2_rtk_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s GPS2_RTK(%d), sysid=%d, compid=%d, seq=%d, time_last_baseline_ms=%d, rtk_receiver_id=%d, wn=%d, tow=%d, rtk_health=%d, rtk_rate=%d, nsats=%d, baseline_coords_type=%d, baseline_a_mm=%d, baseline_b_mm=%d, baseline_c_mm=%d, accuracy=%d, iar_num_hypotheses=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_last_baseline_ms, msg.rtk_receiver_id, msg.wn, msg.tow, msg.rtk_health, msg.rtk_rate, msg.nsats, msg.baseline_coords_type, msg.baseline_a_mm, msg.baseline_b_mm, msg.baseline_c_mm, msg.accuracy, msg.iar_num_hypotheses);
        break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU3: {
        mavlink_scaled_imu3_t msg;
        mavlink_msg_scaled_imu3_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SCALED_IMU3(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, xacc=%d, yacc=%d, zacc=%d, xgyro=%d, ygyro=%d, zgyro=%d, xmag=%d, ymag=%d, zmag=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro, msg.xmag, msg.ymag, msg.zmag);
        break;
    }
    case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE: {
        mavlink_data_transmission_handshake_t msg;
        mavlink_msg_data_transmission_handshake_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s DATA_TRANSMISSION_HANDSHAKE(%d), sysid=%d, compid=%d, seq=%d, type=%d, size=%d, width=%d, height=%d, packets=%d, payload=%d, jpg_quality=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.type, msg.size, msg.width, msg.height, msg.packets, msg.payload, msg.jpg_quality);
        break;
    }
    /*
    case MAVLINK_MSG_ID_ENCAPSULATED_DATA: {
        mavlink_encapsulated_data_t msg;
        mavlink_msg_encapsulated_data_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s ENCAPSULATED_DATA(%d), sysid=%d, compid=%d, seq=%d, seqnr=%d, data=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.seqnr, msg.data);
        break;
    }
    */
    case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
        mavlink_distance_sensor_t msg;
        mavlink_msg_distance_sensor_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s DISTANCE_SENSOR(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, min_distance=%d, max_distance=%d, current_distance=%d, type=%d, id=%d, orientation=%d, covariance=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.min_distance, msg.max_distance, msg.current_distance, msg.type, msg.id, msg.orientation, msg.covariance);
        break;
    }
    case MAVLINK_MSG_ID_TERRAIN_REQUEST: {
        mavlink_terrain_request_t msg;
        mavlink_msg_terrain_request_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s TERRAIN_REQUEST(%d), sysid=%d, compid=%d, seq=%d, lat=%d, lon=%d, grid_spacing=%d, mask=%lld",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.lat, msg.lon, msg.grid_spacing, (long long int)msg.mask);
        break;
    }
    /*
    case MAVLINK_MSG_ID_TERRAIN_DATA: {
        mavlink_terrain_data_t msg;
        mavlink_msg_terrain_data_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s TERRAIN_DATA(%d), sysid=%d, compid=%d, seq=%d, lat=%d, lon=%d, grid_spacing=%d, gridbit=%d, data=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.lat, msg.lon, msg.grid_spacing, msg.gridbit, msg.data);
        break;
    }
    */
    case MAVLINK_MSG_ID_TERRAIN_CHECK: {
        mavlink_terrain_check_t msg;
        mavlink_msg_terrain_check_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s TERRAIN_CHECK(%d), sysid=%d, compid=%d, seq=%d, lat=%d, lon=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.lat, msg.lon);
        break;
    }
    case MAVLINK_MSG_ID_TERRAIN_REPORT: {
        mavlink_terrain_report_t msg;
        mavlink_msg_terrain_report_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s TERRAIN_REPORT(%d), sysid=%d, compid=%d, seq=%d, lat=%d, lon=%d, spacing=%d, terrain_height=%f, current_height=%f, pending=%d, loaded=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.lat, msg.lon, msg.spacing, msg.terrain_height, msg.current_height, msg.pending, msg.loaded);
        break;
    }
    case MAVLINK_MSG_ID_SCALED_PRESSURE2: {
        mavlink_scaled_pressure2_t msg;
        mavlink_msg_scaled_pressure2_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SCALED_PRESSURE2(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, press_abs=%f, press_diff=%f, temperature=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.press_abs, msg.press_diff, msg.temperature);
        break;
    }
    /*
    case MAVLINK_MSG_ID_ATT_POS_MOCAP: {
        mavlink_att_pos_mocap_t msg;
        mavlink_msg_att_pos_mocap_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s ATT_POS_MOCAP(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, q=%d, x=%d, y=%d, z=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_usec, msg.q, msg.x, msg.y, msg.z);
        break;
    }
    */
    /*
    case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET: {
        mavlink_set_actuator_control_target_t msg;
        mavlink_msg_set_actuator_control_target_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SET_ACTUATOR_CONTROL_TARGET(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, group_mlx=%d, target_system=%d, target_component=%d, controls=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_usec, msg.group_mlx, msg.target_system, msg.target_component, msg.controls);
        break;
    }
    */
    /*
    case MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET: {
        mavlink_actuator_control_target_t msg;
        mavlink_msg_actuator_control_target_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s ACTUATOR_CONTROL_TARGET(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, group_mlx=%d, controls=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_usec, msg.group_mlx, msg.controls);
        break;
    }
    */
    case MAVLINK_MSG_ID_ALTITUDE: {
        mavlink_altitude_t msg;
        mavlink_msg_altitude_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s ALTITUDE(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, altitude_monotonic=%f, altitude_amsl=%f, altitude_local=%f, altitude_relative=%f, altitude_terrain=%f, bottom_clearance=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.altitude_monotonic, msg.altitude_amsl, msg.altitude_local, msg.altitude_relative, msg.altitude_terrain, msg.bottom_clearance);
        break;
    }
    /*
    case MAVLINK_MSG_ID_RESOURCE_REQUEST: {
        mavlink_resource_request_t msg;
        mavlink_msg_resource_request_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s RESOURCE_REQUEST(%d), sysid=%d, compid=%d, seq=%d, request_id=%d, uri_type=%d, uri=%d, transfer_type=%d, storage=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.request_id, msg.uri_type, msg.uri, msg.transfer_type, msg.storage);
        break;
    }
    */
    case MAVLINK_MSG_ID_SCALED_PRESSURE3: {
        mavlink_scaled_pressure3_t msg;
        mavlink_msg_scaled_pressure3_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SCALED_PRESSURE3(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, press_abs=%f, press_diff=%f, temperature=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.press_abs, msg.press_diff, msg.temperature);
        break;
    }
    /*
    case MAVLINK_MSG_ID_FOLLOW_TARGET: {
        mavlink_follow_target_t msg;
        mavlink_msg_follow_target_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s FOLLOW_TARGET(%d), sysid=%d, compid=%d, seq=%d, timestamp=%d, est_capabilities=%d, lat=%d, lon=%d, alt=%d, vel=%d, acc=%d, attitude_q=%d, rates=%d, position_cov=%d, custom_state=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.timestamp, msg.est_capabilities, msg.lat, msg.lon, msg.alt, msg.vel, msg.acc, msg.attitude_q, msg.rates, msg.position_cov, msg.custom_state);
        break;
    }
    */
    /*
    case MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE: {
        mavlink_control_system_state_t msg;
        mavlink_msg_control_system_state_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s CONTROL_SYSTEM_STATE(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, x_acc=%d, y_acc=%d, z_acc=%d, x_vel=%d, y_vel=%d, z_vel=%d, x_pos=%d, y_pos=%d, z_pos=%d, airspeed=%d, vel_variance=%d, pos_variance=%d, q=%d, roll_rate=%d, pitch_rate=%d, yaw_rate=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_usec, msg.x_acc, msg.y_acc, msg.z_acc, msg.x_vel, msg.y_vel, msg.z_vel, msg.x_pos, msg.y_pos, msg.z_pos, msg.airspeed, msg.vel_variance, msg.pos_variance, msg.q, msg.roll_rate, msg.pitch_rate, msg.yaw_rate);
        break;
    }
    */
    case MAVLINK_MSG_ID_BATTERY_STATUS: {
        mavlink_battery_status_t msg;
        mavlink_msg_battery_status_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s BATTERY_STATUS(%d), sysid=%d, compid=%d, seq=%d, id=%d, battery_function=%d, type=%d, temperature=%d, voltages=%d, current_battery=%d, current_consumed=%d, energy_consumed=%d, battery_remaining=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.id, msg.battery_function, msg.type, msg.temperature, msg.voltages[0], msg.current_battery, msg.current_consumed, msg.energy_consumed, msg.battery_remaining);
        break;
    }
    case MAVLINK_MSG_ID_AUTOPILOT_VERSION: {
        mavlink_autopilot_version_t msg;
        mavlink_msg_autopilot_version_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s AUTOPILOT_VERSION(%d), sysid=%d, compid=%d, seq=%d, capabilities=%lld, flight_sw_version=%d, middleware_sw_version=%d, os_sw_version=%d, board_version=%d, flight_custom_version=%d, middleware_custom_version=%d, os_custom_version=%d, vendor_id=%d, product_id=%d, uid=%lld",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.capabilities, msg.flight_sw_version, msg.middleware_sw_version, msg.os_sw_version, msg.board_version, msg.flight_custom_version[0], msg.middleware_custom_version[0], msg.os_custom_version[0], msg.vendor_id, msg.product_id, (long long int)msg.uid);
        break;
    }
    case MAVLINK_MSG_ID_LANDING_TARGET: {
        mavlink_landing_target_t msg;
        mavlink_msg_landing_target_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s LANDING_TARGET(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, target_num=%d, frame=%d, angle_x=%f, angle_y=%f, distance=%f, size_x=%f, size_y=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.target_num, msg.frame, msg.angle_x, msg.angle_y, msg.distance, msg.size_x, msg.size_y);
        break;
    }
    case MAVLINK_MSG_ID_ESTIMATOR_STATUS: {
        mavlink_estimator_status_t msg;
        mavlink_msg_estimator_status_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s ESTIMATOR_STATUS(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, flags=%d, vel_ratio=%f, pos_horiz_ratio=%f, pos_vert_ratio=%f, mag_ratio=%f, hagl_ratio=%f, tas_ratio=%f, pos_horiz_accuracy=%f, pos_vert_accuracy=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.flags, msg.vel_ratio, msg.pos_horiz_ratio, msg.pos_vert_ratio, msg.mag_ratio, msg.hagl_ratio, msg.tas_ratio, msg.pos_horiz_accuracy, msg.pos_vert_accuracy);
        break;
    }
    case MAVLINK_MSG_ID_WIND_COV: {
        mavlink_wind_cov_t msg;
        mavlink_msg_wind_cov_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s WIND_COV(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, wind_x=%f, wind_y=%f, wind_z=%f, var_horiz=%f, var_vert=%f, wind_alt=%f, horiz_accuracy=%f, vert_accuracy=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.wind_x, msg.wind_y, msg.wind_z, msg.var_horiz, msg.var_vert, msg.wind_alt, msg.horiz_accuracy, msg.vert_accuracy);
        break;
    }
    case MAVLINK_MSG_ID_GPS_INPUT: {
        mavlink_gps_input_t msg;
        mavlink_msg_gps_input_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s GPS_INPUT(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, gps_id=%d, ignore_flags=%d, time_week_ms=%d, time_week=%d, fix_type=%d, lat=%d, lon=%d, alt=%f, hdop=%f, vdop=%f, vn=%f, ve=%f, vd=%f, speed_accuracy=%f, horiz_accuracy=%f, vert_accuracy=%f, satellites_visible=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.gps_id, msg.ignore_flags, msg.time_week_ms, msg.time_week, msg.fix_type, msg.lat, msg.lon, msg.alt, msg.hdop, msg.vdop, msg.vn, msg.ve, msg.vd, msg.speed_accuracy, msg.horiz_accuracy, msg.vert_accuracy, msg.satellites_visible);
        break;
    }
    /*
    case MAVLINK_MSG_ID_GPS_RTCM_DATA: {
        mavlink_gps_rtcm_data_t msg;
        mavlink_msg_gps_rtcm_data_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s GPS_RTCM_DATA(%d), sysid=%d, compid=%d, seq=%d, flags=%d, len=%d, data=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.flags, msg.len, msg.data);
        break;
    }
    */
    case MAVLINK_MSG_ID_HIGH_LATENCY: {
        mavlink_high_latency_t msg;
        mavlink_msg_high_latency_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s HIGH_LATENCY(%d), sysid=%d, compid=%d, seq=%d, base_mode=%d, custom_mode=%d, landed_state=%d, roll=%d, pitch=%d, heading=%d, throttle=%d, heading_sp=%d, latitude=%d, longitude=%d, altitude_amsl=%d, altitude_sp=%d, airspeed=%d, airspeed_sp=%d, groundspeed=%d, climb_rate=%d, gps_nsat=%d, gps_fix_type=%d, battery_remaining=%d, temperature=%d, temperature_air=%d, failsafe=%d, wp_num=%d, wp_distance=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.base_mode, msg.custom_mode, msg.landed_state, msg.roll, msg.pitch, msg.heading, msg.throttle, msg.heading_sp, msg.latitude, msg.longitude, msg.altitude_amsl, msg.altitude_sp, msg.airspeed, msg.airspeed_sp, msg.groundspeed, msg.climb_rate, msg.gps_nsat, msg.gps_fix_type, msg.battery_remaining, msg.temperature, msg.temperature_air, msg.failsafe, msg.wp_num, msg.wp_distance);
        break;
    }
    case MAVLINK_MSG_ID_VIBRATION: {
        mavlink_vibration_t msg;
        mavlink_msg_vibration_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s VIBRATION(%d), sysid=%d, compid=%d, seq=%d, time_usec=%lld, vibration_x=%f, vibration_y=%f, vibration_z=%f, clipping_0=%d, clipping_1=%d, clipping_2=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, (long long int)msg.time_usec, msg.vibration_x, msg.vibration_y, msg.vibration_z, msg.clipping_0, msg.clipping_1, msg.clipping_2);
        break;
    }
    case MAVLINK_MSG_ID_HOME_POSITION: {
        mavlink_home_position_t msg;
        mavlink_msg_home_position_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s HOME_POSITION(%d), sysid=%d, compid=%d, seq=%d, latitude=%d, longitude=%d, altitude=%d, x=%f, y=%f, z=%f, q=%f, approach_x=%f, approach_y=%f, approach_z=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.latitude, msg.longitude, msg.altitude, msg.x, msg.y, msg.z, msg.q[0], msg.approach_x, msg.approach_y, msg.approach_z);
        break;
    }
    case MAVLINK_MSG_ID_SET_HOME_POSITION: {
        mavlink_set_home_position_t msg;
        mavlink_msg_set_home_position_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SET_HOME_POSITION(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, latitude=%d, longitude=%d, altitude=%d, x=%f, y=%f, z=%f, q=%f, approach_x=%f, approach_y=%f, approach_z=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.latitude, msg.longitude, msg.altitude, msg.x, msg.y, msg.z, msg.q[0], msg.approach_x, msg.approach_y, msg.approach_z);
        break;
    }
    case MAVLINK_MSG_ID_MESSAGE_INTERVAL: {
        mavlink_message_interval_t msg;
        mavlink_msg_message_interval_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MESSAGE_INTERVAL(%d), sysid=%d, compid=%d, seq=%d, message_id=%d, interval_us=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.message_id, msg.interval_us);
        break;
    }
    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
        mavlink_extended_sys_state_t msg;
        mavlink_msg_extended_sys_state_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s EXTENDED_SYS_STATE(%d), sysid=%d, compid=%d, seq=%d, vtol_state=%d, landed_state=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.vtol_state, msg.landed_state);
        break;
    }
    case MAVLINK_MSG_ID_ADSB_VEHICLE: {
        mavlink_adsb_vehicle_t msg;
        mavlink_msg_adsb_vehicle_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s ADSB_VEHICLE(%d), sysid=%d, compid=%d, seq=%d, ICAO_address=%d, lat=%d, lon=%d, altitude_type=%d, altitude=%d, heading=%d, hor_velocity=%d, ver_velocity=%d, callsign=%.9s, emitter_type=%d, tslc=%d, flags=%d, squawk=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.ICAO_address, msg.lat, msg.lon, msg.altitude_type, msg.altitude, msg.heading, msg.hor_velocity, msg.ver_velocity, msg.callsign, msg.emitter_type, msg.tslc, msg.flags, msg.squawk);
        break;
    }
    case MAVLINK_MSG_ID_COLLISION: {
        mavlink_collision_t msg;
        mavlink_msg_collision_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s COLLISION(%d), sysid=%d, compid=%d, seq=%d, src=%d, id=%d, action=%d, threat_level=%d, time_to_minimum_delta=%f, altitude_minimum_delta=%f, horizontal_minimum_delta=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.src, msg.id, msg.action, msg.threat_level, msg.time_to_minimum_delta, msg.altitude_minimum_delta, msg.horizontal_minimum_delta);
        break;
    }
    /*
    case MAVLINK_MSG_ID_V2_EXTENSION: {
        mavlink_v2_extension_t msg;
        mavlink_msg_v2_extension_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s V2_EXTENSION(%d), sysid=%d, compid=%d, seq=%d, target_network=%d, target_system=%d, target_component=%d, message_type=%d, payload=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_network, msg.target_system, msg.target_component, msg.message_type, msg.payload);
        break;
    }
    */
    /*
    case MAVLINK_MSG_ID_MEMORY_VECT: {
        mavlink_memory_vect_t msg;
        mavlink_msg_memory_vect_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MEMORY_VECT(%d), sysid=%d, compid=%d, seq=%d, address=%d, ver=%d, type=%d, value=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.address, msg.ver, msg.type, msg.value);
        break;
    }
    */
    case MAVLINK_MSG_ID_DEBUG_VECT: {
        mavlink_debug_vect_t msg;
        mavlink_msg_debug_vect_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s DEBUG_VECT(%d), sysid=%d, compid=%d, seq=%d, name=%.10s, time_usec=%lld, x=%f, y=%f, z=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.name, (long long int)msg.time_usec, msg.x, msg.y, msg.z);
        break;
    }
    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: {
        mavlink_named_value_float_t msg;
        mavlink_msg_named_value_float_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s NAMED_VALUE_FLOAT(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, name=%.10s, value=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.name, msg.value);
        break;
    }
    case MAVLINK_MSG_ID_NAMED_VALUE_INT: {
        mavlink_named_value_int_t msg;
        mavlink_msg_named_value_int_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s NAMED_VALUE_INT(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, name=%.10s, value=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.name, msg.value);
        break;
    }
    case MAVLINK_MSG_ID_STATUSTEXT: {
        mavlink_statustext_t msg;
        mavlink_msg_statustext_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s STATUSTEXT(%d), sysid=%d, compid=%d, seq=%d, severity=%d, text=%.50s",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.severity, msg.text);
        break;
    }
    case MAVLINK_MSG_ID_DEBUG: {
        mavlink_debug_t msg;
        mavlink_msg_debug_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s DEBUG(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, ind=%d, value=%f",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.ind, msg.value);
        break;
    }
    case MAVLINK_MSG_ID_BATTERY2: {
        mavlink_battery2_t msg;
        mavlink_msg_battery2_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s BATTERY2(%d), sysid=%d, compid=%d, seq=%d, voltage=%d, current_battery=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.voltage, msg.current_battery);
        break;
    }
    /*
    case MAVLINK_MSG_ID_SETUP_SIGNING: {
        mavlink_setup_signing_t msg;
        mavlink_msg_setup_signing_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s SETUP_SIGNING(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, secret_key=%d, initial_timestamp=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.secret_key, msg.initial_timestamp);
        break;
    }
    case MAVLINK_MSG_ID_BUTTON_CHANGE: {
        mavlink_button_change_t msg;
        mavlink_msg_button_change_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s BUTTON_CHANGE(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, last_change_ms=%d, state=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.last_change_ms, msg.state);
        break;
    }
    case MAVLINK_MSG_ID_PLAY_TUNE: {
        mavlink_play_tune_t msg;
        mavlink_msg_play_tune_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s PLAY_TUNE(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, tune=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.tune);
        break;
    }
    case MAVLINK_MSG_ID_CAMERA_INFORMATION: {
        mavlink_camera_information_t msg;
        mavlink_msg_camera_information_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s CAMERA_INFORMATION(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, camera_id=%d, vendor_name=%d, model_name=%d, focal_length=%d, sensor_size_h=%d, sensor_size_v=%d, resolution_h=%d, resolution_v=%d, lense_id=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.camera_id, msg.vendor_name, msg.model_name, msg.focal_length, msg.sensor_size_h, msg.sensor_size_v, msg.resolution_h, msg.resolution_v, msg.lense_id);
        break;
    }
    case MAVLINK_MSG_ID_CAMERA_SETTINGS: {
        mavlink_camera_settings_t msg;
        mavlink_msg_camera_settings_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s CAMERA_SETTINGS(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, camera_id=%d, aperture=%d, aperture_locked=%d, shutter_speed=%d, shutter_speed_locked=%d, iso_sensitivity=%d, iso_sensitivity_locked=%d, white_balance=%d, white_balance_locked=%d, mode_id=%d, color_mode_id=%d, image_format_id=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.camera_id, msg.aperture, msg.aperture_locked, msg.shutter_speed, msg.shutter_speed_locked, msg.iso_sensitivity, msg.iso_sensitivity_locked, msg.white_balance, msg.white_balance_locked, msg.mode_id, msg.color_mode_id, msg.image_format_id);
        break;
    }
    case MAVLINK_MSG_ID_STORAGE_INFORMATION: {
        mavlink_storage_information_t msg;
        mavlink_msg_storage_information_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s STORAGE_INFORMATION(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, storage_id=%d, status=%d, total_capacity=%d, used_capacity=%d, available_capacity=%d, read_speed=%d, write_speed=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.storage_id, msg.status, msg.total_capacity, msg.used_capacity, msg.available_capacity, msg.read_speed, msg.write_speed);
        break;
    }
    case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS: {
        mavlink_camera_capture_status_t msg;
        mavlink_msg_camera_capture_status_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s CAMERA_CAPTURE_STATUS(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, camera_id=%d, image_status=%d, video_status=%d, image_interval=%d, video_framerate=%d, image_resolution_h=%d, image_resolution_v=%d, video_resolution_h=%d, video_resolution_v=%d, recording_time_ms=%d, available_capacity=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.camera_id, msg.image_status, msg.video_status, msg.image_interval, msg.video_framerate, msg.image_resolution_h, msg.image_resolution_v, msg.video_resolution_h, msg.video_resolution_v, msg.recording_time_ms, msg.available_capacity);
        break;
    }
    case MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED: {
        mavlink_camera_image_captured_t msg;
        mavlink_msg_camera_image_captured_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s CAMERA_IMAGE_CAPTURED(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, time_utc=%d, camera_id=%d, lat=%d, lon=%d, alt=%d, relative_alt=%d, q=%d, image_index=%d, capture_result=%d, file_url=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.time_utc, msg.camera_id, msg.lat, msg.lon, msg.alt, msg.relative_alt, msg.q, msg.image_index, msg.capture_result, msg.file_url);
        break;
    }
    case MAVLINK_MSG_ID_FLIGHT_INFORMATION: {
        mavlink_flight_information_t msg;
        mavlink_msg_flight_information_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s FLIGHT_INFORMATION(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, arming_time_utc=%d, takeoff_time_utc=%d, flight_uuid=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.arming_time_utc, msg.takeoff_time_utc, msg.flight_uuid);
        break;
    }
    case MAVLINK_MSG_ID_MOUNT_ORIENTATION: {
        mavlink_mount_orientation_t msg;
        mavlink_msg_mount_orientation_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s MOUNT_ORIENTATION(%d), sysid=%d, compid=%d, seq=%d, time_boot_ms=%d, roll=%d, pitch=%d, yaw=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw);
        break;
    }
    case MAVLINK_MSG_ID_LOGGING_DATA: {
        mavlink_logging_data_t msg;
        mavlink_msg_logging_data_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s LOGGING_DATA(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, sequence=%d, length=%d, first_message_offset=%d, data=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.sequence, msg.length, msg.first_message_offset, msg.data);
        break;
    }
    case MAVLINK_MSG_ID_LOGGING_DATA_ACKED: {
        mavlink_logging_data_acked_t msg;
        mavlink_msg_logging_data_acked_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s LOGGING_DATA_ACKED(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, sequence=%d, length=%d, first_message_offset=%d, data=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.sequence, msg.length, msg.first_message_offset, msg.data);
        break;
    }
    case MAVLINK_MSG_ID_LOGGING_ACK: {
        mavlink_logging_ack_t msg;
        mavlink_msg_logging_ack_decode(&message, &msg);
        snprintf(buff, sizeof(buff), "%s LOGGING_ACK(%d), sysid=%d, compid=%d, seq=%d, target_system=%d, target_component=%d, sequence=%d",
                 prefix, message.msgid, message.sysid, message.compid, message.seq, msg.target_system, msg.target_component, msg.sequence);
        break;
    }
    */
    default:
        snprintf(buff, sizeof(buff), "%s msgid=%d, sysid=%d, compid=%d, seq=%d",
                 prefix, message.msgid, message.sysid, message.compid,
                 message.seq);
        break;
    }

    syslog(priority, "%s", buff);
}
