/*
 MAVReport.cc

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
*/

#include "MAVReport.h"

namespace radioroom {

// Masks of MAVLink messages used to compose single HIGH_LATENCY message
constexpr uint16_t mavlink_msg_mask_heartbeat = 0x0001;
constexpr uint16_t mavlink_msg_mask_sys_status = 0x0002;
constexpr uint16_t mavlink_msg_mask_gps_raw_int = 0x0004;
constexpr uint16_t mavlink_msg_mask_attitude = 0x0008;
constexpr uint16_t mavlink_msg_mask_global_position_int = 0x0010;
constexpr uint16_t mavlink_msg_mask_mission_current = 0x0020;
constexpr uint16_t mavlink_msg_mask_nav_controller_output = 0x0040;
constexpr uint16_t mavlink_msg_mask_vfr_hud = 0x0080;
constexpr uint16_t mavlink_msg_mask_battery2 = 0x0100;  // optional

constexpr uint16_t mavlink_msg_mask_high_latency = 0x00FF;

inline int16_t rad_to_centidegrees(float rad) {
  return rad * 18000.0 / 3.14159265358979323846;
}

MAVReport::MAVReport() : sysid(1), compid(0), mask(0) {}

/**
 * Integrates data from the specified MAVLink message into the HIGH_LATENCY
 * message.
 */
bool MAVReport::update(const mavlink_message_t& msg) {
  sysid = msg.sysid;
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:  // 0
      report.base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
      report.custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
      compid = msg.compid;
      mask |= mavlink_msg_mask_heartbeat;
      return true;
    case MAVLINK_MSG_ID_SYS_STATUS:  // 1
      report.battery_remaining =
          mavlink_msg_sys_status_get_battery_remaining(&msg);
      report.temperature =
          mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000;
      mask |= mavlink_msg_mask_sys_status;
      return true;
    case MAVLINK_MSG_ID_GPS_RAW_INT:  // 24
      // report_msg.latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
      // report_msg.longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
      // report.altitude_amsl = mavlink_msg_gps_raw_int_get_alt(&msg) / 1000;
      report.groundspeed = mavlink_msg_gps_raw_int_get_vel(&msg) / 100;
      report.gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
      report.gps_nsat = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
      mask |= mavlink_msg_mask_gps_raw_int;
      return true;
    case MAVLINK_MSG_ID_ATTITUDE:  // 30
      report.heading =
          (rad_to_centidegrees(mavlink_msg_attitude_get_yaw(&msg)) + 36000) %
          36000;
      report.roll = rad_to_centidegrees(mavlink_msg_attitude_get_roll(&msg));
      report.pitch = rad_to_centidegrees(mavlink_msg_attitude_get_pitch(&msg));
      mask |= mavlink_msg_mask_attitude;
      return true;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  // 33
      report.latitude = mavlink_msg_global_position_int_get_lat(&msg);
      report.longitude = mavlink_msg_global_position_int_get_lon(&msg);
      high_latency.altitude_amsl = mavlink_msg_global_position_int_get_alt(&msg) / 1000;
      report.altitude_sp =
          mavlink_msg_global_position_int_get_relative_alt(&msg) / 1000;
      mask |= mavlink_msg_mask_global_position_int;
      return true;
    case MAVLINK_MSG_ID_MISSION_CURRENT:  // 42
      report.wp_num = mavlink_msg_mission_current_get_seq(&msg);
      mask |= mavlink_msg_mask_mission_current;
      return true;
    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:  // 62
      report.wp_distance = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
      report.heading_sp =
          mavlink_msg_nav_controller_output_get_nav_bearing(&msg) * 100;
      mask |= mavlink_msg_mask_nav_controller_output;
      return true;
    case MAVLINK_MSG_ID_VFR_HUD:  // 74
      report.airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
      report.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
      // high_latency.heading = mavlink_msg_vfr_hud_get_heading(&msg) * 100;
      report.climb_rate = mavlink_msg_vfr_hud_get_climb(&msg);
      report.throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
      mask |= mavlink_msg_mask_vfr_hud;
      return true;
    case MAVLINK_MSG_ID_BATTERY2:  // 147
      uint16_t batt2_voltage = mavlink_msg_battery2_get_voltage(&msg);
      report.temperature_air = batt2_voltage / 1000;
      mask |= mavlink_msg_mask_battery2;
      return true;
  }

  return false;
}

void MAVReport::get_message(mavlink_message_t& msg) const {
  mavlink_msg_high_latency_encode(sysid, compid, &msg, &report);
}

}  // namespace radioroom
