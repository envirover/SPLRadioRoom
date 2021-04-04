/*
 MAVReport.cc

 Telemetry for MAVLink autopilots.

 (C) Copyright 2021 Envirover.

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

constexpr uint16_t mavlink_msg_mask_high_latency = 0x00FF;

inline int16_t rad_to_centidegrees(float rad) {
  return rad * 18000.0 / 3.14159265358979323846;
}

MAVReport::MAVReport() : sysid(1), compid(0), mask(0) {}

/**
 * Integrates data from the specified MAVLink message into the HIGH_LATENCY2
 * message.
 */
bool MAVReport::update(const mavlink_message_t& msg) {
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:  // 0
      report.type = mavlink_msg_heartbeat_get_type(&msg);
      report.autopilot = mavlink_msg_heartbeat_get_autopilot(&msg);
      report.custom_mode = get_custom_mode(
          mavlink_msg_heartbeat_get_base_mode(&msg),
          mavlink_msg_heartbeat_get_custom_mode(&msg));
      sysid = msg.sysid;
      compid = msg.compid;
      mask |= mavlink_msg_mask_heartbeat;
      return true;
    case MAVLINK_MSG_ID_SYS_STATUS:  // 1
      report.battery = mavlink_msg_sys_status_get_battery_remaining(&msg);
      report.custom0 =
          mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000;
      report.failure_flags = get_failure_flags(
          mavlink_msg_sys_status_get_onboard_control_sensors_health(&msg));
      mask |= mavlink_msg_mask_sys_status;
      return true;
    case MAVLINK_MSG_ID_GPS_RAW_INT:  // 24
      report.groundspeed = mavlink_msg_gps_raw_int_get_vel(&msg) / 100 * 5;
      report.eph = mavlink_msg_gps_raw_int_get_eph(&msg);
      report.epv = mavlink_msg_gps_raw_int_get_epv(&msg);
      mask |= mavlink_msg_mask_gps_raw_int;
      return true;
    case  MAVLINK_MSG_ID_SCALED_PRESSURE:  // 29
      report.temperature_air =
          mavlink_msg_scaled_pressure_get_temperature(&msg) / 100;
      mask |= mavlink_msg_mask_scaled_pressure;
      return true;
    case MAVLINK_MSG_ID_ATTITUDE:  // 30
      report.heading =
          ((rad_to_centidegrees(mavlink_msg_attitude_get_yaw(&msg)) + 36000) %
          36000) / 2;
      mask |= mavlink_msg_mask_attitude;
      return true;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  // 33
      report.timestamp = mavlink_msg_global_position_int_get_time_boot_ms(&msg);
      report.latitude = mavlink_msg_global_position_int_get_lat(&msg);
      report.longitude = mavlink_msg_global_position_int_get_lon(&msg);
      report.altitude = mavlink_msg_global_position_int_get_alt(&msg) / 1000;
      report.target_altitude =
          mavlink_msg_global_position_int_get_relative_alt(&msg) / 1000;
      mask |= mavlink_msg_mask_global_position_int;
      return true;
    case MAVLINK_MSG_ID_MISSION_CURRENT:  // 42
      report.wp_num = mavlink_msg_mission_current_get_seq(&msg);
      mask |= mavlink_msg_mask_mission_current;
      return true;
    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:  // 62
      report.target_distance =
          mavlink_msg_nav_controller_output_get_wp_dist(&msg);
      report.target_heading =
          mavlink_msg_nav_controller_output_get_nav_bearing(&msg) / 2;
      mask |= mavlink_msg_mask_nav_controller_output;
      return true;
    case MAVLINK_MSG_ID_VFR_HUD:  // 74
      report.airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg) * 5;
      report.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg) * 5;
      report.climb_rate = mavlink_msg_vfr_hud_get_climb(&msg) * 10;
      report.throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
      mask |= mavlink_msg_mask_vfr_hud;
      return true;
    case MAVLINK_MSG_ID_BATTERY2:  // 147
      report.custom1 = mavlink_msg_battery2_get_voltage(&msg) / 1000;
      mask |= mavlink_msg_mask_battery2;
      return true;
    case MAVLINK_MSG_ID_WIND:  // 168
      report.wind_heading = mavlink_msg_wind_get_direction(&msg) / 2;
      report.windspeed = mavlink_msg_wind_get_speed(&msg) * 5;
      mask |= mavlink_msg_mask_wind;
      return true;
  }

  return false;
}

void MAVReport::get_message(uint8_t channel_id, mavlink_message_t& msg,
                            uint16_t& msg_mask) {
  report.custom2 = channel_id;
  msg_mask = mask;
  mavlink_msg_high_latency2_encode(sysid, compid, &msg, &report);
}

uint16_t MAVReport::get_custom_mode(uint8_t base_mode,
                                    uint32_t custom_mode) const {
  return base_mode | ((custom_mode & 0xFF) << 8);
}

uint16_t MAVReport::get_failure_flags(uint32_t sensors_health) const {
  uint16_t failure_flags = 0;

  if (sensors_health & MAV_SYS_STATUS_SENSOR_GPS) {
    failure_flags |= HL_FAILURE_FLAG_GPS;
  }

  if (sensors_health & MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE) {
    failure_flags |= HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE;
  }

  if (sensors_health & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE) {
    failure_flags |= HL_FAILURE_FLAG_ABSOLUTE_PRESSURE;
  }

  if (sensors_health & MAV_SYS_STATUS_SENSOR_3D_ACCEL) {
    failure_flags |= HL_FAILURE_FLAG_3D_ACCEL;
  }

  if (sensors_health & MAV_SYS_STATUS_SENSOR_3D_GYRO) {
    failure_flags |= HL_FAILURE_FLAG_3D_GYRO;
  }

  if (sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG) {
    failure_flags |= HL_FAILURE_FLAG_3D_MAG;
  }

  if (sensors_health & MAV_SYS_STATUS_TERRAIN) {
    failure_flags |= HL_FAILURE_FLAG_TERRAIN;
  }

  if (sensors_health & MAV_SYS_STATUS_SENSOR_BATTERY) {
    failure_flags |= HL_FAILURE_FLAG_BATTERY;
  }

  if (sensors_health & MAV_SYS_STATUS_SENSOR_RC_RECEIVER) {
    failure_flags |= HL_FAILURE_FLAG_RC_RECEIVER;
  }

  if (sensors_health & MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS) {
    failure_flags |= HL_FAILURE_FLAG_ENGINE;
  }

  if (sensors_health & MAV_SYS_STATUS_GEOFENCE) {
    failure_flags |= HL_FAILURE_FLAG_GEOFENCE;
  }

  return failure_flags;
}

}  // namespace radioroom

