/*
 HighLatencyMsg.cpp

 Iridium SBD telemetry for ArduPilot.
 
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
 */

#include "Arduino.h"
#include "HighLatencyMsg.h"

#ifndef M_PI
#define M_PI 3.14159265359
#endif

inline int16_t radToCentidegrees(float rad) {
  return rad / M_PI * 18000;
}

HighLatencyMsg::HighLatencyMsg(uint8_t sysid, uint8_t compid) : 
                               seq(0), sysid(sysid), compid(compid)
{
  memset(&highLatency, 0, sizeof(highLatency));
  highLatency.gps_nsat = 255;
  highLatency.landed_state = MAV_LANDED_STATE_UNDEFINED;
}
  
bool HighLatencyMsg::update(const mavlink_message_t& msg)
{
  switch (msg.msgid) {
  case MAVLINK_MSG_ID_HEARTBEAT:    //0
    highLatency.base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
    highLatency.custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
    return true;
  case MAVLINK_MSG_ID_SYS_STATUS:   //1
    highLatency.battery_remaining = mavlink_msg_sys_status_get_battery_remaining(&msg);
    return true;
  case MAVLINK_MSG_ID_GPS_RAW_INT:    //24
    highLatency.latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
    highLatency.longitude = mavlink_msg_gps_raw_int_get_lon(&msg);
    highLatency.altitude_amsl = mavlink_msg_gps_raw_int_get_alt(&msg) / 1000;
    highLatency.gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
    highLatency.gps_nsat = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
    return true;
  case MAVLINK_MSG_ID_ATTITUDE:   //30
    highLatency.heading = radToCentidegrees(mavlink_msg_attitude_get_yaw(&msg));
    highLatency.roll = radToCentidegrees(mavlink_msg_attitude_get_roll(&msg));
    highLatency.pitch = radToCentidegrees(mavlink_msg_attitude_get_pitch(&msg));
    return true;
  case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    return true;
  case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:    //33
    //highLatency.latitude = mavlink_msg_global_position_int_get_lat(&msg);
    //highLatency.longitude = mavlink_msg_global_position_int_get_lon(&msg);
    //highLatency.altitude_amsl = mavlink_msg_global_position_int_get_alt(&msg);
    highLatency.altitude_sp = mavlink_msg_global_position_int_get_relative_alt(&msg);
    return true;
  case MAVLINK_MSG_ID_MISSION_CURRENT:    //42
    highLatency.wp_num = mavlink_msg_mission_current_get_seq(&msg);
    return true;
  case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:    //62
    highLatency.wp_distance = mavlink_msg_nav_controller_output_get_wp_dist(&msg);
    highLatency.heading_sp = mavlink_msg_nav_controller_output_get_nav_bearing(&msg) * 100;
    return true;
  case MAVLINK_MSG_ID_VFR_HUD:    //74
    highLatency.airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
    highLatency.groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
    //highLatency.heading = mavlink_msg_vfr_hud_get_heading(&msg) * 100;
    highLatency.climb_rate = mavlink_msg_vfr_hud_get_climb(&msg);
    highLatency.throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
    return true;
  }

  return false;
}

void HighLatencyMsg::print()
{
  Serial.println("**");
  Serial.print("custom_mode = "); Serial.println(highLatency.custom_mode);
  Serial.print("latitude = "); Serial.println(highLatency.latitude);
  Serial.print("longitude = "); Serial.println(highLatency.longitude);
  Serial.print("roll = "); Serial.println(highLatency.roll);
  Serial.print("pitch = "); Serial.println(highLatency.pitch);
  Serial.print("heading = "); Serial.println(highLatency.heading);
  Serial.print("heading_sp = "); Serial.println(highLatency.heading_sp);
  Serial.print("altitude_amsl = "); Serial.println(highLatency.altitude_amsl);
  Serial.print("altitude_sp = "); Serial.println(highLatency.altitude_sp);
  Serial.print("wp_distance = "); Serial.println(highLatency.wp_distance);
  Serial.print("base_mode = "); Serial.println(highLatency.base_mode);
  Serial.print("landed_state = "); Serial.println(highLatency.landed_state);
  Serial.print("throttle = "); Serial.println(highLatency.throttle);
  Serial.print("airspeed = "); Serial.println(highLatency.airspeed);
  Serial.print("airspeed_sp = "); Serial.println(highLatency.airspeed_sp);
  Serial.print("groundspeed = "); Serial.println(highLatency.groundspeed);
  Serial.print("climb_rate = "); Serial.println(highLatency.climb_rate);
  Serial.print("gps_nsat = "); Serial.println(highLatency.gps_nsat);
  Serial.print("gps_fix_type = "); Serial.println(highLatency.gps_fix_type);
  Serial.print("battery_remaining = "); Serial.println(highLatency.battery_remaining);
  Serial.print("temperature = "); Serial.println(highLatency.temperature);
  Serial.print("temperature_air = "); Serial.println(highLatency.temperature_air);
  Serial.print("failsafe = "); Serial.println(highLatency.failsafe);
  Serial.print("wp_num = "); Serial.println(highLatency.wp_num);
}

void HighLatencyMsg::encode(mavlink_message_t& msg) 
{
  mavlink_msg_high_latency_encode(sysid, compid, &msg, &highLatency);
  msg.seq = seq++;
}

