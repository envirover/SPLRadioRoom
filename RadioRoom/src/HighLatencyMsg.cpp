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

#include <string.h>
#include <stdio.h>
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
    highLatency.groundspeed = mavlink_msg_gps_raw_int_get_vel(&msg) / 100;
    highLatency.gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
    highLatency.gps_nsat = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
    return true;
  case MAVLINK_MSG_ID_ATTITUDE:   //30
    highLatency.heading = (radToCentidegrees(mavlink_msg_attitude_get_yaw(&msg)) + 36000) % 36000;
    highLatency.roll = radToCentidegrees(mavlink_msg_attitude_get_roll(&msg));
    highLatency.pitch = radToCentidegrees(mavlink_msg_attitude_get_pitch(&msg));
    return true;
  case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
    return true;
  case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:    //33
    //highLatency.latitude = mavlink_msg_global_position_int_get_lat(&msg);
    //highLatency.longitude = mavlink_msg_global_position_int_get_lon(&msg);
    //highLatency.altitude_amsl = mavlink_msg_global_position_int_get_alt(&msg) / 1000;
    highLatency.altitude_sp = mavlink_msg_global_position_int_get_relative_alt(&msg) / 1000;
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
  printf("**");
  printf("custom_mode = %d\n", highLatency.custom_mode);
  printf("latitude = %d\n", highLatency.latitude);
  printf("longitude = %d\n", highLatency.longitude);
  printf("roll = %d\n", highLatency.roll);
  printf("pitch = %d\n", highLatency.pitch);
  printf("heading = %d\n", highLatency.heading);
  printf("heading_sp = %d\n", highLatency.heading_sp);
  printf("altitude_amsl = %d\n", highLatency.altitude_amsl);
  printf("altitude_sp = %d\n", highLatency.altitude_sp);
  printf("wp_distance = %d\n", highLatency.wp_distance);
  printf("base_mode = %d\n", highLatency.base_mode);
  printf("landed_state = %d\n", highLatency.landed_state);
  printf("throttle = %d\n", highLatency.throttle);
  printf("airspeed = %d\n", highLatency.airspeed);
  printf("airspeed_sp = %d\n", highLatency.airspeed_sp);
  printf("groundspeed = %d\n", highLatency.groundspeed);
  printf("climb_rate = %d\n", highLatency.climb_rate);
  printf("gps_nsat = %d\n", highLatency.gps_nsat);
  printf("gps_fix_type = %d\n", highLatency.gps_fix_type);
  printf("battery_remaining = %d\n", highLatency.battery_remaining);
  printf("temperature = %d\n", highLatency.temperature);
  printf("temperature_air = %d\n", highLatency.temperature_air);
  printf("failsafe = %d\n", highLatency.failsafe);
  printf("wp_num = %d\n", highLatency.wp_num);
}

void HighLatencyMsg::encode(mavlink_message_t& msg) 
{
  mavlink_msg_high_latency_encode(sysid, compid, &msg, &highLatency);
  msg.seq = seq++;
}

