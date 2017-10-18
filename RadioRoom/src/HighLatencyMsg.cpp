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
#include <math.h>
#include "HighLatencyMsg.h"

inline int16_t radToCentidegrees(float rad) {
  return rad / M_PI * 18000;
}

HighLatencyMsg::HighLatencyMsg(uint8_t sysid, uint8_t compid) : 
                               seq(0), sysid(sysid), compid(compid)
{
  memset(&high_latency, 0, sizeof(high_latency));
  high_latency.gps_nsat = 255;
  high_latency.landed_state = MAV_LANDED_STATE_UNDEFINED;
}
  
bool HighLatencyMsg::update(const mavlink_message_t& msg)
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

void HighLatencyMsg::print()
{
  printf("**");
  printf("custom_mode = %d\n", high_latency.custom_mode);
  printf("latitude = %d\n", high_latency.latitude);
  printf("longitude = %d\n", high_latency.longitude);
  printf("roll = %d\n", high_latency.roll);
  printf("pitch = %d\n", high_latency.pitch);
  printf("heading = %d\n", high_latency.heading);
  printf("heading_sp = %d\n", high_latency.heading_sp);
  printf("altitude_amsl = %d\n", high_latency.altitude_amsl);
  printf("altitude_sp = %d\n", high_latency.altitude_sp);
  printf("wp_distance = %d\n", high_latency.wp_distance);
  printf("base_mode = %d\n", high_latency.base_mode);
  printf("landed_state = %d\n", high_latency.landed_state);
  printf("throttle = %d\n", high_latency.throttle);
  printf("airspeed = %d\n", high_latency.airspeed);
  printf("airspeed_sp = %d\n", high_latency.airspeed_sp);
  printf("groundspeed = %d\n", high_latency.groundspeed);
  printf("climb_rate = %d\n", high_latency.climb_rate);
  printf("gps_nsat = %d\n", high_latency.gps_nsat);
  printf("gps_fix_type = %d\n", high_latency.gps_fix_type);
  printf("battery_remaining = %d\n", high_latency.battery_remaining);
  printf("temperature = %d\n", high_latency.temperature);
  printf("temperature_air = %d\n", high_latency.temperature_air);
  printf("failsafe = %d\n", high_latency.failsafe);
  printf("wp_num = %d\n", high_latency.wp_num);
}

void HighLatencyMsg::encode(mavlink_message_t& msg) 
{
  mavlink_msg_high_latency_encode(sysid, compid, &msg, &high_latency);
  msg.seq = seq++;
}

