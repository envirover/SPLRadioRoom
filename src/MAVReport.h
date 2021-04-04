/*
 MAVReport.h

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
#ifndef SRC_MAVREPORT_H_
#define SRC_MAVREPORT_H_

#include "MAVLinkLib.h"

namespace radioroom {

// Masks of MAVLink messages used to compose single HIGH_LATENCY2 message
constexpr uint16_t mavlink_msg_mask_heartbeat = 0x0001;
constexpr uint16_t mavlink_msg_mask_sys_status = 0x0002;
constexpr uint16_t mavlink_msg_mask_gps_raw_int = 0x0004;
constexpr uint16_t mavlink_msg_mask_attitude = 0x0008;
constexpr uint16_t mavlink_msg_mask_global_position_int = 0x0010;
constexpr uint16_t mavlink_msg_mask_mission_current = 0x0020;
constexpr uint16_t mavlink_msg_mask_nav_controller_output = 0x0040;
constexpr uint16_t mavlink_msg_mask_vfr_hud = 0x0080;
constexpr uint16_t mavlink_msg_mask_battery2 = 0x0100;         // optional
constexpr uint16_t mavlink_msg_mask_wind = 0x0200;             // optional
constexpr uint16_t mavlink_msg_mask_scaled_pressure = 0x0400;  // optional

// Class used to update and retrieve vehicle state report.
class MAVReport {
 public:
  MAVReport();

  // Integrates the specified message into report message of HIGH_LATENCY2 type.
  //
  // Returns true if the message was integrated.
  bool update(const mavlink_message_t& msg);

  // Retrieves HIGH_LATENCY2 report message.
  // Set custom2 HIGH_LATENCY2.field to the channel_id value.
  // msg_mask identifies MAVLink messages used to compose HIGH_LATENCY2 message.
  void get_message(uint8_t channel_id, mavlink_message_t& msg,
                   uint16_t& msg_mask);

 private:
  // Constructs 2 byte HIGH_LATENCY2.custom_mode from HEARTBEAT.base_mode and
  // HEARTBEAT.custom_mode.
  uint16_t get_custom_mode(uint8_t base_mode, uint32_t custom_mode) const;

  // Constructs HIGH_LATENCY2.failure_flags bitmap from
  // SYS_STATUS.onboard_control_sensors_health
  uint16_t get_failure_flags(uint32_t sensors_health) const;

  mavlink_high_latency2_t report;
  uint8_t sysid;
  uint8_t compid;
  uint16_t mask;
};

}  // namespace radioroom

#endif  // SRC_MAVREPORT_H_
