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

// Class used to update and retrieve vehicle state report.
class MAVReport {
 public:
  MAVReport();

  // Integrates the specified message into report message of HIGH_LATENCY type.
  //
  // Returns true if the message was integrated.
  bool update(const mavlink_message_t& msg);

  // Retrieves HIGH_LATENCY report message
  void get_message(mavlink_message_t& msg) const;

 private:
  mavlink_high_latency_t report;
  uint8_t sysid;
  uint8_t compid;
  uint16_t mask;
};

}  // namespace radioroom

#endif  // SRC_MAVREPORT_H_
