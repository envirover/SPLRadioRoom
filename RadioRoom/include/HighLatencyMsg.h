/*
 HighLatencyMsg.h

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

#ifndef HIGHLATENCYMSG_H_
#define HIGHLATENCYMSG_H_

#undef F
#include "mavlink/include/standard/mavlink.h"

/**
 * Wrapper for mavlink_high_latency_t class.
 */
class HighLatencyMsg
{
  mavlink_high_latency_t high_latency;
  uint8_t seq;
  uint8_t sysid;
  uint8_t compid;
  
public:
  HighLatencyMsg(uint8_t sysid, uint8_t compid);
  
 /*
  * Integrates high frequency message into HIGH_LATENCY type message.
  * 
  * @param msg message received from Ardupilot
  * @return true if the message was integrated or should be just swallowed
  */
  bool update(const mavlink_message_t& msg);

  // Debug print of HIGH_LATENCY message
  void print();

  //Encodes HIGH_LATENCY message
  void encode(mavlink_message_t& msg);
};

#endif // HIGHLATENCYMSG_H_
