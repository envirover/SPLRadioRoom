/*
 MAVLinkLogger.h

 This file is a part of SPL RadioRoom project.

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

#ifndef MAVLINKLOGGER_H_
#define MAVLINKLOGGER_H_

#include <syslog.h>
#include "mavlink.h"

/**
 * Class MAVLinkLogger provides static methos for logging MAVLink messages to syslog.
 */
class MAVLinkLogger {
public:
    /**
     * Logs MAVLink message to syslog at the specified priority with the specified text prefix.
     *
     * Depending on inportance and frequency of the message, 'priority' parameter should be set to
     * LOG_ERR, LOG_NOTICE, LOG_INFO, or LOG_DEBUG.
     *
     * Example prefixes:
     * "MAV >>" - message received form the autopilot.
     * "MAV <<" - message sent to the autopilot.
     * "SBD >>" - message received from SBD transceiver
     * "SBD <<" - message sent to the SBD transceiver
     */
    static void log(int priority, const char* prefix, const mavlink_message_t& message);
};

#endif /* MAVLINKLOGGER_H_ */
