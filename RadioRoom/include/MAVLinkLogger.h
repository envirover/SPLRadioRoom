/*
 * MAVLinkLogger.h
 *
 *  Created on: Oct 22, 2017
 *      Author: Pavel Bobov
 */

#ifndef MAVLINKLOGGER_H_
#define MAVLINKLOGGER_H_

#include <syslog.h>
#include "mavlink.h"

/**
 * Logs MAVLink messages to syslog.
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
     * "SBD >>" - message received from ISBD transceiver
     * "SBD <<" - message sent to the ISBD transceiver
     */
    static void log(int priority, const char* prefix, const mavlink_message_t& message);
};

#endif /* MAVLINKLOGGER_H_ */
