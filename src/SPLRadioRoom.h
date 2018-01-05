/*
 SPLRadioRoom.h

 Iridium SBD telemetry for MAVLink autopilots.

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

 Created on: Oct 17, 2017
     Author: Pavel Bobov
*/

#ifndef SPLRADIOROOM_H_
#define SPLRADIOROOM_H_

#include "MAVLinkSerial.h"
#include "MAVLinkSBD.h"
#include "SPLConfig.h"
#include "Stopwatch.h"
#include <vector>

#define HL_REPORT_PERIOD_PARAM "HL_REPORT_PERIOD"

/**
 * Iridium SBD telemetry for MAVLink autopilots.
 */
class SPLRadioRoom {

    MAVLinkSerial           autopilot;
    MAVLinkSBD              isbd;
    mavlink_high_latency_t  high_latency;
    Stopwatch               report_time;

public:

    /**
     * Calls default constructors for MAVLinkSerial and IridiumSBD.
     */
    SPLRadioRoom();

    /**
     * Initializes MAVLinkSerial and IridiumSBD instances.
     *
     * Returns true if MAVLinkSerial and IridiumSBD were successfully initialized.
     */
    bool init();

    /*
     * Closes MAVLinkSerial and IridiumSBD.
     */
    void close();

    /**
     * Requests data streams from autopilot and updates HIGH_LATENCY MAVLink message.
     * Starts ISBD session if report period is elapsed or if ring alert received.
     */
    void loop();

private:

    /**
     * If the specified message is of type PARAM_SET and the parameter name is HL_REPORT_PERIOD,
     * the method sets report period configuration property. Otherwise the method does nothing.
     *
     * Returns true if the message was handled.
     */
    bool handle_param_set(const mavlink_message_t& msg, mavlink_message_t& ack);

    /**
     * Handles writing waypoints list as described  in
     * http://qgroundcontrol.org/mavlink/waypoint_protocol
     *
     * If message specified by msg parameter is of type MISSION_COUNT,
     * the method retrieves all the mission items from ISBD, sends them
     * to the autopilot, and sends MISSION_ACK to ISBD. Otherwise the
     * method does nothing and just returns false.
     *
     * Returns true if waypoints list was updated in the autopilot.
     */
    bool handle_mission_write(const mavlink_message_t& msg, mavlink_message_t& ack);

    /**
     * Sends the specified HIGH_LATENCY message to ISBD.
     * Receives and handles all the messages in the MT queue.
     */
    void isbd_session(mavlink_message_t& mo_msg);

    /**
     * Retrieves all the required data and composes HIGH_LATENCY message.
     */
    void get_high_latency_msg(mavlink_message_t& msg);

    /*
     * Integrates the specified message into the HIGH_LATENCY message.
     *
     * Returns true if the message was integrated.
     */
    bool update_high_latency_msg(const mavlink_message_t& msg);
};

#endif /* SPLRADIOROOM_H_ */
