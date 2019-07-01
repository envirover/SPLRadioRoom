/*
 MAVLinkHandler.h

BVLOS telemetry for MAVLink autopilots.

 (C) Copyright 2018 Envirover.

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

#ifndef MAVLINKHANDLER_H_
#define MAVLINKHANDLER_H_

#include "MAVLinkSerial.h"
#include "MAVLinkChannel.h"
#include "Stopwatch.h"
#include <vector>
#include "Config.h"
#include "MAVLinkISBDChannel.h"
#include "MAVLinkTCPChannel.h"

#define HL_REPORT_PERIOD_PARAM "HL_REPORT_PERIOD"

/**
 * Telemetry for MAVLink autopilots.
 */
class MAVLinkHandler {

    MAVLinkSerial           autopilot;
    MAVLinkISBDChannel      isbd_channel;
    MAVLinkTCPChannel       tcp_channel;
    Stopwatch               report_time;

public:

    /**
     * Default constructor.
     */
    MAVLinkHandler();

    /**
     * Initializes enabled ISBD and TCP comm links and autopilot connections.
     *
     * Returns true if autopilot and enabled comm link connections were initialized successfully.
     */
    bool init();

    /*
     * Closes all opened connections.
     */
    void close();

    /**
     * Starts TCP session if report period is elapsed.
     * Starts ISBD session if ring alert received.
     */
    void loop();

private:

    /**
     * Starts TCP session if TCP channel is enabled and MAVlink message is available
     * or TCP report period is elapsed.
     */
    void tcp_loop();

    /**
     * Starts ISBD session if ISBD channel is enabled and MAVlink message is available
     * or ISBD report period is elapsed.
     */
    void isbd_loop();

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
     * the method retrieves all the mission items from the channel, sends them
     * to the autopilot, and sends MISSION_ACK to the channel. Otherwise the
     * method does nothing and just returns false.
     *
     * Returns true if waypoints list was updated in the autopilot.
     */
    bool handle_mission_write(MAVLinkChannel& channel, const mavlink_message_t& msg, mavlink_message_t& ack);

    /**
     * Sends MISSION_COUNT and MISSION_ITEM messages to the autopilot.
     * Receives MISSION_ACK message from the autopilot.
     *
     * Returns true if the missions were successfully sent to the autopilot.
     */
    bool send_missions_to_autopilot(const mavlink_message_t& mission_count, const vector<mavlink_message_t>& missions, mavlink_message_t& ack);

    /**
     * Sends the specified HIGH_LATENCY message to the Cchannel.
     *
     * Receives and handles all the messages in the MT queue.
     */
    bool comm_session(MAVLinkChannel& channel, mavlink_message_t& mo_msg);

    /*
     * Requests autopilot data streams required to compose HIGH_LATENCY message.
     */
    void request_data_streams();

    /**
     * Retrieves all the required data from the autopilot and composes HIGH_LATENCY message.
     */
    void get_high_latency_msg(mavlink_message_t& msg);

    /*
     * Integrates the specified message into the HIGH_LATENCY message.
     *
     * Returns true if the message was integrated.
     */
    bool update_high_latency_msg(const mavlink_message_t& msg, mavlink_high_latency_t& high_latency, uint16_t& mask);
};

#endif /* MAVLINKHANDLER_H_ */
