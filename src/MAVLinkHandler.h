/*
 MAVLinkHandler.h

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

#ifndef MAVLINKHANDLER_H_
#define MAVLINKHANDLER_H_

#include "MAVLinkAutopilot.h"
#include "MAVLinkISBDChannel.h"
#include "MAVLinkTCPChannel.h"
#include "Stopwatch.h"

constexpr uint16_t max_mission_count = 1024;

/**
 * Telemetry for MAVLink autopilots.
 */
class MAVLinkHandler {
public:
    /**
     * Default constructor.
     */
    MAVLinkHandler();

    /**
     * Initializes enabled comm channels and autopilot connections.
     *
     * Returns true if autopilot and enabled comm link connections were configured successfully.
     */
    bool init();

    /*
     * Closes all opened connections.
     */
    void close();

    /**
     * Single turn of the main message pump that timers, routes and processes 
     * messages received from autopilot and comm channels.
     * 
     * The pump must run in a tight loop started after init(). 
     */
    void loop();

private:
    /*
     * Returns channel that successfully sent or received message last.
     */
    mavio::MAVLinkChannel& active_channel();

    /*
     * Hanlde mobile-originated message received from autopilot.
     */
    void handle_mo_message(const mavlink_message_t& msg, mavio::MAVLinkChannel& channel);

    /*
     * Handle mobile-terminated message received from a comm channel. 
     */
    void handle_mt_message(const mavlink_message_t& msg, mavio::MAVLinkChannel& channel);

    /**
     * Sends report message to one of the comm channels if the channel report 
     * period has elapsed.
     * 
     * returns true if report was sent.
     */
    bool send_report();

    /*
     * Sends heartbeat message to autopilot if hearbeat period has elapsed and
     * the comm channels are not at faulted state (one of the channels successfuly
     * sent a message during it's report period). 
     * 
     * This allows autopilots to handle lost link gracefully if heartbeats are not received.
     */
    bool send_heartbeat();

    /*
     * Requests autopilot data streams required to compose report message.
     */
    void request_data_streams();

    /*
     * Integrates the specified message into the report message of HIGH_LATENCY type.
     *
     * Returns true if the message was integrated.
     */
    bool update_report_msg(const mavlink_message_t& msg);

    mavio::MAVLinkAutopilot       autopilot;
    mavio::MAVLinkISBDChannel     isbd_channel;
    mavio::MAVLinkTCPChannel      tcp_channel;
    Stopwatch              heartbeat_timer;
    Stopwatch              primary_report_timer;
    Stopwatch              secondary_report_timer;
    mavlink_high_latency_t report;
    uint16_t               report_mask;
    mavlink_message_t      mission_count_msg;
    mavlink_message_t      missions[max_mission_count];
    uint16_t               missions_received;
};

#endif /* MAVLINKHANDLER_H_ */
