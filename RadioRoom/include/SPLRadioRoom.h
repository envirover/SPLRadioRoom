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

#include <vector>
#include "MAVLinkSerial.h"
#include "IridiumSBD.h"
#include "SPLConfig.h"

#define MAX_MISSION_COUNT  7

#define MAX_SEND_RETRIES   5

#define HL_REPORT_PERIOD_PARAM "HL_REPORT_PERIOD"

/**
 * SPL RadioRoom companion computer for MAVLink autopilot.
 */
class SPLRadioRoom {

    MAVLinkSerial           autopilot;
    IridiumSBD              isbd;
    mavlink_high_latency_t  high_latency;
    uint8_t                 seq; // MO message sequence number
    unsigned long           last_report_time;

public:

    /**
     * Calls default constructors for MAVLinkSerial and IridiumSBD.
     */
    SPLRadioRoom();

    /**
     * Destructor
     */
    virtual ~SPLRadioRoom();

    /**
     * Initializes MAVLinkSerial and IridiumSBD instances.
     *
     * Returns true if MAVLinkSerial and IridiumSBD were successfully initialized.
     */
    bool init();

    /**
     *
     */
    void loop();

private:

    /**
     * Initialize connection to the autopilot serial.
     */
    bool init_autopilot(vector<string>& devices);

    /**
     * Initialize connection to the ISBD transceiver.
     */
    bool init_isbd(vector<string>& devices);

    /**
     * Debug print of mavlink_message_t message
     */
    //void print_mavlink_msg(const mavlink_message_t& msg) const;

    /**
     * Sends MT message to ISBD and receives MO message from the inbound message queue if any.
     *
     * Returns true if the ISBD session succeeded.
     */
    bool isbd_send_receive_message(const mavlink_message_t& mo_msg, mavlink_message_t& mt_msg, bool& received);

    /**
     * Updates HIGH_LATENCY message reporting period if HL_REPORT_PERIOD parameter value is set by
     * PARAM_SET MT message.
     *
     * Returns true if the message was handled.
     */
    bool handle_param_set(const mavlink_message_t& msg, mavlink_message_t& ack);

    /**
     * Handles writing waypoints list as described  in
     * http://qgroundcontrol.org/mavlink/waypoint_protocol
     *
     * returns true if waypoints list was updated in ardupilot
     */
    bool handle_mission_write(const mavlink_message_t& msg, mavlink_message_t& ack);

    /**
     * Sends the message to ISBD, recieve all the messages in the
     * inbound message queue, if any, pass them to ArduPilot,
     * sends ACKs back to ISBD.
     */
    void isbd_session(mavlink_message_t& mo_msg);

    /**
     * Reads and processes MAVLink messages from ArduPilot.
     */
    void comm_receive();

    /*
     * Integrates high frequency message into HIGH_LATENCY type message.
     *
     * @param msg message received from autopilot
     * @return true if the message was integrated or should be just swallowed
     */
    bool update_high_latency_msg(const mavlink_message_t& msg);
};

#endif /* SPLRADIOROOM_H_ */
