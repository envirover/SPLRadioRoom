/*
 SPLRadioRoom.h

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

 Created on: Oct 17, 2017
     Author: Pavel Bobov
*/

#ifndef SPLRADIOROOM_H_
#define SPLRADIOROOM_H_

#include "MAVLinkSerial.h"
#include "HighLatencyMsg.h"
#include "IridiumSBD.h"
#include "SPLConfig.h"

#define MAX_MISSION_COUNT  7

#define MAX_SEND_RETRIES   5

#define AP_TELEM_BAUD_RATE B57600
#define ISBD_BAUD_RATE     B19200

#define HL_REPORT_PERIOD_PARAM "HL_REPORT_PERIOD"

class SPLRadioRoom {
    Serial telem;
    MAVLinkSerial  ardupilot;

    Serial nss;
    IridiumSBD isbd;

    SPLConfig config;
    HighLatencyMsg high_latency_msg;

    mavlink_message_t missions[MAX_MISSION_COUNT];

    unsigned long last_report_time;

public:
    SPLRadioRoom();
    virtual ~SPLRadioRoom();

    void setup();
    void loop();

private:
    void print_mavlink_msg(const mavlink_message_t& msg);
    bool isbd_send_receive_message(const mavlink_message_t& mo_msg, mavlink_message_t& mt_msg, bool& received);
    bool handle_param_set(const mavlink_message_t& msg, mavlink_message_t& ack);
    bool handle_mission_write(const mavlink_message_t& msg, mavlink_message_t& ack);
    void isbd_session(mavlink_message_t& mo_msg);
    void comm_receive();
};

#endif /* SPLRADIOROOM_H_ */
