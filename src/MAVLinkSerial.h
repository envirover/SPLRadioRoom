/*
 MAVLinkSerial.h

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

#ifndef MAVLINKSERIAL_H_
#define MAVLINKSERIAL_H_

#include <ctime>
#include "Serial.h"
#include "mavlink.h"

#define SYSTEM_ID               255
#define COMPONENT_ID            1
#define ARDUPILOT_SYSTEM_ID     1
#define ARDUPILOT_COMPONENT_ID  0

#define SEND_RETRIES            5
#define RECEIVE_RETRIES         10
#define RECEIVE_RETRY_DELAY     10   //ms
#define RETRIES_TIMEOUT         1000

#define MAX_HEARTBEAT_INTERVAL  2000 //ms

/**
 * MAVLinkSerial is used to send and receive MAVLink messages to/from a serial interface.
 */
class MAVLinkSerial
{
    Serial         serial;
    unsigned long  timeout;       // number of milliseconds to wait for the next char before aborting timed read
    clock_t        start_millis;  // used for timeout measurement

public:

    /**
     * Constructs MAVLinkSerial instance using the specified serial interface.
     */
    MAVLinkSerial();

    /**
     * Initialize connection to the serial device.
     */
    bool init(const string& path, int speed, const vector<string>& devices);

    /**
     * Closes connection to the serial device.
     */
    void close();

    /**
     * Returns the path of serial device set by init(...) call.
     */
    inline string get_path() const { return serial.get_path(); };

    /**
     * Sends REQUEST_AUTOPILOT_CAPABILITIES message to the autopilot and
     * reads AUTOPILOT_VERSION message replied by the autopilot.
     *
     * Returns true if AUTOPILOT_VERSION message was received.
     */
    bool request_autopilot_version(uint8_t& autopilot, uint8_t& mav_type, uint8_t& sys_id, mavlink_autopilot_version_t& autopilot_version);

    /**
     * Retrieves firmware version string from the specified AUTOPILOT_VERSION message.
     *
     * Returns the  firmware version string.
     */
    char* get_firmware_version(const mavlink_autopilot_version_t& autopilot_version, char* buff, size_t buff_size) const;

    /**
     * Send MAVLink message to ArduPilot.
     *
     * Returns true on success.
     */
    bool send_message(const mavlink_message_t& msg);

    /**
     * Receive MAVLink message from ArduPilot.
     *
     * Returns true if MAVLink message was received.
     */
    bool receive_message(mavlink_message_t& msg);

    /**
     * Retries sending message to ArduPilot until ACK is received.
     *
     * Returns true if ACK message was received.
     */
    bool send_receive_message(const mavlink_message_t& msg, mavlink_message_t& ack);

private:

    /*
     * Checks if MAVLink autopilot is available on the specified serial device.
     *
     * Returns true is autopilot was detected.
     */
    bool detect_autopilot(const string device);

    /**
     * Receive messages from serial several times until received
     * COMMAND_ACK for COMMAND_LONG and COMMAND_INT or
     * MISSION_ACK for MISSION_ITEM message.
     */
    bool receive_ack(const mavlink_message_t& msg, mavlink_message_t& ack);

    /**
     * Compose an unconfirmed COMMAND_ACK or MISSION_ACK message.
     * Unconfirmed ACK messages are sent to GCS if ACK message was not
     * received from autopilot.
     */
    bool compose_failed_ack(const mavlink_message_t& msg, mavlink_message_t& ack);

    /**
     * Read stream with timeout.
     *
     * Returns character read  or -1 in case of timeout.
     */
    int timed_read();
};

#endif // MAVLINKSERIAL_H_
