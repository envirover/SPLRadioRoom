/*
 MAVLinkTCPClient.h

 BVLOS telemetry for MAVLink autopilots.

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

 Created on: Mar 5, 2018
     Author: Pavel Bobov
*/

#ifndef MAVLINKTCPCHANNEL_H_
#define MAVLINKTCPCHANNEL_H_

#include <string>
#include "MAVLinkLib.h"
#include "MAVLinkChannel.h"

/**
 * Sends/receives MAVLink messages to/from a TCP/IP socket.
 */
class MAVLinkTCPChannel : public MAVLinkChannel
{
    int socket_fd;
    unsigned long timeout; // number of milliseconds to wait for the next char before aborting timed read
    clock_t start_millis;  // used for timeout measurement
    std::string address;
    uint16_t port;

public:

    /**
     * Constructs an instance of MAVLinkTcpClient.
     */
    MAVLinkTCPChannel();

    /**
     * Closes connection and frees the resources.
     */
    virtual ~MAVLinkTCPChannel();

    /**
     * Connects to the TCP/IP socket at the specified address and port.
     *
     * Returns true if the connection was successful.
     */
    bool init(const std::string address, uint16_t port);

    /**
     * Closes the connection if it was open.
     */
    void close();

    /**
     * Sends the specified MAVLink message to the socket.
     *
     * Returns true if the message was sent successfully.
     */
    bool send_message(const mavlink_message_t& msg);

    /**
     * Receives MAVLink message from the socket.
     *
     * Returns true if a message was received.
     */
    bool receive_message(mavlink_message_t& msg);

    /**
     * Checks if data is available in the socket input buffer.
     *
     * Returns true if data is available.
     */
    bool message_available();
};

#endif /* MAVLINKTCPCHANNEL_H_ */
