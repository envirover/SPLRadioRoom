/*
 MAVLinkTCP.h

 MAVIO MAVLink I/O library.

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

 Created on: Mar 5, 2018
     Author: Pavel Bobov
*/

#ifndef MAVLINKTCP_H_
#define MAVLINKTCP_H_

#include "MAVLinkLib.h"
#include <string>

namespace mavio {

/**
 * Sends/receives MAVLink messages to/from a TCP/IP socket.
 */
class MAVLinkTCP {
public:
    /**
     * Constructs an instance of MAVLinkTcpClient.
     */
    MAVLinkTCP();

    /**
     * Closes connection and frees the resources.
     */
    virtual ~MAVLinkTCP();

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

private:
    std::string   address; // UV Hub IP address
    uint16_t      port; // UV Hub port
    int           socket_fd; // Socket file descriptor
    unsigned long timeout; // number of milliseconds to wait for the next char before aborting timed read
    clock_t       start_millis; // used for timeout measurement
};

} // namespace mavio

#endif /* MAVLINKTCP_H_ */
