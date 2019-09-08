/*
 MAVLinkISBDChannel.h

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

  Created on: Aug 23, 2019
      Author: pbobo
 */

#ifndef MAVLINKISBDCHANNEL_H_
#define MAVLINKISBDCHANNEL_H_

#include "CircularBuffer.h"
#include "MAVLinkChannel.h"
#include "MAVLinkISBD.h"
#include "MAVLinkLib.h"

#include <atomic>
#include <thread>
#include <string>

/**
 * MAVLinkISBDChannel asynchronously sends and receives MAVLink messages to/from an ISBD transceiver.
 */
class MAVLinkISBDChannel : public MAVLinkChannel {
public:
    MAVLinkISBDChannel();
    ~MAVLinkISBDChannel();

    /**
     * Initializes connection to ISBD transceiver on the specified serial device.
     * If auto_detect_serial is true the method automatically detects the serial device
     * if the transceiver is available on any of the serial devices in the system.
     *
     * Returns true if connection was successful.
     */
    bool init(std::string path, int speed, const std::vector<std::string>& devices);

    /*
     * Closes the serial device used to connect to ISBD.
     */
    void close();

    /**
     * Sends the specified MAVLink message to ISBD.
     *
     * Returns true if the message was sent successfully.
     */
    bool send_message(const mavlink_message_t& msg);

    /**
     * Receives MAVLink message from ISBD transceiver.
     *
     * Returns true if a message was received.
     */
    bool receive_message(mavlink_message_t& msg);

    /**
     * Checks if data is available in ISBD transceiver.
     *
     * Returns true if data is available.
     */
    bool message_available();

     /**
     * Returns time of the last successfully sent message.  
     */
    std::chrono::high_resolution_clock::time_point last_send_time() override;

    /**
     * Returns time of the last successfully received message.  
     */
    std::chrono::high_resolution_clock::time_point last_receive_time() override;   

private:
    /**
     * While running is true, executes send-receive ISBD sessions. 
     */
    void send_receive_task();

    MAVLinkISBD                       isbd;
    std::atomic<bool>                 running;
    std::thread                       send_receive_thread; // Thread of send_receive_task
    CircularBuffer<mavlink_message_t> send_queue; // Queue that buffers messages to be sent to the socket
    CircularBuffer<mavlink_message_t> receive_queue; // Queue that buffers messages received from the socket
};

#endif /* MAVLINKISBDCHANNEL_H_ */