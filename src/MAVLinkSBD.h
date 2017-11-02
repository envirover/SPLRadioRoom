/*
 MAVLinkSBD.h

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

  Created on: Oct 26, 2017
      Author: pbobo
 */

#ifndef MAVLINKSBD_H_
#define MAVLINKSBD_H_

#include "IridiumSBD.h"
#include "mavlink.h"

/**
 * MAVLinkSerial is used to send/receive MAVLink messages to/from an ISBD transceiver.
 */
class MAVLinkSBD {
    Serial stream;
    IridiumSBD isbd;

public:
    MAVLinkSBD();
    ~MAVLinkSBD();

    /**
     * Initializes connection to ISBD transceiver on the specified serial device.
     * If auto_detect_serial is true the method automatically detects the serial device
     * if the transceiver is available on any of the serial devices in the system.
     *
     * Returns true if connection was successful.
     */
    bool init(std::string path, int speed, const vector<string>& devices);

    /*
     * Closes the serial device used to connect to ISBD.
     */
    void close();

    /**
     * Retrieves ring alert flag.
     *
     * Returns true is the operation was successful.
     */
    bool get_ring_alert_flag(uint16_t &raFlag);

    /**
     * Returns the number of mobile terminated messages left in the queue.
     */
    int get_waiting_wessage_count();

    /**
     * Sends MT message to ISBD and receives MO message from the in-bound message queue if any.
     *
     * Returns true if the ISBD session succeeded.
     */
    bool send_receive_message(const mavlink_message_t& mo_msg, mavlink_message_t& mt_msg, bool& received);

private:

    /**
     * Returns true if ISBD transceiver detected at the specified serial device.
     */
    bool detect_transceiver(std::string device);
};

#endif /* MAVLINKSBD_H_ */
