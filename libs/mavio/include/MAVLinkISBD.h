/*
 MAVLinkSBD.h

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

  Created on: Oct 26, 2017
      Author: pbobo
 */

#ifndef MAVLINKISBD_H_
#define MAVLINKISBD_H_

#include "IridiumSBD.h"
#include "MAVLinkLib.h"

namespace mavio {
/**
 * MAVLinkSBD is used to send/receive MAVLink messages to/from an ISBD transceiver.
 */
class MAVLinkISBD {
public:
    MAVLinkISBD();
    ~MAVLinkISBD();

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
     * Checks if data is available in ISBD.
     *
     * Returns true if data is available.
     */
    bool message_available();

    /**
     * Retrieves ring alert flag.
     *
     * Returns true is the operation was successful.
     */
    bool get_ring_alert_flag(uint16_t& raFlag);

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

    /**
     * Returns true if ISBD transceiver detected at the specified serial device.
     */
    bool detect_transceiver(std::string device);

private:
    Serial     stream;
    IridiumSBD isbd;
};

} // namespace mavio

#endif /* MAVLINKISBD_H_ */
