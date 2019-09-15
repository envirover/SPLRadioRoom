/*
 MAVLinkSerial.h

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
 */

#ifndef MAVLINKSERIAL_H_
#define MAVLINKSERIAL_H_

#include "MAVLinkLib.h"
#include "Serial.h"
#include <string>

namespace mavio {

/**
 * MAVLinkSerial is used to send and receive MAVLink messages to/from a serial interface.
 */
class MAVLinkSerial {
public:
    /**
     * Constructs MAVLinkSerial instance using the specified serial interface.
     */
    MAVLinkSerial();

    /**
     * Initialize connection to the serial device with the specified baud rate.
     * 
     * Returns true if serial device connection succeeded.
     */
    bool init(const std::string& path, int speed);

    /**
     * Closes connection to the serial device.
     */
    void close();

    /**
     * Returns the path of serial device set by init(...) call.
     */
    inline std::string get_path() const { return serial.get_path(); };

    /**
     * Sends MAVLink message to the serial interface.
     *
     * Returns true on success.
     */
    bool send_message(const mavlink_message_t& msg);

    /**
     * Receives MAVLink message from the serial interface.
     *
     * Returns true if MAVLink message was received.
     */
    bool receive_message(mavlink_message_t& msg);

private:
    Serial serial;
};

} // namespace mavio

#endif // MAVLINKSERIAL_H_
