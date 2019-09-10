/*
 MAVLinkAutopilot.h
 
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

 Created on: Aug 18, 2019
     Author: Pavel Bobov
*/

#ifndef MAVLINK_AUTOPILOT_
#define MAVLINK_AUTOPILOT_

#include "CircularBuffer.h"
#include "MAVLinkChannel.h"
#include "MAVLinkLib.h"
#include "MAVLinkSerial.h"

#include <atomic>
#include <thread>
#include <vector>

namespace mavio {

constexpr uint8_t SYSTEM_ID              = 255; // GCS system Id
constexpr uint8_t COMPONENT_ID           = 1; // GCS component Id
constexpr uint8_t ARDUPILOT_COMPONENT_ID = 0;

/**
 * Asynchronously sends and receivs MAVLink messages to autopilot over serial interface. 
 */
class MAVLinkAutopilot : public MAVLinkChannel {
public:
    /**
     * Constructs MAVLinkSerial instance.
     */
    MAVLinkAutopilot();

    /**
     * Initializes connection to the serial device and starts send and receive tasks.
     * 
     * @param path serial device path
     * @param speed serial device speed
     * @param devices list of all serial devices 
     * @return true if initialization succeeded
     */
    bool init(const std::string& path, int speed, const std::vector<std::string>& devices);

    /**
     * Stops send and receive tasks and closes the serial device.
     */
    void close() override;

    /**
     * Returns the path of serial device set by init(...) call.
     */
    inline std::string get_path() const { return serial.get_path(); };

    /**
     * Returns the autopilot system id.
     */
    inline uint8_t get_system_id() const { return system_id; };

    /**
     * Send MAVLink message to autopilot.
     *
     * @param msg MAVLink message
     * @return always returns true
     */
    bool send_message(const mavlink_message_t& msg) override;

    /**
     * Receive MAVLink message from autopilot.
     * 
     * @param msg reference to MAVLink message object
     * @return true if MAVLink message was received
     */
    bool receive_message(mavlink_message_t& msg) override;

    /**
     * Returns true if there is a message in the receive queue.
     */
    bool message_available() override;

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
     * Retries sending message to autopilot until ACK is received.
     *
     * Returns true if ACK message was received.
     */
    bool send_receive_message(const mavlink_message_t& msg, mavlink_message_t& ack);

    /**
     * Connects to the serial device.
     */
    bool connect(const std::string& path, int speed, const std::vector<std::string>& devices);

    /*
     * Checks if MAVLink autopilot is available on the specified serial device.
     *
     * If an autopilot was detected, returns the autopilot's system id, otherwise returns 0.
     */
    uint8_t detect_autopilot(const std::string device);

    /**
     * Sends REQUEST_AUTOPILOT_CAPABILITIES message to the autopilot and
     * reads AUTOPILOT_VERSION message replied by the autopilot.
     *
     * Returns true if AUTOPILOT_VERSION message was received.
     */
    bool request_autopilot_version(uint8_t& autopilot, uint8_t& mav_type, uint8_t& sys_id, mavlink_autopilot_version_t& autopilot_version);

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
     * While running is true, retrieves messages from send_queue and sends them to serial. 
     */
    void send_task();

    /**
     *  While running is true, receives messages from serial and pushes them to receive_queue.
     */
    void receive_task();

    /**
     * Retrieves firmware version string from the specified AUTOPILOT_VERSION message.
     *
     * Returns the firmware version string.
     */
    static char* get_firmware_version(const mavlink_autopilot_version_t& autopilot_version, char* buff, size_t buff_size);

    std::atomic<bool>                 running;
    std::thread                       send_thread; // Thread of send_task
    std::thread                       receive_thread; // Thread of receive_task
    MAVLinkSerial                     serial; // Serial interface
    CircularBuffer<mavlink_message_t> send_queue; // Queue that buffers messages sent to autopilot
    CircularBuffer<mavlink_message_t> receive_queue; // Queue that buffers messages received from autopilot
    uint8_t                           system_id; // Autopilot system Id
};

} // namespace mavio

#endif /* MAVLINK_AUTOPILOT_ */