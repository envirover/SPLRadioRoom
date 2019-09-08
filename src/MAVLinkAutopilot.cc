/*
 MAVLinkAutopilot.cc
 
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

#include "MAVLinkAutopilot.h"

#include <chrono>
#include <syslog.h>
#include <time.h>

constexpr int     SEND_RETRIES           = 5;
constexpr int     RECEIVE_RETRIES        = 10;
constexpr int64_t MAX_HEARTBEAT_INTERVAL = 2000; //ms

constexpr struct timespec AUTOPILOT_SEND_INTERVAL[] = { { 0, 10000000L } }; // 10 milliseconds
constexpr struct timespec RECEIVE_RETRY_DELAY[]     = { { 0, 10000000L } }; // 10 ms

constexpr size_t max_autopilot_queue_size = 10;

using namespace std;
using namespace std::chrono;

MAVLinkAutopilot::MAVLinkAutopilot() : MAVLinkChannel("autopilot"),
                                       running(false),
                                       send_thread(),
                                       receive_thread(),
                                       serial(),
                                       send_queue(max_autopilot_queue_size),
                                       receive_queue(max_autopilot_queue_size),
                                       system_id(0)
{
}

bool MAVLinkAutopilot::init(const string& path, int speed, const vector<string>& devices)
{
    bool ret = connect(path, speed, devices);

    if (!running) {
        running = true;

        std::thread send_th(&MAVLinkAutopilot::send_task, this);
        send_thread.swap(send_th);

        std::thread receive_th(&MAVLinkAutopilot::receive_task, this);
        receive_thread.swap(receive_th);
    }

    return ret;
}

void MAVLinkAutopilot::close()
{
    if (running) {
        running = false;

        receive_thread.join();
        send_thread.join();
    }

    serial.close();
}

bool MAVLinkAutopilot::request_autopilot_version(uint8_t& autopilot, uint8_t& mav_type, uint8_t& sys_id, mavlink_autopilot_version_t& autopilot_version)
{
    mavlink_message_t msg, msg_command_long;
    autopilot = mav_type = sys_id = 0;
    memset(&autopilot_version, 0, sizeof(autopilot_version));

    for (high_resolution_clock::time_point start = high_resolution_clock::now();
         duration_cast<milliseconds>(high_resolution_clock::now() - start).count() < MAX_HEARTBEAT_INTERVAL;) {
        if (serial.receive_message(msg)) {
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                autopilot = mavlink_msg_heartbeat_get_autopilot(&msg);
                mav_type  = mavlink_msg_heartbeat_get_type(&msg);
                sys_id    = msg.sysid;

                if (autopilot != MAV_AUTOPILOT_INVALID) //Filter out heartbeat messages forwarded from GCS
                    break;
            }
        }

        nanosleep(RECEIVE_RETRY_DELAY, NULL);
    }

    //Return false if heartbeat message was not received
    if (sys_id == 0) {
        syslog(LOG_DEBUG, "Heartbeat not received.\n");
        return false;
    }

    for (int i = 0; i < SEND_RETRIES; i++) {
        mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg_command_long,
            sys_id, ARDUPILOT_COMPONENT_ID,
            MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
            i, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        if (serial.send_message(msg_command_long)) {
            for (int j = 0; j < RECEIVE_RETRIES; j++) {
                if (serial.receive_message(msg)) {
                    //printf("**** msg.msgid = %d\n", msg.msgid);
                    if (msg.msgid == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
                        mavlink_msg_autopilot_version_decode(&msg, &autopilot_version);
                        sys_id = msg.sysid;
                        return true;
                    }
                }
            }
        } else {
            syslog(LOG_DEBUG, "Failed to send message to autopilot.\n");
        }

        nanosleep(RECEIVE_RETRY_DELAY, NULL);
    }

    return true;
}

char* MAVLinkAutopilot::get_firmware_version(const mavlink_autopilot_version_t& autopilot_version, char* buff, size_t buff_size)
{
    strncpy(buff, "unknown", buff_size);

    if (autopilot_version.flight_sw_version != 0) {
        int                   majorVersion, minorVersion, patchVersion;
        FIRMWARE_VERSION_TYPE versionType;

        majorVersion = (autopilot_version.flight_sw_version >> (8 * 3)) & 0xFF;
        minorVersion = (autopilot_version.flight_sw_version >> (8 * 2)) & 0xFF;
        patchVersion = (autopilot_version.flight_sw_version >> (8 * 1)) & 0xFF;
        versionType  = (FIRMWARE_VERSION_TYPE)((autopilot_version.flight_sw_version >> (8 * 0)) & 0xFF);

        snprintf(buff, buff_size, "%d.%d.%d/%d ", majorVersion, minorVersion, patchVersion, versionType);
    }

    return buff;
}

bool MAVLinkAutopilot::send_message(const mavlink_message_t& msg)
{
    send_queue.push(msg);
    return true;
}

bool MAVLinkAutopilot::receive_message(mavlink_message_t& msg)
{
    return receive_queue.pop(msg);
}

bool MAVLinkAutopilot::message_available()
{
    return !receive_queue.empty();
}

std::chrono::high_resolution_clock::time_point MAVLinkAutopilot::last_send_time()
{
    return send_queue.last_push_time();
}

std::chrono::high_resolution_clock::time_point MAVLinkAutopilot::last_receive_time()
{
    return receive_queue.last_push_time();
}

bool MAVLinkAutopilot::send_receive_message(const mavlink_message_t& msg, mavlink_message_t& ack)
{
    for (int i = 0; i < SEND_RETRIES; i++) {
        if (send_message(msg)) {
            if (msg.msgid != MAVLINK_MSG_ID_COMMAND_LONG && msg.msgid != MAVLINK_MSG_ID_COMMAND_INT && msg.msgid != MAVLINK_MSG_ID_MISSION_ITEM && msg.msgid != MAVLINK_MSG_ID_PARAM_SET) {
                return false;
            }

            if (receive_ack(msg, ack)) {
                return true;
            }
        }
    }

    return compose_failed_ack(msg, ack);
}

bool MAVLinkAutopilot::connect(const string& path, int speed, const vector<string>& devices)
{
    syslog(LOG_NOTICE, "Connecting to autopilot (%s %d)...", path.data(), speed);

    system_id = 0;

    if (serial.init(path, speed) == 0) {
        system_id = detect_autopilot(path);

        if (system_id) {
            return true;
        }

        serial.close();
    } else {
        syslog(LOG_WARNING, "Failed to open serial device '%s'.", path.data());
    }

    if (devices.size() > 0) {
        syslog(LOG_NOTICE, "Attempting to detect autopilot at the available serial devices...");
        for (size_t i = 0; i < devices.size(); i++) {
            if (devices[i] == path)
                continue;

            if (serial.init(devices[i], speed) == 0) {
                system_id = detect_autopilot(devices[i].data());
                if (system_id) {
                    return true;
                }

                serial.close();
            } else {
                syslog(LOG_DEBUG, "Failed to open serial device '%s'.", devices[i].data());
            }
        }
    }

    serial.init(path, speed);

    syslog(LOG_ERR, "Autopilot was not detected on any of the serial devices.");

    return false;
}

uint8_t MAVLinkAutopilot::detect_autopilot(const string device)
{
    mavlink_autopilot_version_t autopilot_version;
    uint8_t                     autopilot, mav_type, sys_id;

    if (!request_autopilot_version(autopilot, mav_type, sys_id, autopilot_version)) {
        syslog(LOG_DEBUG, "Autopilot not detected at serial device '%s'.", device.data());
        return 0;
    }

    char buff[64];
    get_firmware_version(autopilot_version, buff, sizeof(buff));

    syslog(LOG_NOTICE, "Autopilot detected at serial device '%s'.", device.data());
    syslog(LOG_NOTICE, "MAV type: %d, system id: %d, autopilot class: %d, firmware version: %s", mav_type, sys_id, autopilot, buff);

    return sys_id;
}

bool MAVLinkAutopilot::receive_ack(const mavlink_message_t& msg, mavlink_message_t& ack)
{
    for (int i = 0; i < RECEIVE_RETRIES; i++) {
        switch (msg.msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG:
        case MAVLINK_MSG_ID_COMMAND_INT:
            if (receive_message(ack) && ack.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                return true;
            }
            break;
        case MAVLINK_MSG_ID_MISSION_ITEM:
            if (receive_message(ack) && (ack.msgid == MAVLINK_MSG_ID_MISSION_ACK || ack.msgid == MAVLINK_MSG_ID_MISSION_REQUEST)) {
                return true;
            }
            break;
        case MAVLINK_MSG_ID_PARAM_SET:
            if (receive_message(ack) && ack.msgid == MAVLINK_MSG_ID_PARAM_VALUE) {
                return true;
            }
            break;
        default:
            return false;
        }

        nanosleep(RECEIVE_RETRY_DELAY, NULL);
    }

    return false;
}

bool MAVLinkAutopilot::compose_failed_ack(const mavlink_message_t& msg, mavlink_message_t& ack)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG:
        mavlink_command_ack_t command_ack;
        command_ack.command = mavlink_msg_command_long_get_command(&msg);
        command_ack.result  = MAV_RESULT_FAILED;
        mavlink_msg_command_ack_encode(system_id, ARDUPILOT_COMPONENT_ID, &ack, &command_ack);
        return true;
    case MAVLINK_MSG_ID_COMMAND_INT:
        command_ack.command = mavlink_msg_command_int_get_command(&msg);
        command_ack.result  = MAV_RESULT_FAILED;
        mavlink_msg_command_ack_encode(system_id, ARDUPILOT_COMPONENT_ID, &ack, &command_ack);
        return true;
    case MAVLINK_MSG_ID_MISSION_ITEM:
        mavlink_mission_ack_t mission_ack;
        mission_ack.target_system    = msg.sysid;
        mission_ack.target_component = msg.compid;
        mission_ack.type             = MAV_MISSION_ERROR;
        mavlink_msg_mission_ack_encode(system_id, ARDUPILOT_COMPONENT_ID, &ack, &mission_ack);
        return true;
    case MAVLINK_MSG_ID_PARAM_SET:
        mavlink_param_value_t param_value;
        param_value.param_count = 0;
        param_value.param_index = 0;
        param_value.param_type  = mavlink_msg_param_value_get_param_type(&msg);
        mavlink_msg_param_value_get_param_id(&msg, param_value.param_id);
        param_value.param_value = mavlink_msg_param_set_get_param_value(&msg);
        mavlink_msg_param_value_encode(system_id, ARDUPILOT_COMPONENT_ID, &ack, &param_value);
        return true;
    default:
        ack.len = ack.msgid = 0;
        return false;
    }
}

void MAVLinkAutopilot::send_task()
{
    while (running) {
        mavlink_message_t msg;

        if (send_queue.pop(msg)) {
            serial.send_message(msg);
        }

        nanosleep(AUTOPILOT_SEND_INTERVAL, NULL);
    }
}

void MAVLinkAutopilot::receive_task()
{
    while (running) {
        mavlink_message_t msg;

        if (serial.receive_message(msg)) {
            receive_queue.push(msg);
        }
    }
}
