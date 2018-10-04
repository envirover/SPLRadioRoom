/*
 MAVLinkSBD.cc

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


#include "MAVLinkISBDChannel.h"

#include "MAVLinkLogger.h"
#include <stdio.h>
#include <syslog.h>

MAVLinkISBDChannel::MAVLinkISBDChannel() : MAVLinkChannel("ISBD"), stream(), isbd(stream), received_messages()
{
}

MAVLinkISBDChannel::~MAVLinkISBDChannel()
{
}

bool MAVLinkISBDChannel::get_ring_alert_flag(uint16_t &ra_flag)
{
    uint16_t mo_flag = 0, mo_msn = 0, mt_flag = 0, mt_msn = 0, msg_waiting = 0;

    ra_flag = 0;

    int err = isbd.getStatusExtended(mo_flag, mo_msn, mt_flag, mt_msn, ra_flag, msg_waiting);

    if (err != ISBD_SUCCESS) {
        syslog(LOG_WARNING, "Failed to get ISBD status. Error  = %d", err);
    } else if (ra_flag) {
        syslog(LOG_INFO, "Ring alert received.");
    }

    return err == ISBD_SUCCESS;
}

int MAVLinkISBDChannel::get_waiting_wessage_count()
{
    return isbd.getWaitingMessageCount();
}

bool MAVLinkISBDChannel::detect_transceiver(string device) {
    int ret = isbd.begin();

    if (ret == ISBD_SUCCESS || ret == ISBD_ALREADY_AWAKE) {
        char model[256], imea[256];
        imea[0] = model[0] = 0;
        ret = isbd.getTransceiverModel(model, sizeof(model));
        if (ret == ISBD_SUCCESS) {
            ret = isbd.getTransceiverSerialNumber(imea, sizeof(imea));
            if (ret == ISBD_SUCCESS) {
                syslog(LOG_NOTICE, "%s (IMEA %s) detected at serial device '%s'.", model, imea, device.data());
                return true;
            }
        }
    }

    syslog(LOG_DEBUG, "ISBD transceiver not detected at serial device '%s'. Error code = %d.", device.data(), ret);
    return false;
}

bool MAVLinkISBDChannel::init(string path, int speed, const vector<string>& devices)
{
    syslog(LOG_INFO, "Connecting to ISBD transceiver (%s %d)...", path.data(), speed);

    isbd.setPowerProfile(1);

    if (stream.open(path, speed) == 0) {
        if (detect_transceiver(path)) {
            return true;
        }

        stream.close();
    } else {
        syslog(LOG_INFO, "Failed to open serial device '%s'.", path.data());
    }

    if (devices.size() > 0) {
        syslog(LOG_INFO, "Attempting to detect ISBD transceiver at the available serial devices...");

        for (size_t i = 0; i < devices.size(); i++) {
            if (devices[i] == path)
                continue;

            if (stream.open(devices[i].data(), speed) == 0) {
                if (detect_transceiver(devices[i])) {
                    return true;
                } else {
                    stream.close();
                }
            } else {
                syslog(LOG_DEBUG, "Failed to open serial device '%s'.", devices[i].data());
            }
        }
    }

    stream.open(path, speed);
    syslog(LOG_ERR, "ISBD transceiver was not detected on any of the serial devices.");

    return false;
}

void MAVLinkISBDChannel::close()
{
    stream.close();
    syslog(LOG_DEBUG, "ISBD connection closed.");
}

/**
 * Sends the specified MAVLink message to ISBD.
 *
 * Returns true if the message was sent successfully.
 */
bool MAVLinkISBDChannel::send_message(const mavlink_message_t& msg)
{
    if (msg.len == 0 && msg.msgid == 0) {
       return true;
    }

    mavlink_message_t mt_msg;
    bool received;
    bool ret = send_receive_message(msg, mt_msg, received);

    if (received) {
        received_messages.push(mt_msg);
    }

    return ret;
}

/**
 * Receives MAVLink message from ISBD.
 *
 * Returns true if a message was received.
 */
bool MAVLinkISBDChannel::receive_message(mavlink_message_t& msg)
{
    if (!received_messages.empty()) {
        msg = received_messages.front();
        received_messages.pop();
        return true;
    }

    mavlink_message_t mo_msg;
    mo_msg.len   = 0;
    mo_msg.msgid = 0;

    bool received = false;
    send_receive_message(mo_msg, msg, received);

    return received;
}

/**
 * Checks if data is available in ISBD.
 *
 * Returns true if data is available.
 */
bool MAVLinkISBDChannel::message_available()
{
    if (!received_messages.empty() || isbd.getWaitingMessageCount() > 0) {
        return true;
    }

    uint16_t ra_flag = 0;

    get_ring_alert_flag(ra_flag);

    return ra_flag != 0;
}

bool MAVLinkISBDChannel::send_receive_message(const mavlink_message_t& mo_msg, mavlink_message_t& mt_msg, bool& received)
{
    uint8_t buf[ISBD_MAX_MT_MGS_SIZE];
    size_t buf_size = sizeof(buf);
    uint16_t len = 0;

    if (mo_msg.len != 0 && mo_msg.msgid != 0) {
        len = mavlink_msg_to_send_buffer(buf, &mo_msg);
    }

    received = false;

    int ret = isbd.sendReceiveSBDBinary(buf, len, buf, buf_size);

    if (ret != ISBD_SUCCESS) {
        if (mo_msg.len != 0 && mo_msg.msgid != 0) {
            char prefix[32];
            snprintf(prefix, 32, "SBD << FAILED(%d)", ret);
            MAVLinkLogger::log(LOG_WARNING, prefix, mo_msg);
        } else {
            syslog(LOG_WARNING, "SBD >> FAILED(%d)", ret); //Failed to receive MT message from ISBD
        }

        return false;
    }

    if (buf_size > 0) {
        mavlink_status_t mavlink_status;

        for (size_t i = 0; i < buf_size; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &mt_msg, &mavlink_status)) {
                received = true;

                MAVLinkLogger::log(LOG_INFO, "SBD >>", mt_msg);
                break;
            }
        }

        if (!received) {
            syslog(LOG_WARNING, "Failed to parse MAVLink message received from ISBD.");
        }
    }

    MAVLinkLogger::log(LOG_INFO, "SBD <<", mo_msg);

    return true;
}
