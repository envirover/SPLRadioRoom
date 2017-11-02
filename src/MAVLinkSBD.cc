/*
 MAVLinkSBD.cpp

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


#include "MAVLinkSBD.h"
#include "MAVLinkLogger.h"
#include <syslog.h>

MAVLinkSBD::MAVLinkSBD() : stream(), isbd(stream)
{
}

MAVLinkSBD::~MAVLinkSBD()
{
}

bool MAVLinkSBD::get_ring_alert_flag(uint16_t &ra_flag)
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

int MAVLinkSBD::get_waiting_wessage_count()
{
    return isbd.getWaitingMessageCount();
}

bool MAVLinkSBD::detect_transceiver(string device) {
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

bool MAVLinkSBD::init(string path, int speed, const vector<string>& devices)
{
    syslog(LOG_NOTICE, "Connecting to ISBD transceiver (%s %d)...", path.data(), speed);

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
        syslog(LOG_NOTICE, "Attempting to detect ISBD transceiver at the available serial devices...");

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

void MAVLinkSBD::close()
{
    stream.close();
}

bool MAVLinkSBD::send_receive_message(const mavlink_message_t& mo_msg, mavlink_message_t& mt_msg, bool& received)
{
    uint8_t buf[ISBD_MAX_MT_MGS_SIZE];
    size_t buf_size = sizeof(buf);
    uint16_t len = 0;

    if (mo_msg.len != 0 && mo_msg.msgid != 0) {
        len = mavlink_msg_to_send_buffer(buf, &mo_msg);
    }

    received = false;

    if (isbd.sendReceiveSBDBinary(buf, len, buf, buf_size) != ISBD_SUCCESS) {
        MAVLinkLogger::log(LOG_WARNING, "SBD << FAILED", mo_msg);
        return false;
    }

    if (buf_size > 0) {
        mavlink_status_t mavlink_status;

        for (size_t i = 0; i < buf_size; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &mt_msg, &mavlink_status)) {
                received = true;

                MAVLinkLogger::log(LOG_DEBUG, "SBD >>", mt_msg);
                break;
            }
        }
    }

    MAVLinkLogger::log(LOG_DEBUG, "SBD <<", mo_msg);

    return true;
}
