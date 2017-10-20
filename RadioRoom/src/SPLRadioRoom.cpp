/*
 SPLRadioRoom.cpp

 Iridium SBD telemetry for ArduPilot.

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

 Created on: Oct 17, 2017
     Author: Pavel Bobov
 */
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <istream>
#include "SPLRadioRoom.h"

using namespace std;

SPLRadioRoom::SPLRadioRoom() :
    high_latency_msg(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID),
    ardupilot(telem),
    isbd(nss),
    last_report_time(0)

{
}

SPLRadioRoom::~SPLRadioRoom()
{
}


void SPLRadioRoom::print_mavlink_msg(const mavlink_message_t& msg) const
{
    printf("** \nmsgid = %d\ncompid = %d\n", msg.msgid, msg.compid);
}

bool SPLRadioRoom::isbd_send_receive_message(const mavlink_message_t& mo_msg, mavlink_message_t& mt_msg, bool& received)
{
    uint8_t buf[ISBD_MAX_MT_MGS_SIZE];
    size_t buf_size = sizeof(buf);
    uint16_t len = 0;

    if (mo_msg.len != 0 && mo_msg.msgid != 0) {
        printf("Sending MO message.\n");
        print_mavlink_msg(mo_msg);

        len = mavlink_msg_to_send_buffer(buf, &mo_msg);
    }

    received = false;

    if (isbd.sendReceiveSBDBinary(buf, len, buf, buf_size) != ISBD_SUCCESS) {
        return false;
    }

    if (buf_size > 0) {
        mavlink_status_t mavlink_status;

        for (size_t i = 0; i < buf_size; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &mt_msg, &mavlink_status)) {
                received = true;

                printf("MT message received.\n");
                print_mavlink_msg(mt_msg);

                break;
            }
        }
    }

    return true;
}

bool SPLRadioRoom::handle_param_set(const mavlink_message_t& msg, mavlink_message_t& ack)
{
    if (msg.msgid == MAVLINK_MSG_ID_PARAM_SET) {
        char param_id[17];
        mavlink_msg_param_set_get_param_id(&msg, param_id);
        if (strncmp(param_id, HL_REPORT_PERIOD_PARAM, 16) == 0) {
            float value = mavlink_msg_param_set_get_param_value(&msg);
            config.set_report_period(value);

            mavlink_param_value_t paramValue;
            paramValue.param_value = value;
            paramValue.param_count = 0;
            paramValue.param_index = 0;
            mavlink_msg_param_set_get_param_id(&msg, paramValue.param_id);
            paramValue.param_type = mavlink_msg_param_set_get_param_type(&msg);

            mavlink_msg_param_value_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &paramValue);

            return true;
        }
    }

    return false;
}

bool SPLRadioRoom::handle_mission_write(const mavlink_message_t& msg, mavlink_message_t& ack)
{
    mavlink_message_t missions[MAX_MISSION_COUNT];

    if (msg.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {
        printf("MISSION_COUNT MT message received.\n");

        uint16_t count = mavlink_msg_mission_count_get_count(&msg);

        if (count > MAX_MISSION_COUNT) {
            printf("Not enough memory for storing missions.\n");

            mavlink_mission_ack_t mission_ack;
            mission_ack.target_system = msg.sysid;
            mission_ack.target_component = msg.compid;
            mission_ack.type = MAV_MISSION_NO_SPACE;
            mavlink_msg_mission_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &mission_ack);
            ack.seq = 0; //TODO: use global counter for message sequence numbers.

            return true;
        }

        mavlink_message_t mt_msg, mo_msg;
        mo_msg.len = mo_msg.msgid = 0;

        printf("Receiving mission items from ISBD.\n");

        uint16_t idx = 0;

        for (uint16_t i = 0; i < count * MAX_SEND_RETRIES && idx < count; i++) {
            bool received = false;

            if (isbd_send_receive_message(mo_msg, mt_msg, received)) {
                if (received && mt_msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM) {
                    printf("MISSION_ITEM MT message received.\n");
                    memcpy(missions + idx, &mt_msg, sizeof(mavlink_message_t));
                    idx++;
                }
            } else {
                usleep(5000000);
            }
        }

        if (idx != count) {
            printf("Not all mission items received.\n");

            mavlink_mission_ack_t mission_ack;
            mission_ack.target_system = msg.sysid;
            mission_ack.target_component = msg.compid;
            mission_ack.type = MAV_MISSION_ERROR;
            mavlink_msg_mission_ack_encode(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID, &ack, &mission_ack);
            ack.seq = 0; //TODO: use global counter for message sequence numbers.

            return true;
        }

        printf("Sending mission items to ArduPilot...\n");

        for (int i = 0; i < MAX_SEND_RETRIES; i++) {
            if (ardupilot.send_message(msg)) {
                break;
            }

            usleep(10000);
        }

        for (uint16_t i = 0; i < count; i++) {
            ardupilot.send_receive_message(missions[i], ack);

            printf("Mission item sent to ArduPilot.\n");

            usleep(10000);
        }

        if (mavlink_msg_mission_ack_get_type(&ack) == MAV_MISSION_ACCEPTED) {
            printf("Mission accepted by ArduPilot.\n");
        } else {
            printf("Mission not accepted by ArduPilot: %d\n", mavlink_msg_mission_ack_get_type(&ack));
        }

        return true;
    }

    return false;
}

void SPLRadioRoom::isbd_session(mavlink_message_t& mo_msg)
{

    printf("ISBD session started.\n");

    bool received;
    bool ack_received = false;
    mavlink_message_t mt_msg;

    do {
        ack_received = false;

        if (isbd_send_receive_message(mo_msg, mt_msg, received)) {
            mo_msg.len = mo_msg.msgid = 0;
            if (received) {
                ack_received = handle_param_set(mt_msg, mo_msg);

                if (!ack_received) {
                    ack_received = handle_mission_write(mt_msg, mo_msg);
                    printf("MISSION_ACK received.\n");
                }

                if (!ack_received) {
                    ack_received = ardupilot.send_receive_message(mt_msg, mo_msg);

                    if (ack_received) {
                        printf("ACK received from ArduPilot.\n");
                        print_mavlink_msg(mo_msg);
                    }
                }
            }
        }
    } while (isbd.getWaitingMessageCount() > 0 || ack_received);

    printf("ISBD session ended.\n");
}

/**
 * Filters out MO messages from ArduPilot.
 *
 * Returns true if the message is allowed to pass though the filter.
boolean filterMessage(const mavlink_message_t& msg) {
  //TODO: Add all relevant messages
  return false;
}
*/

void SPLRadioRoom::comm_receive()
{
    mavlink_message_t msg;

    //digitalWrite(LED_PIN, LOW);

    if (ardupilot.receive_message(msg)) {
        //digitalWrite(LED_PIN, HIGH);

        high_latency_msg.update(msg);

        /*
        if (filterMessage(msg)) {
          print_mavlink_msg(msg);

          isbd_session(msg);
        }
        */
    }
}


/*
bool ISBDCallback() {
  mavlink_message_t msg;

  digitalWrite(LED_PIN, LOW);

  if (ardupilot.receiveMessage(msg)) {
    digitalWrite(LED_PIN, HIGH);

    high_latency_msg.update(msg);
  }

  nss.listen();

  return true;
}
*/

bool detect_autopilot(MAVLinkSerial& ardupilot, const char* device)
{
    mavlink_autopilot_version_t autopilot_version;
    uint8_t autopilot, mav_type, sys_id;

    if (!ardupilot.request_autopilot_version(autopilot, mav_type, sys_id, autopilot_version)) {
        printf("Autopilot not detected at serial device '%s'.\n", device);
        return false;
    }

    char buff[64];
    ardupilot.get_firmware_version(autopilot_version, buff, sizeof(buff));

    printf("Autopilot detected at serial device '%s'.\n", device);
    printf("Autopilot: %d, MAV type: %d, system id: %d, firmware version: %s\n", autopilot, mav_type, sys_id, buff);

    return true;
}

bool detect_isbd(IridiumSBD& isbd, const char* device) {
    int ret = isbd.begin();

    if (ret == ISBD_SUCCESS || ret == ISBD_ALREADY_AWAKE) {
        char model[256], imea[256];
        imea[0] = model[0] = 0;
        ret = isbd.getTransceiverModel(model, sizeof(model));
        if (ret == ISBD_SUCCESS) {
            ret = isbd.getTransceiverSerialNumber(imea, sizeof(imea));
            if (ret == ISBD_SUCCESS) {
                printf("%s (IMEA %s) detected at serial device '%s'.\n", model, imea, device);
                return true;
            }
        }
    }

    printf("ISBD transceiver not detected at serial device '%s'. Error code = %d.\n", device, ret);
    return false;
}

bool SPLRadioRoom::init()
{
    // Detecting autopilot's and ISBD transceiver's serial devices.

    std::string s(config.get_serials());

    size_t pos = 0;
    string token;
    vector<string> devices;

    while ((pos = s.find(",")) != string::npos) {
        token = s.substr(0, pos);
        devices.push_back(token);
        s.erase(0, pos + 1);
    }

    devices.push_back(s);

    bool autopilot_connected = false;

    if (telem.open(config.get_mavlink_serial(), config.get_mavlink_serial_speed()) == 0) {
        if (detect_autopilot(ardupilot, config.get_mavlink_serial())) {
            autopilot_connected = true;
        } else {
            telem.close();
        }
    } else {
        printf("Failed to open serial device '%s'.\n", config.get_mavlink_serial());
    }

    if (!autopilot_connected && config.get_auto_detect_serials()) {
        printf("WARN: Autopilot was not detected on the specified serial device. Trying to detect autopilot on other devices.\n");
        for (int i = 0; i < devices.size(); i++) {
            if (devices[i] == config.get_mavlink_serial())
                continue;

            if (telem.open(devices[i].data(), config.get_mavlink_serial_speed()) == 0) {
                if (detect_autopilot(ardupilot, devices[i].data())) {
                    autopilot_connected = true;
                    config.set_mavlink_serial(devices[i].data());
                    break;
                }

                telem.close();
            } else {
               printf("DEBUG: Failed to open serial device '%s'.\n", devices[i].data());
            }
        }
    }

    if (!autopilot_connected) {
        telem.open(config.get_mavlink_serial(), config.get_mavlink_serial_speed());
        printf("SEVERE: Autopilot was not detected on any specified serial devices (%s).\n",
               config.get_serials());
    }

    bool isbd_connected = false;

    isbd.setPowerProfile(1);

    if (strcmp(config.get_mavlink_serial(), config.get_isbd_serial()) != 0) {
        if (nss.open(config.get_isbd_serial(), config.get_isbd_serial_speed()) == 0) {
            if (detect_isbd(isbd, config.get_isbd_serial())) {
                isbd_connected = true;
            } else {
                nss.close();
            }
        } else {
            printf("Failed to open serial device '%s'.\n", config.get_isbd_serial());
        }
    }

    if (!isbd_connected && config.get_auto_detect_serials()) {
        printf("WARN: ISBD transceiver was not detected on the specified serial device. Trying to detect transceiver on other devices.\n");
        for (int i = 0; i < devices.size(); i++) {
            if (devices[i] == config.get_mavlink_serial() || devices[i] == config.get_isbd_serial())
                continue;

            if (nss.open(devices[i].data(), config.get_isbd_serial_speed()) == 0) {
                if (detect_isbd(isbd, devices[i].data())) {
                    isbd_connected = true;
                    config.set_isbd_serial(devices[i].data());
                    break;
                } else {
                    nss.close();
                }
            } else {
                printf("DEBUG: Failed to open serial device '%s'.\n", devices[i].data());
            }
        }
    }

    if (!isbd_connected) {
        nss.open(config.get_isbd_serial(), config.get_isbd_serial_speed());
        printf("SEVERE: ISBD transceiver was not detected on any of the specified serial devices (%s).\n",
               config.get_serials());
    }

    last_report_time = clock();

    return autopilot_connected && isbd_connected;
}

void SPLRadioRoom::loop()
{
    // Request data streams
    uint8_t req_stream_ids[] = {MAV_DATA_STREAM_EXTRA1, MAV_DATA_STREAM_EXTRA2, MAV_DATA_STREAM_EXTENDED_STATUS,
                                MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_RAW_CONTROLLER
                               };
    uint16_t req_message_rates[] = {2, 3, 2, 2, 2};

    for (size_t i = 0; i < sizeof(req_stream_ids)/sizeof(req_stream_ids[0]); i++) {
        mavlink_message_t msg;
        mavlink_msg_request_data_stream_pack(255, 1, &msg, 1, 1, req_stream_ids[i], req_message_rates[i], 1);
        ardupilot.send_message(msg);
        usleep(10000);
    }

    for (int i = 0; i < 100; i++) {
        comm_receive();
        usleep(10000);
    }

    high_latency_msg.print();

    uint16_t mo_flag = 0, mo_msn = 0, mt_flag = 0, mt_msn = 0, ra_flag = 0, msg_waiting = 0;

    int err = isbd.getStatusExtended(mo_flag, mo_msn, mt_flag, mt_msn, ra_flag, msg_waiting);

    if (err != 0) {
        printf("SBDSX failed: error %d\n", err);
    } else {
        printf("Ring Alert flag: %d\n", ra_flag);
    }

    clock_t current_time = clock();

    unsigned long elapsedTime = (current_time - last_report_time) / CLOCKS_PER_SEC;

    printf("Elapsed time: %d\n", elapsedTime);

    printf("Report period: %d\n", config.get_report_period());

    // Start ISBD session if ring alert is received or HIGH_LATENCY report period is elapsed.
    if (ra_flag || elapsedTime > config.get_report_period()) {
        //high_latency_msg.print();

        mavlink_message_t msg;
        high_latency_msg.encode(msg);

        isbd_session(msg);

        last_report_time = current_time;
    }
}
