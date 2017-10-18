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
#include "SPLRadioRoom.h"

SPLRadioRoom::SPLRadioRoom() :
    high_latency_msg(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID),
    ardupilot(telem), isbd(nss), last_report_time(0)

{
    // TODO Auto-generated constructor stub

}

SPLRadioRoom::~SPLRadioRoom()
{
    // TODO Auto-generated destructor stub
}

/*
 * Debug print of mavlink_message_t message
 */
void SPLRadioRoom::print_mavlink_msg(const mavlink_message_t& msg)
{
    printf("** \nmsgid = %d\ncompid = %d\n", msg.msgid, msg.compid);
}

/**
 * Sends MT message to ISBD and receives MO message from the inbound message queue if any.
 *
 * Returns true if the ISBD session succeeded.
 */
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


/*
 * Updates HIGH_LATENCY message reporting period if HL_REPORT_PERIOD parameter value is set by
 * PARAM_SET MT message.
 *
 * Returns true if the message was handled.
 */
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

/*
 * Handles writing waypoints list as described  in
 * http://qgroundcontrol.org/mavlink/waypoint_protocol
 *
 * returns true if waypoints list was updated in ardupilot
 */
bool SPLRadioRoom::handle_mission_write(const mavlink_message_t& msg, mavlink_message_t& ack)
{
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

/*
 * Sends the message to ISBD, recieve all the messages in the
 * inbound message queue, if any, pass them to ArduPilot,
 * sends ACKs back to ISBD.
 */
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

/*
 * Reads and processes MAVLink messages from ArduPilot.
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


void SPLRadioRoom::setup()
{
    telem.open("/dev/ttyUSB0", AP_TELEM_BAUD_RATE);

    config.init();
    config.set_report_period(DEFAULT_REPORT_PERIOD);

    // Init SBD
    nss.open("/dev/ttyUSB1", ISBD_BAUD_RATE);

    //isbd.attachConsole(std::stdout);
    //isbd.attachDiags(std::stdout);
    isbd.setPowerProfile(1);
    isbd.begin();

    int signalQuality = -1;
    int err = isbd.getSignalQuality(signalQuality);
    if (err != 0) {
        printf("SignalQuality failed: error %d\n", err);
    } else {
        printf("SignalQuality: %d\n", signalQuality);
    }

    last_report_time = clock();
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
