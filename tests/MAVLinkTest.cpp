/*
 MAVLinkTest.cpp

 This file is a part of SPLRadioRoom project.

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
 */

#include <stdio.h>
#include <unistd.h>
#include "MAVLinkSerial.h"
#include "HighLatencyMsg.h"

#define AP_TELEM_BAUD_RATE B57600

Serial telem;
MAVLinkSerial  ardupilot(telem);

Serial sbdSerial;

HighLatencyMsg highLatencyMsg(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID);

/*
 * Debug print of mavlink_message_t message
 */
void printMavlinkMsg(const mavlink_message_t& msg) {
  printf("**\n");
  printf("msgid = %d\n", msg.msgid);
  printf("compid = %d\n", msg.compid);
}

/*
 * Reads and processes MAVLink messages from ArduPilot.
 */
void commReceive() {
  mavlink_message_t msg;

  //digitalWrite(LED_PIN, LOW);

  if (ardupilot.receive_message(msg)) {
    //digitalWrite(LED_PIN, HIGH);

    highLatencyMsg.update(msg);

    printMavlinkMsg(msg);
  }
}


void setup() {
  int ret = telem.open("/dev/USB0", AP_TELEM_BAUD_RATE);
  if (ret != 0) {
      printf("Failed to connect to autopilot.\n");
  }

  for (int i = 0; i < 100; i++) {
      mavlink_message_t msg;
      mavlink_msg_command_long_pack(255, 1, &msg, 1, 0, MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, i, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      if (!ardupilot.send_message(msg)) {
          printf("Failed to send message to autopilot.\n");
      }

      usleep(10000);

      mavlink_message_t msg2;

      if (ardupilot.receive_message(msg2)) {
          printf("**** msg.msgid = %d\n", msg2.msgid);
          if (msg2.msgid == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
              mavlink_autopilot_version_t autopilot_version;
              mavlink_msg_autopilot_version_decode(&msg2, &autopilot_version);
              printf("%u.%u.%u.%u.%u.%u\n",
                      autopilot_version.capabilities,
                      autopilot_version.uid,
                      autopilot_version.flight_custom_version,
                      autopilot_version.flight_sw_version,
                      autopilot_version.middleware_custom_version,
                      autopilot_version.middleware_sw_version);
          }
      }

  }

}

void loop() {
  uint8_t req_stream_ids[] = {MAV_DATA_STREAM_EXTRA1, MAV_DATA_STREAM_EXTRA2, MAV_DATA_STREAM_EXTENDED_STATUS,
                              MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_RAW_CONTROLLER};
  uint16_t req_message_rates[] = {2, 3, 2, 2, 2};

  for (size_t i = 0; i < sizeof(req_stream_ids)/sizeof(req_stream_ids[0]); i++) {
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(255, 1, &msg, 1, 1, req_stream_ids[i], req_message_rates[i], 1);
    ardupilot.send_message(msg);
    usleep(10000);
  }

  for (int i = 0; i < 100; i++) {
    commReceive();

    usleep(10000);
  }

  printf("*** HIGH_LATENCY ***\n");
  highLatencyMsg.print();
}

int main(int argc, char** argv) {
    setup();

    while(1) {
        loop();
    }

    return 0;

}
