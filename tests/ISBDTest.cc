/*
 ISBDTest.cpp

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
#include "IridiumSBD.h"

#define ISBD_BAUD_RATE     B19200

Serial sbdSerial;

HighLatencyMsg highLatencyMsg(ARDUPILOT_SYSTEM_ID, ARDUPILOT_COMPONENT_ID);


int main(int argc, char** argv) {

    sbdSerial.open("/dev/ttyUSB1", ISBD_BAUD_RATE);

    IridiumSBD isbd(sbdSerial);
    //isbd.attachConsole(std::cout);
    //isbd.attachDiags(std::cout);
    isbd.begin();

    for (int i = 0; i < 10; i++) {
        int signalQuality = 0;
        int ret = isbd.getSignalQuality(signalQuality);
        printf("Signal quality = %d, ret = %d\n", signalQuality, ret);

        uint16_t moFlag = 0;
        uint16_t moMSN = 0;
        uint16_t mtFlag = 0;
        uint16_t mtMSN = 0;
        uint16_t raFlag = 0;
        uint16_t msgWaiting = 0;
        ret = isbd.getStatusExtended(moFlag, moMSN, mtFlag, mtMSN, raFlag, msgWaiting);
        printf("Status extended: moFlag=%d, moMSN=%d, mtFlag=%d, mtMSN=%d, raFlag=%d, msgWaiting=%d, ret=%d\n",
                moFlag, moMSN, mtFlag, mtMSN, raFlag, msgWaiting, ret);


        ret = isbd.getWaitingMessageCount();
        printf("Waiting message count = %d\n", ret);

        printf("Is asleep = %d\n", isbd.isAsleep());

        int sri =0;
        ret = isbd.queryRingIndicationStatus(sri);
        printf("Ring indication status = %d\n", sri);

        uint8_t txData[340];
        int txDataSize = snprintf((char *)txData, 340, "MSG%d", i);

        uint8_t rxBuffer[340];
        size_t rxSize = 340;
        rxBuffer[0] = 0;
        ret = isbd.sendReceiveSBDBinary(txData, txDataSize, rxBuffer, rxSize);

        if (ret == ISBD_SUCCESS) {
          rxBuffer[rxSize] = 0;
          for (int i = 0; i < rxSize; i++)
              printf("%d\n", rxBuffer[i]);
          printf("SendReceive SBD Binary: ret = %d, data = %s, rxSize = %d\n", ret, rxBuffer, rxSize);

        } else {
          printf("SendReceive SBD Binary: ret = %d\n", ret);
        }

        //ret = isbd.sendReceiveSBDText("Hello!", rxBuffer, rxBufferSize);
        //printf("SendReceive SBD Text: rxBufferSize = %d, ret = %d\n", rxBufferSize, ret);

        //ret = isbd.sendSBDBinary(txData, sizeof(txData));
        //printf("Send SBD Binary: ret = %d\n", ret);

        //ret = isbd.sendSBDText("Hello!");
        //printf("Send SBD Text: ret = %d\n", ret);
    }

    return 0;

}
