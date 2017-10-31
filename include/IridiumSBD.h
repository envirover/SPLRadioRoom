/*
IridiumSBD.h

POSIX version of IridiumSBD library for Iridium SBD ("Short Burst Data") Communications.

Original work Copyright (C) 2013-4 Mikal Hart
Modified work Copyright (C) 2017   Envirover

All rights reserved.

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

#include <stdlib.h>
#include <iostream>
#include "Serial.h"

#define ISBD_LIBRARY_REVISION           2

#define ISBD_DEFAULT_AT_TIMEOUT         30
#define ISBD_DEFAULT_CSQ_INTERVAL       10
#define ISBD_DEFAULT_CSQ_INTERVAL_USB   20
#define ISBD_DEFAULT_SBDIX_INTERVAL     30
#define ISBD_DEFAULT_SBDIX_INTERVAL_USB 30
#define ISBD_DEFAULT_SENDRECEIVE_TIME   300
#define ISBD_STARTUP_MAX_TIME           240
#define ISBD_DEFAULT_CSQ_MINIMUM        2

#define ISBD_SUCCESS             0
#define ISBD_ALREADY_AWAKE       1
#define ISBD_SERIAL_FAILURE      2
#define ISBD_PROTOCOL_ERROR      3
#define ISBD_CANCELLED           4
#define ISBD_NO_MODEM_DETECTED   5
#define ISBD_SBDIX_FATAL_ERROR   6
#define ISBD_SENDRECEIVE_TIMEOUT 7
#define ISBD_RX_OVERFLOW         8
#define ISBD_REENTRANT           9
#define ISBD_IS_ASLEEP           10
#define ISBD_NO_SLEEP_PIN        11

#define ISBD_MAX_MT_MGS_SIZE 270

typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;

extern bool isbdCallback() __attribute__((weak));

/**
 * POSIX implementation for Iridium SBD transceiver communication.
 */
class IridiumSBD {

    Serial& stream; // Communicating with the Iridium

    // Timings
    int csqInterval;
    int sbdixInterval;
    int atTimeout; //seconds
    int sendReceiveTimeout;

    // State variables
    int remainingMessages;
    int sleepPin;
    bool asleep;
    bool reentrant;
    int  minimumCSQ;
    bool useWorkaround;
    unsigned long lastPowerOnTime;

public:
    IridiumSBD(Serial& serial) :
        stream(serial),
        csqInterval(ISBD_DEFAULT_CSQ_INTERVAL),
        sbdixInterval(ISBD_DEFAULT_SBDIX_INTERVAL),
        atTimeout(ISBD_DEFAULT_AT_TIMEOUT),
        sendReceiveTimeout(ISBD_DEFAULT_SENDRECEIVE_TIME),
        remainingMessages(-1),
        sleepPin(-1),
        asleep(false),
        reentrant(false),
        minimumCSQ(ISBD_DEFAULT_CSQ_MINIMUM),
        useWorkaround(true),
        lastPowerOnTime(0UL)
    {
    }

    int begin();

    int getTransceiverModel(char *buffer, size_t bufferSize);
    int getTransceiverSerialNumber(char *buffer, size_t bufferSize);
    int sendSBDText(const char *message);
    int sendSBDBinary(const uint8_t *txData, size_t txDataSize);
    int sendReceiveSBDText(const char *message, uint8_t *rxBuffer, size_t &rxBufferSize);
    int sendReceiveSBDBinary(const uint8_t *txData, size_t txDataSize, uint8_t *rxBuffer, size_t &rxBufferSize);
    int getSignalQuality(int &quality);
    int queryRingIndicationStatus(int &sri);

    //This command returns current state of the mobile originated and mobile terminated buffers,
    //and the SBD ring alert status.
    int getStatusExtended(uint16_t &moFlag, uint16_t &moMSN, uint16_t &mtFlag, uint16_t &mtMSN, uint16_t &raFlag, uint16_t &msgWaiting);

    int getWaitingMessageCount();
    int sleep();
    bool isAsleep();

    void setPowerProfile(int profile);          // 0 = direct connect (default), 1 = USB
    void adjustATTimeout(int seconds);          // default value = 20 seconds
    void adjustSendReceiveTimeout(int seconds); // default value = 300 seconds
    void setMinimumSignalQuality(int quality);  // a number between 1 and 5, default ISBD_DEFAULT_CSQ_MINIMUM
    void useMSSTMWorkaround(bool useWorkAround); // true to use workaround from Iridium Alert 5/7

private:

    // Internal utilities
    bool smartWait(int seconds);
    bool waitForATResponse(char *response=NULL, int responseSize=0, const char *prompt=NULL, const char *terminator="OK\r\n");

    int  internalBegin();
    int  internalGetTransceiverModel(char *buffer, size_t bufferSize);
    int  internalGetTransceiverSerialNumber(char *buffer, size_t bufferSize);
    int  internalSendReceiveSBD(const char *txTxtMessage, const uint8_t *txData, size_t txDataSize, uint8_t *rxBuffer, size_t *prxBufferSize);
    int  internalQueryRingIndicationStatus(int &sri);
    int  internalGetStatusExtended(uint16_t &moFlag, uint16_t &moMSN, uint16_t &mtFlag, uint16_t &mtMSN, uint16_t &raFlag, uint16_t &msgWaiting);
    int  internalGetSignalQuality(int &quality);
    int  internalMSSTMWorkaround(bool &okToProceed);
    int  internalSleep();

    int  doSBDIX(uint16_t &moCode, uint16_t &moMSN, uint16_t &mtCode, uint16_t &mtMSN, uint16_t &mtLen, uint16_t &mtRemaining);
    int  doSBDRB(uint8_t *rxBuffer, size_t *prxBufferSize); // in/out
    int  readUInt(uint16_t &u);
    void power(bool on);

    void send(const char *str);
    void send(uint16_t n);

    bool cancelled();
};
