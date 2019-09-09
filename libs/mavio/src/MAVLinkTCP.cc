/*
 MAVLinkTcpClient.cc

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

 Created on: Mar 5, 2018
     Author: Pavel Bobov
*/

#include "MAVLinkTCP.h"

#include "MAVLinkLogger.h"
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <syslog.h>

using namespace mavio;

constexpr int poll_timeout = 100; // milliseconds

MAVLinkTCP::MAVLinkTCP()
    : address(""), port(0), socket_fd(0), timeout(1000), start_millis(0)
{
}

MAVLinkTCP::~MAVLinkTCP()
{
    close();
}

bool MAVLinkTCP::init(const std::string address, uint16_t port)
{
    if (address.empty()) {
        return false;
    }

    this->address = address;
    this->port    = port;

    struct hostent* server = ::gethostbyname(address.c_str());

    if (server == NULL) {
        syslog(LOG_ERR, "No such host '%s'.", address.c_str());
        return false;
    }

    socket_fd = ::socket(AF_INET, SOCK_STREAM, 0);

    if (socket_fd < 0) {
        syslog(LOG_ERR, "Socket creation failed.");
        socket_fd = 0;
        return false;
    }

    struct sockaddr_in serv_addr;

    memset((char*)&serv_addr, 0, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;

    memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);

    serv_addr.sin_port = htons(port);

    if (::connect(socket_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        ::close(socket_fd);
        socket_fd = 0;
        syslog(LOG_ERR, "Connection to 'tcp://%s:%d' failed.", address.c_str(), port);
        return false;
    }

    struct timeval tv;
    tv.tv_sec  = 1; /* 1 sec timeout */
    tv.tv_usec = 0; // Not init'ing this can cause strange errors
    if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv))) {
        syslog(LOG_ERR, "Failed to set socket receive timeout. (errno = %d)", errno);
    }

    syslog(LOG_NOTICE, "Connected to 'tcp://%s:%d'.", address.c_str(), port);

    return true;
}

void MAVLinkTCP::close()
{
    if (socket_fd != 0) {
        ::close(socket_fd);
        socket_fd = 0;
    }
}

bool MAVLinkTCP::send_message(const mavlink_message_t& msg)
{
    if (msg.len == 0 && msg.msgid == 0) {
        return true;
    }

    if (socket_fd == 0) {
        init(address, port);
    }

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    //Copy the message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    uint16_t n = ::send(socket_fd, buf, len, 0);

    if (n == len) {
        MAVLinkLogger::log(LOG_INFO, "TCP <<", msg);
    } else {
        MAVLinkLogger::log(LOG_WARNING, "TCP << FAILED", msg);
        close();
        init(address, port);
    }

    return n == len;
}

bool MAVLinkTCP::receive_message(mavlink_message_t& msg)
{
    if (socket_fd == 0) {
        return false;
    }

    if (!message_available()) {
        return false;
    }

    uint8_t stx;
    int     rc = ::recv(socket_fd, &stx, 1, 0);

    if (rc > 0) {
        if (stx != MAVLINK_STX) {
            return false;
        }

        uint8_t payload_length;
        rc = ::recv(socket_fd, &payload_length, 1, 0);

        if (rc > 0) {
            uint8_t buffer[263];
            rc = ::recv(socket_fd, buffer, payload_length + 6, MSG_WAITALL);

            if (rc > 0) {
                mavlink_status_t mavlink_status;

                mavlink_parse_char(MAVLINK_COMM_0, stx, &msg, &mavlink_status);
                mavlink_parse_char(MAVLINK_COMM_0, payload_length, &msg, &mavlink_status);

                for (int i = 0; i < rc; i++) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &mavlink_status)) {
                        MAVLinkLogger::log(LOG_INFO, "TCP >>", msg);
                        return true;
                    }
                }
            }
        }
    }

    if (rc < 0) {
        syslog(LOG_ERR, "Failed to receive MAVLink message from socket (errno = %d).", errno);
    } else if (rc == 0) {
        MAVLinkLogger::log(LOG_WARNING, "TCP >> FAILED (The stream socket peer has performed an orderly shutdown)", msg);
        close();
        init(address, port);
    } else {
        syslog(LOG_ERR, "Failed to parse MAVLink message.");
    }

    return false;
}

bool MAVLinkTCP::message_available()
{
    if (socket_fd == 0) {
        return false;
    }

    struct pollfd fds[1];
    int           nfds = 1;

    memset(fds, 0, sizeof(fds));

    fds[0].fd     = socket_fd;
    fds[0].events = POLLIN;

    return ::poll(fds, nfds, poll_timeout) > 0;
}
