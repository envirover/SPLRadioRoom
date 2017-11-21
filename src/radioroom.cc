/*
 radioroom.cpp

 This file is a part of SPL RadioRoom project.

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

#include "SPLRadioRoom.h"
#include <syslog.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>

#include "build.h"

#define LOG_IDENTITY     "radioroom"

SPLRadioRoom radioroom;

static int running = 0;

void print_help()
{
    std::cout << "Usage: radioroom [options]" << std::endl;
    std::cout << "options:" << std::endl;
    std::cout << "    -c <config file>" << std::endl;
    std::cout << "          Alternative configuration file instead of /etc/radioroom.conf." << std::endl;
    std::cout << std::endl;
    std::cout << "    -h    Print this help and exit." << std::endl;
    std::cout << std::endl;
    std::cout << "    -v    Verbose logging." << std::endl;
    std::cout << std::endl;
    std::cout << "    -V    Print version and exit." << std::endl;
}

void print_version()
{
    std::cout << RADIO_ROOM_VERSION << "." << BUILD_NUM << std::endl;
    std::cout << "MAVLink wire protocol version " << MAVLINK_WIRE_PROTOCOL_VERSION << std::endl;
}

void handle_signal(int sig)
{
    if (sig == SIGTERM) {
        running = 0;

        /* Reset signal handling to default behavior */
        signal(SIGTERM, SIG_DFL);
    }
}

int main(int argc, char** argv) {
    std::string config_file = DEFAULT_CONFIG_FILE;

    int c;
    while ((c = getopt(argc, argv, "c:hvV")) != -1) {
        switch (c) {
        case 'c':
            config_file = optarg;
            break;
        case 'h':
            print_help();
            return EXIT_SUCCESS;
        case 'v':
            config.set_debug_mode(true);
            break;
        case 'V':
            print_version();
            return EXIT_SUCCESS;
        case '?':
            if (optopt == 'c') {
                std::cout << "Option -c requires an argument." << std::endl;
            } else if (isprint(optopt)) {
                std::cout << "Unknown option '-" << std::string(1, optopt) << "'." << std::endl;
            } else {
                std::cout << "Unknown option character '" << std::string(1, optopt) << "'." << std::endl;
            }
            return EXIT_FAILURE;
        }
    }

    openlog(LOG_IDENTITY, LOG_CONS | LOG_NDELAY, LOG_USER);
    setlogmask(config.get_debug_mode() ? LOG_UPTO(LOG_DEBUG) : LOG_UPTO(LOG_INFO));

    syslog(LOG_INFO, "Starting %s.%s...", RADIO_ROOM_VERSION, BUILD_NUM);

    if (config.init(config_file) < 0) {
        syslog(LOG_ERR, "Can't load configuration file '%s'", config_file.data());
    }

    signal(SIGTERM, handle_signal);

     if (radioroom.init()) {
        syslog(LOG_NOTICE, "%s.%s started.", RADIO_ROOM_VERSION, BUILD_NUM);
    } else {
        syslog(LOG_CRIT, "%s.%s initialization failed.", RADIO_ROOM_VERSION, BUILD_NUM);
        return EXIT_FAILURE;
    }

    running = 1;

    while (running) {
        radioroom.loop();
    }

    syslog(LOG_INFO, "Stopping %s.%s...", RADIO_ROOM_VERSION, BUILD_NUM);

    radioroom.close();

    syslog(LOG_NOTICE, "%s.%s stopped.", RADIO_ROOM_VERSION, BUILD_NUM);

    closelog();

    return EXIT_SUCCESS;
}
