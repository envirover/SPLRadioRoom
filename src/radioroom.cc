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
#include "version.h"


#define DEFAULT_PID_FILE "/var/run/radioroom.pid"

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
    std::cout << RADIO_ROOM_VERSION << std::endl;
    std::cout << "MAVLink wire protocol version " << MAVLINK_WIRE_PROTOCOL_VERSION << std::endl;
}

void handle_signal(int sig)
{
    if (sig == SIGINT || sig == SIGTERM) {
        running = 0;

        /* Reset signal handling to default behavior */
        signal(SIGINT, SIG_DFL);
        signal(SIGTERM, SIG_DFL);
    }
}

//void daemonize(const std::string& pid_file_name)
//{
//    //Fork the Parent Process
//     pid_t pid = fork();
//
//    if (pid < 0) {
//        exit(EXIT_FAILURE);
//    }
//
//    //We got a good pid, Close the Parent Process
//    if (pid > 0) {
//        exit(EXIT_SUCCESS);
//    }
//
//    //Change file mask
//    umask(0);
//
//    //Create a new signature id for the child.
//     if (setsid() < 0) {
//        exit(EXIT_FAILURE);
//    }
//
//    /* Ignore signal sent from child to parent process */
//    signal(SIGCHLD, SIG_IGN);
//
//    //Change Directory
//    if ((chdir("/")) < 0) {
//        exit(EXIT_FAILURE);
//    }
//
//    /* Try to write PID of daemon to lockfile */
//    if (!pid_file_name.empty())
//    {
//        char str[256];
//
//        int pid_fd = open(pid_file_name.data(), O_RDWR|O_CREAT, 0640);
//
//        if (pid_fd < 0) {
//            /* Can't open lockfile */
//            std::cout << "Failed to open PID file '" << pid_file_name << "'." << std::endl;
//            std::cout << "Run radioroom as superuser or specify an alternative PID file."  << std::endl;
//            exit(EXIT_FAILURE);
//        }
//
//        if (lockf(pid_fd, F_TLOCK, 0) < 0) {
//            /* Can't lock file */
//            std::cout << "Failed to lock file '" << pid_file_name << "'." << std::endl;
//            std::cout << "radioroom process is already started." << std::endl;
//            exit(EXIT_FAILURE);
//        }
//
//        /* Get current PID */
//        sprintf(str, "%d\n", getpid());
//
//        /* Write PID to lockfile */
//        write(pid_fd, str, strlen(str));
//    }
//
//    /* Reopen stdin (fd = 0), stdout (fd = 1), stderr (fd = 2) */
//    stdin = fopen("/dev/null", "r");
//    stdout = fopen("/dev/null", "w+");
//    stderr = fopen("/dev/null", "w+");
//}

int main(int argc, char** argv) {
    std::string config_file = DEFAULT_CONFIG_FILE;
    //std::string pid_file    = DEFAULT_PID_FILE;
    //bool nodetach = false;

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

    syslog(LOG_NOTICE, "Starting %s...", RADIO_ROOM_VERSION);

    if (config.init(config_file) < 0) {
        syslog(LOG_ERR, "Can't load configuration file '%s'", config_file.data());
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

     if (radioroom.init()) {
        syslog(LOG_NOTICE, "%s started.", RADIO_ROOM_VERSION);
    } else {
        syslog(LOG_CRIT, "%s initialization failed.", RADIO_ROOM_VERSION);
    }

    running = 1;

    while (running) {
        radioroom.loop();
    }

    radioroom.close();

    syslog(LOG_NOTICE, "%s stopped.", RADIO_ROOM_VERSION);

    closelog();

    return EXIT_SUCCESS;
}
