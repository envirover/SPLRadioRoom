/*
 SPLConfig.cpp

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
 */

#include "SPLConfig.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

SPLConfig config;

SPLConfig::SPLConfig() :
    mavlink_serial(DEFAULT_MAVLINK_SERIAL),
    isbd_serial(DEFAULT_ISBD_SERIAL),
    mavlink_serial_speed(AP_TELEM_BAUD_RATE),
    isbd_serial_speed(ISBD_BAUD_RATE),
    auto_detect_serials(true),
    report_period(DEFAULT_REPORT_PERIOD)
{
}

int SPLConfig::init(int argc, char** argv)
{
    int c;

    while ((c = getopt(argc, argv, "mi:")) != -1) {
        switch (c) {
        case 'm':
            mavlink_serial = optarg;
            break;
        case 'i':
            isbd_serial = optarg;
            break;
        case '?':
            if (optopt == 'm' || optopt == 'i') {
                fprintf(stderr, "Option -%c requires an argument.\n", optopt);
            } else if (isprint(optopt)) {
                fprintf(stderr, "Unknown option `-%c'.\n", optopt);
            } else {
                fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
            }
            return 1;
        default:
            abort();
        }
    }

    return 0;
}

const char* SPLConfig::get_mavlink_serial() const
{
    return mavlink_serial.data();
}

void SPLConfig::set_mavlink_serial(const char* path)
{
    mavlink_serial = path;
}

speed_t SPLConfig::get_mavlink_serial_speed() const
{
    return mavlink_serial_speed;
}

void SPLConfig::set_mavlink_serial_speed(speed_t speed)
{
    mavlink_serial_speed = speed;
}

const char* SPLConfig::get_isbd_serial() const
{
    return isbd_serial.data();
}

void SPLConfig::set_isbd_serial(const char* path)
{
    isbd_serial = path;
}

speed_t SPLConfig::get_isbd_serial_speed() const
{
    return isbd_serial_speed;
}

void SPLConfig::set_isbd_serial_speed(speed_t speed)
{
    isbd_serial_speed = speed;
}

bool SPLConfig::get_auto_detect_serials() const
{
    return auto_detect_serials;
}

void SPLConfig::set_auto_detect_serials(bool a)
{
    auto_detect_serials = a;
}

unsigned long SPLConfig::get_report_period() const
{
    return report_period;
}

void SPLConfig::set_report_period(unsigned long period)
{
    report_period = period;
}

