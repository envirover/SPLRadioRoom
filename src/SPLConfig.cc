/*
 SPLConfig.cpp

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

#include "SPLConfig.h"
#include "INIReader.h"

SPLConfig config;

SPLConfig::SPLConfig() :
    autopilot_serial(DEFAULT_AUTOPILOT_SERIAL),
    isbd_serial(DEFAULT_ISBD_SERIAL),
    autopilot_serial_speed(AUTOPILOT_SERIAL_BAUD_RATE),
    isbd_serial_speed(ISBD_SERIAL_BAUD_RATE),
    auto_detect_serials(true),
    report_period(DEFAULT_REPORT_PERIOD),
    debug_mode(false)
{
}

int SPLConfig::init(const std::string& config_file)
{
    INIReader conf(config_file);

    int ret = conf.ParseError();

    if (ret < 0) {
        return ret;
    }

    set_autopilot_serial(conf.Get(AUTOPILOT_CONFIG_SECTION,
                                  AUTOPILOT_SERIAL_PROPERTY,
                                  DEFAULT_AUTOPILOT_SERIAL));

    set_autopilot_serial_speed(conf.GetInteger(AUTOPILOT_CONFIG_SECTION,
                                               AUTOPILOT_SERIAL_SPEED_PROPERTY,
                                               AUTOPILOT_SERIAL_BAUD_RATE));

    set_isbd_serial(conf.Get(ISBD_CONFIG_SECTION,
                             ISBD_SERIAL_PROPERTY,
                             DEFAULT_ISBD_SERIAL));

    set_isbd_serial_speed(conf.GetInteger(ISBD_CONFIG_SECTION,
                                          ISBD_SERIAL_SPEED_PROPERTY,
                                          ISBD_SERIAL_BAUD_RATE));

    set_auto_detect_serials(conf.GetBoolean(RADIOROOM_CONFIG_SECTION,
                                            AUTO_DETECT_SERIALS_PROPERTY,
                                            true));

    set_report_period(conf.GetReal(RADIOROOM_CONFIG_SECTION,
                                   REPORT_PERIOD_PROPERTY,
                                   DEFAULT_REPORT_PERIOD));

    return 0;
}

bool SPLConfig::get_debug_mode() const
{
    return debug_mode;
}

void SPLConfig::set_debug_mode(bool debug)
{
    debug_mode = debug;
}

std::string SPLConfig::get_autopilot_serial() const
{
    return autopilot_serial;
}

void SPLConfig::set_autopilot_serial(const std::string& path)
{
    autopilot_serial = path;
}

int SPLConfig::get_autopilot_serial_speed() const
{
    return autopilot_serial_speed;
}

void SPLConfig::set_autopilot_serial_speed(int speed)
{
    autopilot_serial_speed = speed;
}

std::string SPLConfig::get_isbd_serial() const
{
    return isbd_serial;
}

void SPLConfig::set_isbd_serial(const std::string& path)
{
    isbd_serial = path;
}

int SPLConfig::get_isbd_serial_speed() const
{
    return isbd_serial_speed;
}

void SPLConfig::set_isbd_serial_speed(int speed)
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

double SPLConfig::get_report_period() const
{
    return report_period;
}

void SPLConfig::set_report_period(double period)
{
    report_period = period;
}

