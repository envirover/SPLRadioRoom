/*
 Config.cc

 This file is a part of RadioRoom project.

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

#include "Config.h"
#include "INIReader.h"

Config config;

Config::Config() :
    autopilot_serial(DEFAULT_AUTOPILOT_SERIAL),
    autopilot_serial_speed(AUTOPILOT_SERIAL_BAUD_RATE),
    auto_detect_serials(true),
    debug_mode(false),
    isbd_enabled(DEFAULT_ISBD_ENABLED),
    isbd_serial(DEFAULT_ISBD_SERIAL),
    isbd_serial_speed(ISBD_SERIAL_BAUD_RATE),
    isbd_report_period(DEFAULT_ISBD_REPORT_PERIOD),
    tcp_enabled(DEFAULT_TCP_ENABLED),
    tcp_host(DEFAULT_TCP_HOST),
    tcp_port(DEFAULT_TCP_PORT),
    tcp_report_period(DEFAULT_TCP_REPORT_PERIOD)
{
}

int Config::init(const std::string& config_file)
{
    INIReader conf(config_file);

    int ret = conf.ParseError();

    if (ret < 0) {
        return ret;
    }

    /* [autopilot] config section */

    set_autopilot_serial(conf.Get(AUTOPILOT_CONFIG_SECTION,
                                  AUTOPILOT_SERIAL_PROPERTY,
                                  DEFAULT_AUTOPILOT_SERIAL));

    set_autopilot_serial_speed(conf.GetInteger(AUTOPILOT_CONFIG_SECTION,
                                               AUTOPILOT_SERIAL_SPEED_PROPERTY,
                                               AUTOPILOT_SERIAL_BAUD_RATE));

    /* [radioroom] config section */

    set_auto_detect_serials(conf.GetBoolean(RADIOROOM_CONFIG_SECTION,
                                            AUTO_DETECT_SERIALS_PROPERTY,
                                            true));

    /* [isbd] config section */

    set_isbd_enabled(conf.GetBoolean(ISBD_CONFIG_SECTION,
                                     ISBD_ENABLED_PROPERTY,
                                     DEFAULT_ISBD_ENABLED));

    set_isbd_serial(conf.Get(ISBD_CONFIG_SECTION,
                             ISBD_SERIAL_PROPERTY,
                             DEFAULT_ISBD_SERIAL));

    set_isbd_serial_speed(conf.GetInteger(ISBD_CONFIG_SECTION,
                                          ISBD_SERIAL_SPEED_PROPERTY,
                                          ISBD_SERIAL_BAUD_RATE));


    set_isbd_report_period(conf.GetReal(ISBD_CONFIG_SECTION,
                                        REPORT_PERIOD_PROPERTY,
                                        DEFAULT_ISBD_REPORT_PERIOD));

    /* [tcp] config section */

    set_tcp_enabled(conf.GetBoolean(TCP_CONFIG_SECTION,
                                    TCP_ENABLED_PROPERTY,
                                    DEFAULT_TCP_ENABLED));

    set_tcp_host(conf.Get(TCP_CONFIG_SECTION,
                          TCP_HOST_PROPERTY,
                          DEFAULT_TCP_HOST));

    set_tcp_port(conf.GetInteger(TCP_CONFIG_SECTION,
                                 TCP_PORT_PROPERTY,
                                 DEFAULT_TCP_PORT));

    set_tcp_report_period(conf.GetReal(TCP_CONFIG_SECTION,
                                       REPORT_PERIOD_PROPERTY,
                                       DEFAULT_TCP_REPORT_PERIOD));
    return 0;
}

bool Config::get_debug_mode() const
{
    return debug_mode;
}

void Config::set_debug_mode(bool debug)
{
    debug_mode = debug;
}

std::string Config::get_autopilot_serial() const
{
    return autopilot_serial;
}

void Config::set_autopilot_serial(const std::string& path)
{
    autopilot_serial = path;
}

int Config::get_autopilot_serial_speed() const
{
    return autopilot_serial_speed;
}

void Config::set_autopilot_serial_speed(int speed)
{
    autopilot_serial_speed = speed;
}

bool Config::get_isbd_enabled() const
{
    return isbd_enabled;
}

void Config::set_isbd_enabled(bool enabled)
{
    isbd_enabled = enabled;
}

std::string Config::get_isbd_serial() const
{
    return isbd_serial;
}

void Config::set_isbd_serial(const std::string& path)
{
    isbd_serial = path;
}

int Config::get_isbd_serial_speed() const
{
    return isbd_serial_speed;
}

void Config::set_isbd_serial_speed(int speed)
{
    isbd_serial_speed = speed;
}

bool Config::get_auto_detect_serials() const
{
    return auto_detect_serials;
}

void Config::set_auto_detect_serials(bool a)
{
    auto_detect_serials = a;
}

double Config::get_isbd_report_period() const
{
    return isbd_report_period;
}

void Config::set_isbd_report_period(double period)
{
    isbd_report_period = period;
}

bool Config::get_tcp_enabled() const
{
    return tcp_enabled;
}

void Config::set_tcp_enabled(bool enabled)
{
    tcp_enabled = enabled;
}

std::string Config::get_tcp_host() const
{
    return tcp_host;
}

void Config::set_tcp_host(const std::string& host)
{
    tcp_host = host;
}

int Config::get_tcp_port() const
{
    return tcp_port;
}

void Config::set_tcp_port(int port)
{
    tcp_port = port;
}

double Config::get_tcp_report_period() const {
    return tcp_report_period;
}

void Config::set_tcp_report_period(double period) {
    tcp_report_period = period;
}
