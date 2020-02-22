/*
 Config.cc

 This file is a part of RadioRoom project.

 (C) Copyright 2019 Envirover.

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

namespace radioroom {

Config config;

constexpr char default_autopilot_serial[] = "/dev/ttyusb0";
constexpr int autopilot_serial_baud_rate = 57600;

constexpr bool default_isbd_enabled = true;
constexpr char default_isbd_serial[] = "/dev/ttyusb1";
constexpr int isbd_serial_baud_rate = 19200;

constexpr bool default_tcp_enabled = false;
constexpr char default_tcp_host[] = "";
constexpr int default_tcp_port = 5060;

constexpr double default_isbd_report_period = 300.0;  // 5 minutes
constexpr double default_tcp_report_period = 60.0;    // 1 minute

// radioroom.conf properties
constexpr char autopilot_config_section[] = "autopilot";
constexpr char autopilot_serial_property[] = "serial";
constexpr char autopilot_serial_speed_property[] = "serial_speed";

constexpr char radioroom_config_section[] = "radioroom";
constexpr char auto_detect_serials_property[] = "auto_detect_serials";
constexpr char report_period_property[] = "report_period";

constexpr char isbd_config_section[] = "isbd";
constexpr char isbd_enabled_property[] = "enabled";
constexpr char isbd_serial_property[] = "serial";
constexpr char isbd_serial_speed_property[] = "serial_speed";

constexpr char tcp_config_section[] = "tcp";
constexpr char tcp_enabled_property[] = "enabled";
constexpr char tcp_host_property[] = "host";
constexpr char tcp_port_property[] = "port";

Config::Config()
    : autopilot_serial(default_autopilot_serial),
      autopilot_serial_speed(autopilot_serial_baud_rate),
      auto_detect_serials(true),
      debug_mode(false),
      isbd_enabled(default_isbd_enabled),
      isbd_serial(default_isbd_serial),
      isbd_serial_speed(isbd_serial_baud_rate),
      isbd_report_period(default_isbd_report_period),
      tcp_enabled(default_tcp_enabled),
      tcp_host(default_tcp_host),
      tcp_port(default_tcp_port),
      tcp_report_period(default_tcp_report_period) {}

int Config::init(const std::string& config_file) {
  INIReader conf(config_file);

  int ret = conf.ParseError();

  if (ret < 0) {
    return ret;
  }

  /* [autopilot] config section */

  set_autopilot_serial(conf.Get(autopilot_config_section,
                                autopilot_serial_property,
                                default_autopilot_serial));

  set_autopilot_serial_speed(conf.GetInteger(autopilot_config_section,
                                             autopilot_serial_speed_property,
                                             autopilot_serial_baud_rate));

  /* [radioroom] config section */

  set_auto_detect_serials(conf.GetBoolean(radioroom_config_section,
                                          auto_detect_serials_property, true));

  /* [isbd] config section */

  set_isbd_enabled(conf.GetBoolean(isbd_config_section, isbd_enabled_property,
                                   default_isbd_enabled));

  set_isbd_serial(
      conf.Get(isbd_config_section, isbd_serial_property, default_isbd_serial));

  set_isbd_serial_speed(conf.GetInteger(
      isbd_config_section, isbd_serial_speed_property, isbd_serial_baud_rate));

  set_isbd_report_period(conf.GetReal(
      isbd_config_section, report_period_property, default_isbd_report_period));

  /* [tcp] config section */

  set_tcp_enabled(conf.GetBoolean(tcp_config_section, tcp_enabled_property,
                                  default_tcp_enabled));

  set_tcp_host(
      conf.Get(tcp_config_section, tcp_host_property, default_tcp_host));

  set_tcp_port(
      conf.GetInteger(tcp_config_section, tcp_port_property, default_tcp_port));

  set_tcp_report_period(conf.GetReal(tcp_config_section, report_period_property,
                                     default_tcp_report_period));
  return 0;
}

bool Config::get_debug_mode() const { return debug_mode; }

void Config::set_debug_mode(bool debug) { debug_mode = debug; }

std::string Config::get_autopilot_serial() const { return autopilot_serial; }

void Config::set_autopilot_serial(const std::string& path) {
  autopilot_serial = path;
}

int Config::get_autopilot_serial_speed() const {
  return autopilot_serial_speed;
}

void Config::set_autopilot_serial_speed(int speed) {
  autopilot_serial_speed = speed;
}

bool Config::get_isbd_enabled() const { return isbd_enabled; }

void Config::set_isbd_enabled(bool enabled) { isbd_enabled = enabled; }

std::string Config::get_isbd_serial() const { return isbd_serial; }

void Config::set_isbd_serial(const std::string& path) { isbd_serial = path; }

int Config::get_isbd_serial_speed() const { return isbd_serial_speed; }

void Config::set_isbd_serial_speed(int speed) { isbd_serial_speed = speed; }

bool Config::get_auto_detect_serials() const { return auto_detect_serials; }

void Config::set_auto_detect_serials(bool a) { auto_detect_serials = a; }

double Config::get_isbd_report_period() const { return isbd_report_period; }

void Config::set_isbd_report_period(double period) {
  isbd_report_period = period;
}

bool Config::get_tcp_enabled() const { return tcp_enabled; }

void Config::set_tcp_enabled(bool enabled) { tcp_enabled = enabled; }

std::string Config::get_tcp_host() const { return tcp_host; }

void Config::set_tcp_host(const std::string& host) { tcp_host = host; }

int Config::get_tcp_port() const { return tcp_port; }

void Config::set_tcp_port(int port) { tcp_port = port; }

double Config::get_tcp_report_period() const { return tcp_report_period; }

void Config::set_tcp_report_period(double period) {
  tcp_report_period = period;
};

}  // namespace radioroom
