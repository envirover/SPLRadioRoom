/*
 Config.h

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
#include <string>

#define DEFAULT_CONFIG_FILE         "/etc/radioroom.conf"

#define DEFAULT_AUTOPILOT_SERIAL    "/dev/ttyUSB0"
#define AUTOPILOT_SERIAL_BAUD_RATE  57600

#define DEFAULT_ISBD_ENABLED        true
#define DEFAULT_ISBD_SERIAL         "/dev/ttyUSB1"
#define ISBD_SERIAL_BAUD_RATE       19200

#define DEFAULT_TCP_ENABLED         false
#define DEFAULT_TCP_HOST            ""
#define DEFAULT_TCP_PORT            5060

#define DEFAULT_ISBD_REPORT_PERIOD  300.0 // 5 minutes
#define DEFAULT_TCP_REPORT_PERIOD   60.0 // 1 minute

// radioroom.conf properties
#define AUTOPILOT_CONFIG_SECTION        "autopilot"
#define AUTOPILOT_SERIAL_PROPERTY       "serial"
#define AUTOPILOT_SERIAL_SPEED_PROPERTY "serial_speed"

#define RADIOROOM_CONFIG_SECTION        "radioroom"
#define AUTO_DETECT_SERIALS_PROPERTY    "auto_detect_serials"
#define REPORT_PERIOD_PROPERTY          "report_period"

#define ISBD_CONFIG_SECTION             "isbd"
#define ISBD_ENABLED_PROPERTY           "enabled"
#define ISBD_SERIAL_PROPERTY            "serial"
#define ISBD_SERIAL_SPEED_PROPERTY      "serial_speed"

#define TCP_CONFIG_SECTION              "tcp"
#define TCP_ENABLED_PROPERTY            "enabled"
#define TCP_HOST_PROPERTY               "host"
#define TCP_PORT_PROPERTY               "port"

/**
 * RadioRoom configuration properties.
 */
class Config {

    std::string   autopilot_serial;
    int           autopilot_serial_speed;

    bool          auto_detect_serials;
    bool          debug_mode;

    bool          isbd_enabled;
    std::string   isbd_serial;
    int           isbd_serial_speed;
    unsigned long isbd_report_period;

    bool          tcp_enabled;
    std::string   tcp_host;
    int           tcp_port;
    unsigned long tcp_report_period;

public:
    Config();

    /*
     * Loads configuration from the specified config file.
     *
     * Returns 0 in case of success and exit code in case of invalid configuration.
     */
    int init(const std::string& config_file);

    /* Autopilot configuration properties */

    std::string get_autopilot_serial() const;
    void set_autopilot_serial(const std::string& path);

    int  get_autopilot_serial_speed() const;
    void set_autopilot_serial_speed(int speed);

    /* RadioRoom configuration properties */

    bool get_auto_detect_serials() const;
    void set_auto_detect_serials(bool a);

    bool get_debug_mode() const;
    void set_debug_mode(bool debug);

    /* ISBD comm link configuration properties */

    bool get_isbd_enabled() const;
    void set_isbd_enabled(bool enabled);

    std::string get_isbd_serial() const;
    void set_isbd_serial(const std::string& path);

    int  get_isbd_serial_speed() const;
    void set_isbd_serial_speed(int speed);

    double get_isbd_report_period() const;
    void set_isbd_report_period(double period);

    /* TCP/IP comm link configuration properties */

    bool get_tcp_enabled() const;
    void set_tcp_enabled(bool enabled);

    std::string get_tcp_host() const;
    void set_tcp_host(const std::string& host);

    int get_tcp_port() const;
    void set_tcp_port(int port);

    double get_tcp_report_period() const;
    void set_tcp_report_period(double period);
};

extern Config config;
