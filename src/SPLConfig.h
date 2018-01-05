/*
 SPLConfig.h

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
#include <string>

#define DEFAULT_CONFIG_FILE         "/etc/radioroom.conf"

#define DEFAULT_AUTOPILOT_SERIAL    "/dev/ttyUSB0"
#define DEFAULT_ISBD_SERIAL         "/dev/ttyUSB1"

#define AUTOPILOT_SERIAL_BAUD_RATE  57600
#define ISBD_SERIAL_BAUD_RATE       19200

#define DEFAULT_REPORT_PERIOD  300.0 // 5 minutes

// radioroom.conf peoperties
#define AUTOPILOT_CONFIG_SECTION        "autopilot"
#define AUTOPILOT_SERIAL_PROPERTY       "serial"
#define AUTOPILOT_SERIAL_SPEED_PROPERTY "serial_speed"

#define ISBD_CONFIG_SECTION             "isbd"
#define ISBD_SERIAL_PROPERTY            "serial"
#define ISBD_SERIAL_SPEED_PROPERTY      "serial_speed"

#define RADIOROOM_CONFIG_SECTION        "radioroom"
#define AUTO_DETECT_SERIALS_PROPERTY    "auto_detect_serials"
#define REPORT_PERIOD_PROPERTY          "report_period"

/**
 * SPL configuration.
 */
class SPLConfig {

    std::string   autopilot_serial;
    std::string   isbd_serial;
    int           autopilot_serial_speed;
    int           isbd_serial_speed;
    bool          auto_detect_serials;
    unsigned long report_period;
    bool          debug_mode;

public:
    SPLConfig();

    /*
     * Loads configuration from the specified config file.
     *
     * Returns 0 in case of success and exit code in case of invalid configuration.
     */
    int init(const std::string& config_file);

    std::string get_autopilot_serial() const;
    void set_autopilot_serial(const std::string& path);

    int  get_autopilot_serial_speed() const;
    void set_autopilot_serial_speed(int speed);

    std::string get_isbd_serial() const;
    void set_isbd_serial(const std::string& path);

    int  get_isbd_serial_speed() const;
    void set_isbd_serial_speed(int speed);

    bool get_auto_detect_serials() const;
    void set_auto_detect_serials(bool a);

    double get_report_period() const;
    // Report period could be changed at runtime by setting HL_REPORT_PERIOD parameter
    void set_report_period(double period);

    bool get_debug_mode() const;
    void set_debug_mode(bool debug);
};

extern SPLConfig config;
