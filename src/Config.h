/*
 Config.h

 This file is a part of RadioRoom project.

 (C) Copyright 2017-2019 Envirover.

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
#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_

#include <string>

namespace radioroom {

constexpr char default_config_file[] = "/etc/radioroom.conf";

/**
 * RadioRoom configuration properties.
 */
class Config {
 public:
  Config();

  /*
   * Loads configuration from the specified config file.
   *
   * Returns 0 in case of success and exit code in case of invalid
   * configuration.
   */
  int init(const std::string& config_file);

  /* Autopilot configuration properties */

  std::string get_autopilot_serial() const;
  void set_autopilot_serial(const std::string& path);

  int get_autopilot_serial_speed() const;
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

  int get_isbd_serial_speed() const;
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

  std::string get_on_armed() const;
  void set_on_armed(const std::string cmd);

  std::string get_on_disarmed() const;
  void set_on_disarmed(const std::string cmd);

  std::string get_on_video_start_capture() const;
  void set_on_video_start_capture(const std::string cmd);

  std::string get_on_video_stop_capture() const;
  void set_on_video_stop_capture(const std::string cmd);

  std::string get_on_image_start_capture() const;
  void set_on_image_start_capture(const std::string cmd);

  std::string get_on_image_stop_capture() const;
  void set_on_image_stop_capture(const std::string cmd);

  std::string get_on_do_digicam_control() const;
  void set_on_do_digicam_control(const std::string cmd);

 private:
  std::string autopilot_serial;
  int autopilot_serial_speed;

  bool auto_detect_serials;
  bool debug_mode;

  bool isbd_enabled;
  std::string isbd_serial;
  int isbd_serial_speed;
  double isbd_report_period;

  bool tcp_enabled;
  std::string tcp_host;
  int tcp_port;
  double tcp_report_period;

  std::string on_armed;
  std::string on_disarmed;
  std::string on_video_start_capture;
  std::string on_video_stop_capture;
  std::string on_image_start_capture;
  std::string on_image_stop_capture;
  std::string on_do_digicam_control;
};

extern Config config;

}  // namespace radioroom

#endif  // SRC_CONFIG_H_
