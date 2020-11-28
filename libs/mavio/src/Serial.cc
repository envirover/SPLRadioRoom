/*
 Serial.cc

 MAVIO MAVLink I/O library.

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
#include "Serial.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <istream>

#include "Logger.h"

namespace mavio {

using std::string;
using std::vector;

constexpr suseconds_t serial_read_timeout = 250000;  // microseconds

constexpr char serial_by_id_dir[] = "/dev/serial/by-id/";
constexpr char serial_by_path_dir[] = "/dev/serial/by-path/";
constexpr char standard_serials[] = "";  // "/dev/ttyS0,/dev/ttyAMA0";

speed_t baud_rate_to_speed_t(int baud_rate) {
  switch (baud_rate) {
    case 0:
      return B0;
    case 50:
      return B50;
    case 75:
      return B75;
    case 110:
      return B110;
    case 134:
      return B134;
    case 150:
      return B150;
    case 200:
      return B200;
    case 300:
      return B300;
    case 600:
      return B600;
    case 1200:
      return B1200;
    case 1800:
      return B1800;
    case 2400:
      return B2400;
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    // case 128000:
    //    return B128000;
    case 230400:
      return B230400;
    // case 256000:
    //    return B256000;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
      break;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
  }

  return B57600;
}

Serial::Serial() : tty_fd(-1), path(), last_error(0) {}

Serial::~Serial() {}

int Serial::open(const string& path, int baud_rate) {
  last_error = 0;
  this->path = path;

  tty_fd = ::open(path.data(), O_RDWR | O_NOCTTY /*| O_NONBLOCK*/);

  if (tty_fd < 0) {
    last_error = errno;
    mavio::log(LOG_ERR, "Failed to open file '%s' (errno = %d).", path.data(),
               errno);
    return -1;
  }

  struct termios tio;

  memset(&tio, 0, sizeof(tio));
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_cflag = CS8 | CREAD | CLOCAL;
  tio.c_lflag = 0;
  tio.c_cc[VMIN] = 1;
  tio.c_cc[VTIME] = 5;

  speed_t speed = baud_rate_to_speed_t(baud_rate);
  cfsetospeed(&tio, speed);
  cfsetispeed(&tio, speed);

  tcgetattr(tty_fd, &old_tio);

  tcsetattr(tty_fd, TCSANOW, &tio);

  return 0;
}

int Serial::close() {
  if (tty_fd >= 0) {
    tcsetattr(tty_fd, TCSANOW, &old_tio);
    return ::close(tty_fd);
  }

  return 0;
}

int Serial::read() {
  unsigned char c;

  int ret = read(&c, 1);

  if (ret <= 0) {
    return -1;
  }

  return c;
}

int Serial::read(void* buffer, size_t size) {
  last_error = 0;

  if (tty_fd < 0) {
    return -1;
  }

  fd_set set;
  struct timeval timeout;

  FD_ZERO(&set);        /* clear the set */
  FD_SET(tty_fd, &set); /* add our file descriptor to the set */

  timeout.tv_sec = 0;
  timeout.tv_usec = serial_read_timeout;

  int rv = ::select(tty_fd + 1, &set, NULL, NULL, &timeout);

  if (rv < 0) {
    last_error = errno;
    return -1; /* an error accured */
  }

  if (rv == 0) {
    return 0; /* a timeout occured */
  }

  int ret = ::read(tty_fd, buffer, size);
  last_error = errno;
  return ret;
}

int Serial::write(int c) { return write(&c, 1); }

int Serial::write(const void* buffer, size_t n) {
  last_error = 0;

  if (tty_fd < 0) {
    return -1;
  }

  int ret = ::write(tty_fd, buffer, n);

  if (ret < 0) {
    last_error = errno;
    return ret;
  }

  ret = ::tcflush(tty_fd, TCIOFLUSH);

  if (ret < 0) {
    last_error = errno;
    return ret;
  }

  return ret;
}

int Serial::get_last_error() {
  return last_error;
}

int Serial::get_serial_devices(vector<string>& devices) {
  DIR* dp;
  struct dirent* dirp;

  if ((dp = opendir(serial_by_path_dir)) != NULL) {
    while ((dirp = readdir(dp)) != NULL) {
      if (string(dirp->d_name) == "." || string(dirp->d_name) == "..") continue;

      string device = string(serial_by_path_dir) + string(dirp->d_name);
#ifdef __CYGWIN__
      devices.push_back(string(device.data()));
#else
      char real_path[PATH_MAX];
      realpath(device.data(), real_path);
      devices.push_back(string(real_path));
#endif
    }

    closedir(dp);
  }

  // Add the default list of serial devices

  string s(standard_serials);
  size_t pos = 0;
  while ((pos = s.find(",")) != string::npos) {
    string device = s.substr(0, pos);
    devices.push_back(device);
    s.erase(0, pos + 1);
  }

  devices.push_back(s);

  return devices.size();
}

}  // namespace mavio
