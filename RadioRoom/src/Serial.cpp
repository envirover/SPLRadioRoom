/*
 Serial.cpp

 This file is a part of SPLRadioRoom project.

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

 Created on: Oct 1, 2017
     Author: Pavel Bobov
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <fstream>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <syslog.h>
#include <errno.h>
#include <istream>
#include <dirent.h>

#include "Serial.h"

Serial::Serial() : tty_fd(0), path()
{
}

Serial::~Serial()
{
}

int Serial::open(const string& path, speed_t speed)
{
    this->path = path;

    tty_fd = ::open(path.data(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (tty_fd < 0) {
        syslog(LOG_ERR, "Failed to open file '%s' (errno = %d).", path.data(), errno);
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

    cfsetospeed(&tio, speed);
    cfsetispeed(&tio, speed);

    tcgetattr (tty_fd, &old_tio);

    tcsetattr(tty_fd, TCSANOW, &tio);

    return 0;
}

int Serial::close()
{
    tcsetattr(tty_fd, TCSANOW, &old_tio);
    return ::close(tty_fd);
}

int Serial::read()
{
    unsigned char c;

    int ret = read(&c, 1);

    if (ret <= 0) {
        return -1;
    }

    return c;
}

int Serial::read(void* buffer, size_t size)
{
    return ::read(tty_fd, buffer, size);
}

int Serial::write(int c)
{
    return write(&c, 1);
}

int Serial::write(const void* buffer, size_t n)
{
    int ret = ::write(tty_fd, buffer, n);

    if (ret < 0)
        return ret;

    if (::tcflush(tty_fd, TCIOFLUSH) < 0)
        return -1;

    return ret;
}

int Serial::get_serial_devices(vector<string>& devices) {
    DIR *dp;
    struct dirent *dirp;

    if ((dp = opendir(SERIAL_BY_ID_DIR)) == NULL) {
        syslog(LOG_ERR, "Failed to open '%s'.", SERIAL_BY_ID_DIR);
        //Use the default list of serial devices
        string s(STANDARD_SERIALS);
        size_t pos = 0;
        while ((pos = s.find(",")) != string::npos) {
            string device = s.substr(0, pos);
            devices.push_back(device);
            s.erase(0, pos + 1);
        }

        devices.push_back(s);
    } else {
        while ((dirp = readdir(dp)) != NULL) {
            if (string(dirp->d_name) == "." || string(dirp->d_name) == "..")
                continue;

            string device = string(SERIAL_BY_ID_DIR) + string(dirp->d_name);
            char real_path[PATH_MAX];
            realpath(device.data(), real_path);
            devices.push_back(string(real_path));
        }

        closedir(dp);
    }

    return devices.size();
}
