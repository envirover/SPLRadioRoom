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

#include "Serial.h"

Serial::Serial() : tty_fd(0) {
}

Serial::~Serial() {
}

int Serial::open(const char * path, speed_t speed) {
    tty_fd = ::open(path, O_RDWR | O_NONBLOCK);

    if (tty_fd <= 0)
        return -1;

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

    tcsetattr(tty_fd, TCSANOW, &tio);

    return 0;
}

int Serial::close() {
    return ::close(tty_fd);
}

int Serial::read() {
    unsigned char c;

    ssize_t ret = ::read(tty_fd, &c, 1);

    if (ret <= 0)
        return -1;

    return c;
}

int Serial::read(void* buffer, size_t size) {
    return ::read(tty_fd, buffer, size);
}

int Serial::write(const void* buffer, size_t n) {
     return ::write(tty_fd, buffer, n);
}
