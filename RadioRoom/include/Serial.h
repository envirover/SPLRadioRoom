/*
 Serial.h

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

#ifndef SERIAL_H_
#define SERIAL_H_

#include <termios.h>

/**
 * Provides access to serial devices.
 */
class Serial
{
    int tty_fd;

public:

    /**
     * Default constructor.
     */
    Serial();

    /**
     * Destructor.
     */
    virtual ~Serial();

    /**
     * Opens serial device with the specified path and sets the baud rate.
     *
     * Returns 0 in case of success or -1 in case of failure.
     */
    int open(const char* path, speed_t speed);

    /**
     * Closes the serial device.
     */
    int close();

    /*
     * Reads single byte from the serial device.
     *
     * Returns the character read, or -1 if none is available.
     */
    int read();

    /**
     * Reads up to size bytes from the serial device, storing the results in the buffer.
     *
     * Returns the number of bytes read or -1 in case of error.
     */
    int read(void* buffer, size_t size);

    /**
     * Writes single character to the serial device.
     *
     * Returns 1 in case of success or -1 in case of error.
     */
    int write(int c);

    /**
     * Writes the specified number of bytes from the specified buffer to the serial device.
     *
     * Returns the number of bytes written or -1 in case of error.
     */
    int write(const void* buffer, size_t n);
};

#endif /* SERIAL_H_ */
