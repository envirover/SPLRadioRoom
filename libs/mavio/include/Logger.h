/*
 Logger.h
 
 MAVIO MAVLink I/O library.
 
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

#ifndef LIBS_MAVIO_INCLUDE_LOGGER_H_
#define LIBS_MAVIO_INCLUDE_LOGGER_H_

// Log priorities
#define LOG_EMERG 0
#define LOG_ALERT 1
#define LOG_CRIT 2
#define LOG_ERR 3
#define LOG_WARNING 4
#define LOG_NOTICE 5
#define LOG_INFO 6
#define LOG_DEBUG 7

// Log mask up to priority
#define LOG_UPTO(pri) ((1 << ((pri) + 1)) - 1)

namespace mavio {

// Initializes global logger.
// Mask determines which future log() calls shall be ignored.
void openlog(const char* identity, int mask);

// Closes logger
void closelog();

// Logs message with given priority
// Interface matches ::syslog() method.
void log(int priority, const char* format, ...);

}  // namespace mavio

#endif  // LIBS_MAVIO_INCLUDE_LOGGER_H_
