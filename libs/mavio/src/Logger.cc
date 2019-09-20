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

#include "Logger.h"

#include <stdarg.h>
#include <stdio.h>
#include <syslog.h>

#include <atomic>
#include <chrono>
#include <ctime>

namespace mavio {

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;
using std::chrono::time_point;

std::atomic_int log_mask(LOG_UPTO(LOG_INFO));

const char* priority2str(int priority) {
  switch (priority) {
    case LOG_ALERT:
      return "ALERT";
    case LOG_CRIT:
      return "CRIT";
    case LOG_DEBUG:
      return "DEBUG";
    case LOG_EMERG:
      return "EMERG";
    case LOG_ERR:
      return "ERR";
    case LOG_INFO:
      return "INFO";
    case LOG_NOTICE:
      return "NOTICE";
    case LOG_WARNING:
      return "WARNING";
    default:
      return "UNDEFINED";
  }
}

void openlog(const char* identity, int mask) {
#ifdef __CYGWIN__
  printf("Opened log '%s'", identity);
  log_mask = mask;
#else
  ::openlog(identity, LOG_CONS | LOG_NDELAY, LOG_USER);
  ::setlogmask(mask);
#endif
}

void closelog() {
#ifndef __CYGWIN__
  ::closelog();
#endif
}

void vlog(int priority, const char* format, va_list args) {
#ifdef __CYGWIN__
  if (log_mask & priority) {
    time_point<system_clock> now = system_clock::now();
    int64_t millis =
        duration_cast<milliseconds>(now.time_since_epoch()).count();
    std::time_t time_now_t = system_clock::to_time_t(now);
    std::tm* t = std::localtime(&time_now_t);

    printf("%04d-%02d-%02d %02d:%02d:%02d:%03d ", t->tm_year, t->tm_mon,
           t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, (int)(millis % 1000));
    printf("%s ", priority2str(priority));
    vprintf(format, args);
    printf("\n");
  }
#else
  ::vsyslog(priority, format, args);
#endif
}

void log(int priority, const char* format, ...) {
  va_list args;
  va_start(args, format);
  mavio::vlog(priority, format, args);
  va_end(args);
}

}  // namespace mavio