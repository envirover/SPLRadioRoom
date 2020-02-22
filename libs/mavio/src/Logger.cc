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
#include <ctime>

#include "timelib.h"

namespace mavio {

std::atomic_int log_mask(LOG_UPTO(LOG_INFO));
std::atomic_bool log_to_stdout(true);

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
  log_to_stdout = false;
#endif
}

void closelog() {
#ifndef __CYGWIN__
  ::closelog();
  log_to_stdout = true;
#endif
}

void vlog(int priority, const char* format, va_list args) {
#ifdef __CYGWIN__
  if (log_mask & priority) {
    char str[64];
    timelib::timestamp(str, sizeof(str));
    printf("%s %s ", str, priority2str(priority));
    vprintf(format, args);
    printf("\n");
  }
#else
  if (log_to_stdout) {
    if (log_mask & priority) {
      char str[64];
      timelib::timestamp(str, sizeof(str));
      printf("%s %s ", str, priority2str(priority));
      vprintf(format, args);
      printf("\n");
    }
  } else {
    ::vsyslog(priority, format, args);
  }
#endif
}

void log(int priority, const char* format, ...) {
  va_list args;
  va_start(args, format);
  mavio::vlog(priority, format, args);
  va_end(args);
}

}  // namespace mavio
