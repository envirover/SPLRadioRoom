/*
 timelib.h

 TimeLib library provides operations for getting current time, measuring
 duration, getting timestamps, and sleeping.

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

#ifndef LIBS_TIMELIB_INCLUDE_TIMELIB_H_
#define LIBS_TIMELIB_INCLUDE_TIMELIB_H_

#include <chrono>

namespace timelib {

// Stopwatch class is used to measure elapsed time.
class Stopwatch {
 public:
  Stopwatch() : start_time(std::chrono::high_resolution_clock::now()) {}

  // Resets stopwatch.
  void reset() { reset(std::chrono::high_resolution_clock::now()); }

  // Set start time.
  void reset(std::chrono::high_resolution_clock::time_point t) {
    start_time = t;
  }

  // Returns number of milliseconds elapsed from the start of the stopwatch.
  std::chrono::milliseconds elapsed_time() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start_time);
  }

 private:
  std::chrono::high_resolution_clock::time_point start_time;
};

// Returns number of milliseconds since epoch from the system clock.
inline std::chrono::milliseconds time_since_epoch() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch());
}

// Sleeps for the specified time in milliseconds.
inline void sleep(const std::chrono::milliseconds& ms) {
  struct timespec t;
  t.tv_sec = ms.count() / 1000;
  t.tv_nsec = (ms.count() % 1000) * 1000000L;
  ::nanosleep(&t, nullptr);
}

// Writes current timestamp into the specified char buffer.
inline void timestamp(char* str, size_t n) {
  std::chrono::time_point<std::chrono::high_resolution_clock> now =
      std::chrono::high_resolution_clock::now();
  int64_t millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now.time_since_epoch())
                       .count();
  std::time_t time_now_t = std::chrono::high_resolution_clock::to_time_t(now);
  std::tm* t = std::localtime(&time_now_t);

  snprintf(str, n, "%04d-%02d-%02d %02d:%02d:%02d.%03d", t->tm_year + 1900,
           t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec,
           static_cast<int>(millis % 1000));
}

// Convert seconds to milliseconds
inline constexpr std::chrono::milliseconds sec2ms(double seconds) {
  return std::chrono::milliseconds(static_cast<int64_t>(seconds * 1000));
}

}  // namespace timelib

#endif  // LIBS_TIMELIB_INCLUDE_TIMELIB_H_
