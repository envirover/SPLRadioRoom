/*
 TimeLibTest.cc

 TimeLib library test.

 Copyright (C) 2019 Envirover

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

#include <iostream>

#include "timelib.h"

using std::cout;
using std::endl;

using timelib::sec2ms;
using timelib::sleep;
using timelib::Stopwatch;
using timelib::time_since_epoch;
using timelib::timestamp;

int main() {
  cout << "TimeLib test started." << endl;
  int64_t start_time = time_since_epoch();
  cout << "Time since epoch = " << start_time << " ms" << endl;

  int64_t interval = sec2ms(1.0);

  while (interval > 0) {
    Stopwatch timer;
    sleep(interval);
    int64_t elapsed_time = timer.elapsed_time();
    char str[32];
    timestamp(str, sizeof(str));
    cout << str << " ";
    cout << "sleep time = " << static_cast<int>(interval) << " ms, "
         << "elapsed time = " << static_cast<int>(elapsed_time) << " ms, "
         << "difference = " << static_cast<int>(elapsed_time - interval)
         << " ms" << endl;
    interval /= 2;
  }

  int64_t end_time = time_since_epoch();
  cout << "Total test time = " << static_cast<int>(end_time - start_time)
       << " ms" << endl;
  cout << "TimeLib test completed." << endl;

  return 0;
}
