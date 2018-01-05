/*
 Stopwatch.h

 (C) Copyright 2018 Envirover.

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

 Created on: Jan 4, 2018
     Author: Pavel Bobov
*/

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#include <time.h>

/**
 * Stopwatch class is used to measure absolute elapsed time.
 */
class Stopwatch
{
    time_t start_time;
    double interval;

public:

    Stopwatch() : start_time(time(0)), interval(0.0)
    {
    }

    /*
     * Starts count down for the specified number of seconds.
     *
     *
     */
    void start(double i)
    {
        start_time = time(0);
        interval = i;
    }

    /**
     * Returns number of seconds elapsed from the start of the stopwatch.
     */
    double elapsed_time()
    {
        return time(0) - start_time;
    }

    /**
     * Returns true if the time interval specified at the start of the stopwatch is elapsed.
     */
    bool elapsed()
    {
        return elapsed_time() >= interval;
    }
};

#endif /* STOPWATCH_H_*/
