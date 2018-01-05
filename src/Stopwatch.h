/*
 * Envirover confidential
 *
 *  [2017] Envirover
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property of
 * Envirover and its suppliers, if any.  The intellectual and technical concepts
 * contained herein are proprietary to Envirover and its suppliers and may be
 * covered by U.S. and Foreign Patents, patents in process, and are protected
 * by trade secret or copyright law.
 *
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Envirover.
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
