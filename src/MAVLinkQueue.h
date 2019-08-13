/*
 MAVLinkQueue.h

Telemetry for MAVLink autopilots.

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

  Created on: Aug 11, 2019
      Author: pbobo
 */

#ifndef MAVLINK_QUEUE_
#define MAVLINK_QUEUE_

#include "MAVLinkLib.h"
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

class MAVLinkQueue
{
public:
    mavlink_message_t pop()
    {
        std::unique_lock<std::mutex> mlock(mutex_);

        while (queue_.empty())
        {
            cond_.wait(mlock);
        }

        mavlink_message_t val = queue_.front();
        queue_.pop();

        return val;
    }

    void pop(mavlink_message_t &item)
    {
        std::unique_lock<std::mutex> mlock(mutex_);

        while (queue_.empty())
        {
            cond_.wait(mlock);
        }

        item = queue_.front();
        queue_.pop();
    }

    void push(const mavlink_message_t &item)
    {
        std::unique_lock<std::mutex> mlock(mutex_);
        queue_.push(item);
        mlock.unlock();
        cond_.notify_one();
    }

    MAVLinkQueue() = default;
    MAVLinkQueue(const MAVLinkQueue &) = delete;            // disable copying
    MAVLinkQueue &operator=(const MAVLinkQueue &) = delete; // disable assignment

private:
    std::queue<mavlink_message_t> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
};

#endif /* MAVLINK_QUEUE_ */
