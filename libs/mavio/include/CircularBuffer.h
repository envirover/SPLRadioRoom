/*
 CircularBuffer.h

MAVIO MAVLink I/O library.

Copyright (C) 2019   Envirover

All rights reserved.

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

#ifndef LIBS_MAVIO_INCLUDE_CIRCULARBUFFER_H_
#define LIBS_MAVIO_INCLUDE_CIRCULARBUFFER_H_

#include <memory>
#include <mutex>
#include <chrono>

#include "timelib.h"

namespace mavio {

/**
 * Thread-safe fixed-size queue.
 */
template <class T>
class CircularBuffer {
 public:
  explicit CircularBuffer(size_t size)
      : buf_(std::unique_ptr<T[]>(new T[size])),
        max_size_(size),
        last_push_time_(timelib::time_since_epoch()) {}

  void push(T item) {
    std::chrono::milliseconds push_time = timelib::time_since_epoch();

    std::lock_guard<std::mutex> lock(mutex_);

    buf_[head_] = item;

    if (full_) {
      tail_ = (tail_ + 1) % max_size_;
    }

    head_ = (head_ + 1) % max_size_;

    full_ = head_ == tail_;

    last_push_time_ = push_time;
  }

  bool pop(T& item) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (empty()) {
      return false;
    }

    // Read data and advance the tail (we now have a free space)
    item = buf_[tail_];
    full_ = false;
    tail_ = (tail_ + 1) % max_size_;

    return true;
  }

  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    head_ = tail_;
    full_ = false;
  }

  bool empty() const {
    // if head and tail are equal, we are empty
    return (!full_ && (head_ == tail_));
  }

  bool full() const {
    // If tail is ahead the head by 1, we are full
    return full_;
  }

  size_t capacity() const { return max_size_; }

  size_t size() const {
    size_t size = max_size_;

    if (!full_) {
      if (head_ >= tail_) {
        size = head_ - tail_;
      } else {
        size = max_size_ + head_ - tail_;
      }
    }

    return size;
  }

  std::chrono::milliseconds last_push_time() {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_push_time_;
  }

 private:
  std::mutex mutex_;
  std::unique_ptr<T[]> buf_;
  size_t head_ = 0;
  size_t tail_ = 0;
  const size_t max_size_;
  bool full_ = 0;

  std::chrono::milliseconds last_push_time_;
};

}  // namespace mavio

#endif // LIBS_MAVIO_INCLUDE_CIRCULARBUFFER_H_
