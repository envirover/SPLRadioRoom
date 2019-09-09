/*
 CircularBuffer.h
 
 Derived from:
 https://github.com/embeddedartistry/embedded-resources/blob/master/examples/cpp/circular_buffer.cpp
 */

#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include <chrono>
#include <memory>
#include <mutex>

namespace mavio {

/**
 * Thread-safe fixed-size queue.
 */
template <class T>
class CircularBuffer {
public:
    explicit CircularBuffer(size_t size) : buf_(std::unique_ptr<T[]>(new T[size])),
                                           max_size_(size),
                                           last_push_time_(std::chrono::high_resolution_clock::now())
    {
    }

    void push(T item)
    {
        std::chrono::high_resolution_clock::time_point push_time = std::chrono::high_resolution_clock::now();

        std::lock_guard<std::mutex> lock(mutex_);

        buf_[head_] = item;

        if (full_) {
            tail_ = (tail_ + 1) % max_size_;
        }

        head_ = (head_ + 1) % max_size_;

        full_ = head_ == tail_;

        last_push_time_ = push_time;
    }

    bool pop(T& item)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (empty()) {
            return false;
        }

        //Read data and advance the tail (we now have a free space)
        item  = buf_[tail_];
        full_ = false;
        tail_ = (tail_ + 1) % max_size_;

        return true;
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        head_ = tail_;
        full_ = false;
    }

    bool empty() const
    {
        //if head and tail are equal, we are empty
        return (!full_ && (head_ == tail_));
    }

    bool full() const
    {
        //If tail is ahead the head by 1, we are full
        return full_;
    }

    size_t capacity() const
    {
        return max_size_;
    }

    size_t size() const
    {
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

    std::chrono::high_resolution_clock::time_point last_push_time()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_push_time_;
    }

private:
    std::mutex           mutex_;
    std::unique_ptr<T[]> buf_;
    size_t               head_ = 0;
    size_t               tail_ = 0;
    const size_t         max_size_;
    bool                 full_ = 0;

    std::chrono::high_resolution_clock::time_point last_push_time_;
};

} // namespace mavio

#endif /* CIRCULARBUFFER_H_ */
