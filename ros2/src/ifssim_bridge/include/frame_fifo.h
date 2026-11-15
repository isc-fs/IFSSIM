// Bounded FIFO between the sensor stream's receive and publish threads.
//
// Every sensor frame carries one IMU sample on the 400 Hz grid (#875), so a
// frame dropped here is a sample missing from /imu. The receive thread must
// never block (see ifssim_ros_wrapper.h), so when the publisher falls behind
// by more than `capacity` frames the oldest are dropped and counted.
// ROS-free so it can be unit-tested.
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace ifssim_bridge {

template <typename T>
class FrameFifo {
public:
    explicit FrameFifo(size_t capacity) : capacity_(capacity > 0 ? capacity : 1) {}

    // Appends; drops the oldest frame when full. Returns true if one was dropped.
    bool push(const T& v)
    {
        bool dropped = false;
        if (q_.size() >= capacity_) {
            q_.pop_front();
            ++dropped_;
            dropped = true;
        }
        q_.push_back(v);
        return dropped;
    }

    // Moves everything queued, oldest first, into out (cleared first).
    void drain(std::vector<T>& out)
    {
        out.assign(q_.begin(), q_.end());
        q_.clear();
    }

    bool empty() const { return q_.empty(); }
    size_t size() const { return q_.size(); }
    uint64_t dropped() const { return dropped_; }

private:
    size_t capacity_;
    std::deque<T> q_;
    uint64_t dropped_ = 0;
};

}  // namespace ifssim_bridge
