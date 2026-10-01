// StampGuard — keeps one topic's header stamps strictly increasing.
//
// GLIM (and any LiDAR-IMU SLAM pipeline) rejects a sample whose stamp is at or
// before the previous one. The bridge republishes each sim tick's stamp several
// times on /imu (a 400 Hz stream over a 60 Hz tick), and LiDAR stamps can
// arrive slightly out of order, so the guard moves such a stamp 1 ns past the
// last one it let through.
//
// Like the /clock mark (sim_clock_gate.h, #611), the last stamp belongs to one
// sim session. A restarted sim starts its clock near zero, so without the
// session rule below every later stamp was pinned just past the previous
// session's last one and crept forward 1 ns per message, while /clock had moved
// on to the new session. A stamp is taken as a new session, and passes
// unchanged, when it is more than SimClockGate::kNewSessionBackstepNs behind
// the last one, or when the caller says so (the first frame on a reconnected
// sensor stream).
//
// Header-only so test/test_stamp_guard.cpp can exercise it without any ROS
// plumbing.

#ifndef IFSSIM_BRIDGE__STAMP_GUARD_H_
#define IFSSIM_BRIDGE__STAMP_GUARD_H_

#include <cstdint>

#include "sim_clock_gate.h"

namespace ifssim_bridge {

class StampGuard {
 public:
  // The stamp to publish for a message captured at `stamp_ns`.
  int64_t next(int64_t stamp_ns, bool new_session = false) {
    const bool behind = has_last_ && stamp_ns <= last_ns_;
    if (behind && !new_session &&
        last_ns_ - stamp_ns <= SimClockGate::kNewSessionBackstepNs) {
      stamp_ns = last_ns_ + 1;
    }
    has_last_ = true;
    last_ns_ = stamp_ns;
    return stamp_ns;
  }

 private:
  bool has_last_ = false;
  int64_t last_ns_ = 0;
};

}  // namespace ifssim_bridge

#endif  // IFSSIM_BRIDGE__STAMP_GUARD_H_
