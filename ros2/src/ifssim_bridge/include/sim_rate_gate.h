// SimRateGate — keeps a timer-driven topic at its rate in sim time.
//
// A few topics are published from wall timers rather than from the sim's
// frames: the LWS steering sensor (/lws/steering_wheel_angle_rad and
// /steering_angle, 100 Hz) and the tyre loads (20 Hz). At real time a wall
// second is a sim second, so that is the rate the pipeline sees. In slow motion (the sim's Timing.SpeedFactor below 1,
// docs/sim_timing.md) a wall second is a fraction of a sim second, and the
// same timers would publish 4x (at 0.25x) more messages per sim second, the
// same values repeated at the same sim time.
//
// The timer still fires on the wall clock; the gate lets a publish through
// once per period of sim time, on average. At real time that is every tick,
// as before (the sim clock moves in its own frame steps, so a tick may be
// skipped and the next one catch up). In slow motion, it is one tick in four.
//
// Header-only so test/test_sim_rate_gate.cpp can exercise it without ROS.

#ifndef IFSSIM_BRIDGE__SIM_RATE_GATE_H_
#define IFSSIM_BRIDGE__SIM_RATE_GATE_H_

#include <cstdint>

namespace ifssim_bridge {

class SimRateGate {
 public:
  // A gap this many periods behind is a stall or a jump: start over rather
  // than burst to catch up.
  static constexpr int64_t kMaxBacklogPeriods = 5;

  explicit SimRateGate(int64_t period_ns) : period_ns_(period_ns) {}

  // Whether a publish is due at sim time `now_ns`. Before the sim clock
  // exists (now_ns <= 0) everything is due, as the plain timer was.
  bool due(int64_t now_ns) {
    if (now_ns <= 0 || period_ns_ <= 0) return true;
    // First call, the clock went back (a new sim session), or far behind.
    if (!has_next_ || now_ns < next_ns_ - 2 * period_ns_ ||
        now_ns > next_ns_ + kMaxBacklogPeriods * period_ns_) {
      has_next_ = true;
      next_ns_ = now_ns + period_ns_;
      return true;
    }
    if (now_ns < next_ns_) return false;
    next_ns_ += period_ns_;
    return true;
  }

 private:
  int64_t period_ns_;
  int64_t next_ns_ = 0;
  bool has_next_ = false;
};

}  // namespace ifssim_bridge

#endif  // IFSSIM_BRIDGE__SIM_RATE_GATE_H_
