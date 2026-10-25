// SimClockGate — decides which sim stamps the bridge publishes on /clock.
//
// /clock must never go backwards inside a sim session, and the 400 Hz
// sensor stream can repeat a sim ns across consecutive frames, so the
// bridge keeps a high-water mark and drops anything at or below it.
//
// The mark must not outlive the session it came from (#611). A new sim
// session (PIE stopped and replayed, or the editor swapped for the
// packaged binary) restarts the game clock near zero, so every stamp lands
// below the old mark. Before this gate, /clock then stopped publishing for
// the life of the container, and every use_sim_time node in the pipeline
// froze with no error: sensors kept streaming at full rate, lifecycle nodes
// still reported `active`, and timers simply never fired.
//
// A stamp therefore starts a new session, and is published, when either:
//   * it is the first frame on a sensor stream the bridge has just
//     reconnected (the usual way a sim restart shows up), or
//   * it is more than kNewSessionBackstepNs behind the mark (a restart the
//     bridge never saw as a disconnect).
// The threshold only has to clear same-ns repeats and sub-frame jitter
// (2.5 ms between frames); 1 s is far above that and far below any real
// gap between sessions.
//
// Header-only so test/test_sim_clock_gate.cpp can exercise it without any
// ROS plumbing.

#ifndef IFSSIM_BRIDGE__SIM_CLOCK_GATE_H_
#define IFSSIM_BRIDGE__SIM_CLOCK_GATE_H_

#include <cstdint>

namespace ifssim_bridge {

class SimClockGate {
 public:
  static constexpr int64_t kNewSessionBackstepNs = 1'000'000'000;  // 1 s

  enum class Step {
    kAdvance,     // later than the mark: publish it
    kHold,        // a repeat or a small step back: drop it
    kNewSession,  // behind the mark, but a new sim session: publish it
  };

  // `first_on_new_stream` is true for the first frame after the sensor
  // stream reconnected.
  Step update(int64_t stamp_ns, bool first_on_new_stream = false) {
    Step step;
    if (!has_mark_ || stamp_ns > mark_ns_) {
      step = Step::kAdvance;
    } else if ((first_on_new_stream && stamp_ns < mark_ns_) ||
               mark_ns_ - stamp_ns > kNewSessionBackstepNs) {
      step = Step::kNewSession;
    } else {
      return Step::kHold;
    }
    has_mark_ = true;
    mark_ns_ = stamp_ns;
    return step;
  }

  // The last stamp published, or 0 before the first one.
  int64_t mark_ns() const { return mark_ns_; }

 private:
  bool has_mark_ = false;
  int64_t mark_ns_ = 0;
};

}  // namespace ifssim_bridge

#endif  // IFSSIM_BRIDGE__SIM_CLOCK_GATE_H_
