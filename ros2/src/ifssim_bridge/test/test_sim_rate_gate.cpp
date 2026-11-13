// Unit tests for include/sim_rate_gate.h: a wall-timer topic keeps its
// rate in sim time, at real time and in slow motion.

#include <gtest/gtest.h>

#include <cstdint>

#include "sim_rate_gate.h"

using ifssim_bridge::SimRateGate;

namespace {

constexpr int64_t kMs = 1'000'000;

// Ticks a 10 ms wall timer for `wall_ms` while the sim clock runs at
// `speed` of real time in `frame_ms` steps; returns how many publishes pass.
int publishes(double speed, int wall_ms, int64_t frame_ms = 0) {
  SimRateGate gate(10 * kMs);   // 100 Hz in sim time
  int n = 0;
  for (int t = 10; t <= wall_ms; t += 10) {
    int64_t sim_ns = static_cast<int64_t>(t * speed * kMs) + 5'000 * kMs;
    if (frame_ms > 0) sim_ns -= sim_ns % (frame_ms * kMs);   // the clock moves in frame steps
    if (gate.due(sim_ns)) ++n;
  }
  return n;
}

}  // namespace

TEST(SimRateGate, RealTimeKeepsEveryTick) {
  // 10 s of wall time at real time: 100 Hz, as the plain timer was.
  EXPECT_NEAR(publishes(1.0, 10'000), 1000, 2);
}

TEST(SimRateGate, RealTimeWithFrameStepsStillAveragesTheRate) {
  // The sim clock moving in 2.5 ms (400 Hz) and 16.7 ms (60 Hz) steps.
  EXPECT_NEAR(publishes(1.0, 10'000, 3), 1000, 10);
  EXPECT_NEAR(publishes(1.0, 10'000, 17), 1000, 70);
}

TEST(SimRateGate, SlowMotionKeepsTheSimRate) {
  // 10 s of wall time at 0.25x is 2.5 s of sim time: 250 publishes, not 1000.
  EXPECT_NEAR(publishes(0.25, 10'000), 250, 2);
  EXPECT_NEAR(publishes(0.1, 10'000), 100, 2);
}

TEST(SimRateGate, NoSimClockYetPublishesAsBefore) {
  SimRateGate gate(10 * kMs);
  EXPECT_TRUE(gate.due(0));
  EXPECT_TRUE(gate.due(0));
}

TEST(SimRateGate, AClockThatGoesBackStartsOver) {
  SimRateGate gate(10 * kMs);
  EXPECT_TRUE(gate.due(100'000 * kMs));
  EXPECT_FALSE(gate.due(100'005 * kMs));
  // A new sim session restarts the game clock near zero: publish at once.
  EXPECT_TRUE(gate.due(20 * kMs));
  EXPECT_FALSE(gate.due(25 * kMs));
  EXPECT_TRUE(gate.due(30 * kMs));
}

TEST(SimRateGate, AStallDoesNotBurst) {
  SimRateGate gate(10 * kMs);
  EXPECT_TRUE(gate.due(1'000 * kMs));
  // The sim clock jumps 2 s (a pause, a load): one publish, then the rate again.
  EXPECT_TRUE(gate.due(3'000 * kMs));
  EXPECT_FALSE(gate.due(3'001 * kMs));
  EXPECT_TRUE(gate.due(3'010 * kMs));
}
