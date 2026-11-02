// Unit tests for include/sim_clock_gate.h: /clock never goes backwards
// inside a sim session, and a new session is never locked out by the
// previous session's high-water mark (#611).

#include <gtest/gtest.h>

#include <cstdint>

#include "sim_clock_gate.h"

using ifssim_bridge::SimClockGate;
using Step = SimClockGate::Step;

namespace {

constexpr int64_t kMs = 1'000'000;
constexpr int64_t kS = 1'000'000'000;

}  // namespace

TEST(SimClockGate, FirstStampPublishesEvenAtZero) {
  SimClockGate gate;
  EXPECT_EQ(gate.update(0), Step::kAdvance);
  EXPECT_EQ(gate.mark_ns(), 0);
}

TEST(SimClockGate, AdvancingStampsPublish) {
  SimClockGate gate;
  for (int64_t t = 0; t < 100; ++t) {
    EXPECT_EQ(gate.update(5 * kS + t * 2'500'000), Step::kAdvance);
  }
}

TEST(SimClockGate, RepeatedStampIsHeld) {
  SimClockGate gate;
  ASSERT_EQ(gate.update(10 * kS), Step::kAdvance);
  EXPECT_EQ(gate.update(10 * kS), Step::kHold);
  EXPECT_EQ(gate.mark_ns(), 10 * kS);
}

TEST(SimClockGate, SmallStepBackIsHeld) {
  SimClockGate gate;
  ASSERT_EQ(gate.update(10 * kS), Step::kAdvance);
  EXPECT_EQ(gate.update(10 * kS - 3 * kMs), Step::kHold);
  EXPECT_EQ(gate.update(10 * kS - SimClockGate::kNewSessionBackstepNs), Step::kHold);
  EXPECT_EQ(gate.mark_ns(), 10 * kS);
}

// The #611 failure: the sim restarts near zero after the old session ran
// for minutes, and the bridge never saw the stream drop.
TEST(SimClockGate, LargeStepBackStartsNewSession) {
  SimClockGate gate;
  ASSERT_EQ(gate.update(300 * kS), Step::kAdvance);
  EXPECT_EQ(gate.update(2 * kS), Step::kNewSession);
  EXPECT_EQ(gate.mark_ns(), 2 * kS);
  // ...and the new session keeps publishing from there.
  EXPECT_EQ(gate.update(2 * kS + 2'500'000), Step::kAdvance);
  EXPECT_EQ(gate.update(3 * kS), Step::kAdvance);
}

TEST(SimClockGate, JustOverThresholdStartsNewSession) {
  SimClockGate gate;
  ASSERT_EQ(gate.update(10 * kS), Step::kAdvance);
  EXPECT_EQ(gate.update(10 * kS - SimClockGate::kNewSessionBackstepNs - 1), Step::kNewSession);
}

// The usual restart: the stream dropped and reconnected, so the first frame
// on the new stream is a new session even if it is only slightly behind.
TEST(SimClockGate, FirstFrameOnNewStreamStartsNewSession) {
  SimClockGate gate;
  ASSERT_EQ(gate.update(500 * kMs), Step::kAdvance);
  EXPECT_EQ(gate.update(100 * kMs, /*first_on_new_stream=*/true), Step::kNewSession);
  EXPECT_EQ(gate.mark_ns(), 100 * kMs);
}

// A reconnect inside the same session (a transient stream glitch) resumes
// ahead of the mark, which is an ordinary advance.
TEST(SimClockGate, ReconnectInSameSessionAdvances) {
  SimClockGate gate;
  ASSERT_EQ(gate.update(60 * kS), Step::kAdvance);
  EXPECT_EQ(gate.update(60 * kS + 40 * kMs, /*first_on_new_stream=*/true), Step::kAdvance);
}

TEST(SimClockGate, ReconnectAtTheMarkIsHeld) {
  SimClockGate gate;
  ASSERT_EQ(gate.update(60 * kS), Step::kAdvance);
  EXPECT_EQ(gate.update(60 * kS, /*first_on_new_stream=*/true), Step::kHold);
}

// Only the first frame after a reconnect gets the benefit of the doubt;
// later small steps back are held as usual.
TEST(SimClockGate, OnlyTheFirstFrameOnNewStreamIsSpecial) {
  SimClockGate gate;
  ASSERT_EQ(gate.update(500 * kMs), Step::kAdvance);
  ASSERT_EQ(gate.update(100 * kMs, /*first_on_new_stream=*/true), Step::kNewSession);
  EXPECT_EQ(gate.update(90 * kMs), Step::kHold);
}

// Without a reconnect, a restart less than 1 s behind the mark is held
// only until the new session passes the old mark.
TEST(SimClockGate, ShortRestartWithoutReconnectCatchesUp) {
  SimClockGate gate;
  ASSERT_EQ(gate.update(800 * kMs), Step::kAdvance);
  EXPECT_EQ(gate.update(10 * kMs), Step::kHold);
  EXPECT_EQ(gate.update(790 * kMs), Step::kHold);
  EXPECT_EQ(gate.update(810 * kMs), Step::kAdvance);
}
