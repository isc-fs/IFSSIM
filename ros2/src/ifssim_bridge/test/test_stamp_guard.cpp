// Unit tests for include/stamp_guard.h: /imu and /lidar_points stamps stay
// strictly increasing inside a sim session, and follow a new session instead
// of staying pinned to the previous one's last stamp.

#include <gtest/gtest.h>

#include <cstdint>

#include "stamp_guard.h"

using ifssim_bridge::SimClockGate;
using ifssim_bridge::StampGuard;

namespace {

constexpr int64_t kMs = 1'000'000;
constexpr int64_t kS = 1'000'000'000;

}  // namespace

TEST(StampGuard, FirstStampPassesEvenAtZero) {
  StampGuard guard;
  EXPECT_EQ(guard.next(0), 0);
}

TEST(StampGuard, AdvancingStampsPassUnchanged) {
  StampGuard guard;
  for (int64_t t = 0; t < 10; ++t) {
    EXPECT_EQ(guard.next(5 * kS + t * 16 * kMs), 5 * kS + t * 16 * kMs);
  }
}

// /imu: the same tick's stamp arrives several times.
TEST(StampGuard, RepeatsAreBumpedOneNanosecondEach) {
  StampGuard guard;
  EXPECT_EQ(guard.next(10 * kS), 10 * kS);
  EXPECT_EQ(guard.next(10 * kS), 10 * kS + 1);
  EXPECT_EQ(guard.next(10 * kS), 10 * kS + 2);
  // The next tick is far ahead of the bumps and passes unchanged.
  EXPECT_EQ(guard.next(10 * kS + 16 * kMs), 10 * kS + 16 * kMs);
}

// LiDAR: a scan that arrives a little out of order.
TEST(StampGuard, SmallRewindIsBumped) {
  StampGuard guard;
  ASSERT_EQ(guard.next(10 * kS), 10 * kS);
  EXPECT_EQ(guard.next(10 * kS - 8 * kMs), 10 * kS + 1);
  // Exactly the threshold behind the last stamp (now 10 s + 1 ns) is still
  // the same session.
  EXPECT_EQ(guard.next(10 * kS + 1 - SimClockGate::kNewSessionBackstepNs), 10 * kS + 2);
}

// The bug: the sim restarted near zero after the previous session reached
// 456 s, and every stamp stayed pinned at 456 s + n ns.
TEST(StampGuard, LargeStepBackFollowsNewSession) {
  StampGuard guard;
  ASSERT_EQ(guard.next(456 * kS), 456 * kS);
  EXPECT_EQ(guard.next(2 * kS), 2 * kS);
  EXPECT_EQ(guard.next(2 * kS), 2 * kS + 1);
  EXPECT_EQ(guard.next(2 * kS + 16 * kMs), 2 * kS + 16 * kMs);
}

TEST(StampGuard, JustOverThresholdFollowsNewSession) {
  StampGuard guard;
  ASSERT_EQ(guard.next(10 * kS), 10 * kS);
  const int64_t t = 10 * kS - SimClockGate::kNewSessionBackstepNs - 1;
  EXPECT_EQ(guard.next(t), t);
}

// A restart less than 1 s behind is only caught through the reconnect, which
// the caller passes in.
TEST(StampGuard, NewSessionFlagFollowsSmallStepBack) {
  StampGuard guard;
  ASSERT_EQ(guard.next(500 * kMs), 500 * kMs);
  EXPECT_EQ(guard.next(100 * kMs, /*new_session=*/true), 100 * kMs);
  EXPECT_EQ(guard.next(100 * kMs), 100 * kMs + 1);
}

TEST(StampGuard, NewSessionFlagAheadOfTheMarkIsUnchanged) {
  StampGuard guard;
  ASSERT_EQ(guard.next(60 * kS), 60 * kS);
  EXPECT_EQ(guard.next(60 * kS + 40 * kMs, /*new_session=*/true), 60 * kS + 40 * kMs);
}
