// Unit tests for include/hesai_cloud.h: the organised sim scan comes out in
// the car's packed 26-byte Hesai layout.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "hesai_cloud.h"

using ifssim_bridge::kHesaiPointStep;
using ifssim_bridge::packHesaiCloud;

namespace {

struct Point {
  float x, y, z, intensity;
  uint16_t ring;
  double t;
};

Point at(const std::vector<uint8_t>& buf, size_t i) {
  Point p{};
  const uint8_t* src = buf.data() + i * kHesaiPointStep;
  std::memcpy(&p.x, src + 0, 4);
  std::memcpy(&p.y, src + 4, 4);
  std::memcpy(&p.z, src + 8, 4);
  std::memcpy(&p.intensity, src + 12, 4);
  std::memcpy(&p.ring, src + 16, 2);
  std::memcpy(&p.t, src + 18, 8);
  return p;
}

}  // namespace

TEST(HesaiCloud, PacksEveryRayAt26Bytes) {
  const std::vector<float> pts(3 * 2 * 4, 1.0f);
  const auto buf = packHesaiCloud(pts.data(), 2, 3, 100.0, 0.09);
  EXPECT_EQ(buf.size(), 3u * 2u * 26u);
}

TEST(HesaiCloud, RingAndTimeFollowTheOrder) {
  // 3 slots x 2 rings, all valid.
  std::vector<float> pts;
  for (int s = 0; s < 3; ++s)
    for (int r = 0; r < 2; ++r) pts.insert(pts.end(), {float(s), float(r), 1.0f, 10.0f});
  const auto buf = packHesaiCloud(pts.data(), 2, 3, 100.0, 0.09);
  for (int s = 0; s < 3; ++s) {
    for (int r = 0; r < 2; ++r) {
      const Point p = at(buf, s * 2 + r);
      EXPECT_EQ(p.ring, r);
      EXPECT_FLOAT_EQ(p.x, float(s));
      EXPECT_FLOAT_EQ(p.y, float(r));
      EXPECT_FLOAT_EQ(p.intensity, 10.0f);
      EXPECT_NEAR(p.t, 100.0 + 0.045 * s, 1e-12);
    }
  }
}

TEST(HesaiCloud, NoReturnIsZeroWithRingAndTime) {
  const std::vector<float> pts = {5.0f, 6.0f, 7.0f, -1.0f, 1.0f, 2.0f, 3.0f, 0.0f};
  const auto buf = packHesaiCloud(pts.data(), 2, 1, 7.5, 0.09);
  const Point miss = at(buf, 0);
  EXPECT_EQ(miss.x, 0.0f);
  EXPECT_EQ(miss.y, 0.0f);
  EXPECT_EQ(miss.z, 0.0f);
  EXPECT_EQ(miss.intensity, 0.0f);
  EXPECT_EQ(miss.ring, 0);
  EXPECT_DOUBLE_EQ(miss.t, 7.5);
  // Intensity 0 is a valid (black) return, not a miss.
  const Point hit = at(buf, 1);
  EXPECT_FLOAT_EQ(hit.x, 1.0f);
  EXPECT_EQ(hit.intensity, 0.0f);
  EXPECT_EQ(hit.ring, 1);
}

TEST(HesaiCloud, EmptyInputGivesEmptyCloud) {
  EXPECT_TRUE(packHesaiCloud(nullptr, 116, 1500, 0.0, 0.09).empty());
  const float one[4] = {0, 0, 0, 0};
  EXPECT_TRUE(packHesaiCloud(one, 0, 1, 0.0, 0.09).empty());
}
