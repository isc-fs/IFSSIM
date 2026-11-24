// Unit tests for include/sensor_stream_sync.h: the bridge measures the sim's
// sensor frame size from the stream, so a sim and a bridge from different
// commits are refused instead of feeding the pipeline 1 frame in 47.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "sensor_stream_sync.h"

using ifssim_bridge::BytesToNextFrame;
using ifssim_bridge::FindMagic;
using ifssim_bridge::FrameStride;

namespace {

constexpr uint32_t kMagic = 0x49465353;  // SENSOR_MAGIC, "IFSS"

// `count` frames of `size` bytes, each starting with the magic, filled with a
// byte pattern that never forms the magic; `offset` bytes of the first frame
// are cut off, as when a read starts mid-frame.
std::vector<uint8_t> Stream(size_t size, int count, size_t offset = 0)
{
  std::vector<uint8_t> out;
  for (int f = 0; f < count; ++f) {
    std::vector<uint8_t> frame(size);
    for (size_t i = 0; i < size; ++i) frame[i] = static_cast<uint8_t>(i % 7);
    std::memcpy(frame.data(), &kMagic, sizeof(kMagic));
    out.insert(out.end(), frame.begin(), frame.end());
  }
  out.erase(out.begin(), out.begin() + static_cast<long>(offset));
  return out;
}

}  // namespace

TEST(SensorStreamSync, MatchingFramesGiveTheirSize) {
  const auto s = Stream(188, 8);
  EXPECT_EQ(FrameStride(s.data(), s.size(), kMagic), 188u);
}

TEST(SensorStreamSync, ASimWithTheOldFrameIsMeasuredNotMisread) {
  // #875's case the other way round: a bridge reading 188 from a 180-byte sim.
  const auto s = Stream(180, 8);
  EXPECT_EQ(FrameStride(s.data(), s.size(), kMagic), 180u);
}

TEST(SensorStreamSync, AProbeStartingMidFrameStillFindsTheSize) {
  // What an old bridge sees from #875's sim: it read 180 of a 188-byte frame.
  const auto s = Stream(188, 9, 180);
  EXPECT_EQ(FindMagic(s.data(), s.size(), kMagic), 8);
  EXPECT_EQ(FrameStride(s.data(), s.size(), kMagic), 188u);
}

TEST(SensorStreamSync, AMagicInsideTheDataIsNotTakenForAFrame) {
  auto s = Stream(188, 8);
  std::memcpy(s.data() + 40, &kMagic, sizeof(kMagic));  // by chance, inside frame 0
  EXPECT_EQ(FrameStride(s.data(), s.size(), kMagic), 188u);
}

TEST(SensorStreamSync, TooLittleStreamCantTell) {
  const auto s = Stream(188, 1);
  EXPECT_EQ(FrameStride(s.data(), s.size(), kMagic), 0u);
  std::vector<uint8_t> noise(1000, 0xAB);
  EXPECT_EQ(FrameStride(noise.data(), noise.size(), kMagic), 0u);
}

TEST(SensorStreamSync, SkipLandsTheNextReadOnAFrame) {
  // Probe began 8 bytes into a frame: 8 frames' worth read, first magic at 180.
  const size_t n = 8 * 188;
  const size_t skip = BytesToNextFrame(n, 180, 188);
  EXPECT_EQ((n + skip - 180) % 188, 0u);
  EXPECT_LT(skip, 188u);
  // Already on a boundary: nothing to skip.
  EXPECT_EQ(BytesToNextFrame(8 * 188, 0, 188), 0u);
}
