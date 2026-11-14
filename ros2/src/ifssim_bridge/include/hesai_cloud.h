// Packs an organised sim LiDAR scan into the car's Hesai PointCloud2 layout.
//
// The car's driver publishes one row of slots x rings, 26 bytes per point,
// packed with no padding:
//   x, y, z, intensity  float32  at 0, 4, 8, 12
//   ring                uint16   at 16   (0 = top beam)
//   timestamp           float64  at 18   (absolute seconds, when the slot fired)
// Slots are in firing order and rings run top first within a slot; a beam with
// no return is (0, 0, 0) with intensity 0, keeping its ring and time.
//
// The plugin's streamLidar2 sends the scan already in that order as
// (x, y, z, intensity) floats, intensity < 0 = no return, so this only adds
// the ring and the time and repacks. ROS-free so it can be unit-tested.
#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

namespace ifssim_bridge {

constexpr uint32_t kHesaiPointStep = 26;
constexpr uint32_t kHesaiOffsetRing = 16;
constexpr uint32_t kHesaiOffsetTimestamp = 18;

// points: slots * channels * 4 floats. Returns slots * channels * 26 bytes.
// Slot s is stamped stamp_s + sweep_s * s / (slots - 1).
inline std::vector<uint8_t> packHesaiCloud(const float* points, int channels, int slots,
                                           double stamp_s, double sweep_s)
{
    std::vector<uint8_t> out;
    if (channels <= 0 || slots <= 0 || points == nullptr) return out;
    const size_t n = static_cast<size_t>(channels) * static_cast<size_t>(slots);
    out.resize(n * kHesaiPointStep);
    uint8_t* dst = out.data();
    const double slot_dt = slots > 1 ? sweep_s / static_cast<double>(slots - 1) : 0.0;
    for (int s = 0; s < slots; ++s) {
        const double t = stamp_s + slot_dt * static_cast<double>(s);
        for (int r = 0; r < channels; ++r) {
            const float* p = points + (static_cast<size_t>(s) * channels + r) * 4;
            float xyzi[4] = {0.0f, 0.0f, 0.0f, 0.0f};
            if (p[3] >= 0.0f) {
                xyzi[0] = p[0];
                xyzi[1] = p[1];
                xyzi[2] = p[2];
                xyzi[3] = p[3];
            }
            const uint16_t ring = static_cast<uint16_t>(r);
            std::memcpy(dst, xyzi, sizeof(xyzi));
            std::memcpy(dst + kHesaiOffsetRing, &ring, sizeof(ring));
            std::memcpy(dst + kHesaiOffsetTimestamp, &t, sizeof(t));
            dst += kHesaiPointStep;
        }
    }
    return out;
}

}  // namespace ifssim_bridge
