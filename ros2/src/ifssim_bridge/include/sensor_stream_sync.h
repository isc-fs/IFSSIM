// Sensor stream framing — tells whether the sim writes the sensor frame this
// bridge reads, and finds the next frame boundary when the stream is out of step.
//
// The sensor stream is a raw TCP byte stream of fixed-size frames (SensorFrame,
// FFSDSSensorFrame on the plugin side), each starting with SENSOR_MAGIC. There
// is no length prefix: the bridge reads sizeof(SensorFrame) bytes at a time.
// When the plugin and the bridge come from different commits and the frame
// changed size between them (#875 appended imu_timestamp: 180 -> 188 bytes),
// every read after the first one starts mid-frame. The bridge used to drop
// those frames on the bad magic and carry on, so only ~1 frame in 47 got
// through: /imu, wheel speeds and steering reached the pipeline at a few Hz and
// the car drove badly, with no error anywhere.
//
// The magic is what lets the bridge measure the sim's frame size: the distance
// between consecutive magics in the stream. FrameStride() finds it in a probe
// of the stream; the bridge compares it with sizeof(SensorFrame) and refuses
// a mismatch loudly.
//
// Header-only so test/test_sensor_stream_sync.cpp can exercise it without any
// ROS plumbing.

#ifndef IFSSIM_BRIDGE__SENSOR_STREAM_SYNC_H_
#define IFSSIM_BRIDGE__SENSOR_STREAM_SYNC_H_

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace ifssim_bridge {

// Offset of the first `magic` (as the plugin writes it: native byte order, any
// alignment) at or after `from` in buf[0, n), or -1 when there is none.
inline long FindMagic(const uint8_t* buf, size_t n, uint32_t magic, size_t from = 0)
{
  for (size_t i = from; i + sizeof(magic) <= n; ++i) {
    uint32_t v;
    std::memcpy(&v, buf + i, sizeof(v));
    if (v == magic) return static_cast<long>(i);
  }
  return -1;
}

// The frame size the stream in buf[0, n) carries: the distance from its first
// magic to the next one, confirmed by a third magic one more stride on when the
// buffer reaches that far (a magic's 4 bytes could turn up inside a frame's data
// by chance). 0 when the buffer holds too few magics to tell.
inline size_t FrameStride(const uint8_t* buf, size_t n, uint32_t magic)
{
  const long first = FindMagic(buf, n, magic);
  if (first < 0) return 0;
  for (long next = FindMagic(buf, n, magic, first + 1); next >= 0;
       next = FindMagic(buf, n, magic, next + 1)) {
    const size_t stride = static_cast<size_t>(next - first);
    const size_t third = static_cast<size_t>(next) + stride;
    if (third + sizeof(magic) > n) return stride;  // can't confirm: take it
    uint32_t v;
    std::memcpy(&v, buf + third, sizeof(v));
    if (v == magic) return stride;
  }
  return 0;
}

// After reading buf[0, n) from the stream, how many more bytes to skip so the
// next read starts on a frame boundary, for frames of `frame_size` bytes whose
// first magic in buf is at `first_magic`.
inline size_t BytesToNextFrame(size_t n, size_t first_magic, size_t frame_size)
{
  const size_t into_frame = (n - first_magic) % frame_size;
  return into_frame == 0 ? 0 : frame_size - into_frame;
}

}  // namespace ifssim_bridge

#endif  // IFSSIM_BRIDGE__SENSOR_STREAM_SYNC_H_
