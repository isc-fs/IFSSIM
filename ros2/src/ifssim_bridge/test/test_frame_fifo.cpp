// Unit tests for include/frame_fifo.h.

#include <gtest/gtest.h>

#include <vector>

#include "frame_fifo.h"

using ifssim_bridge::FrameFifo;

TEST(FrameFifo, KeepsEveryFrameInOrderUnderCapacity) {
  FrameFifo<int> q(8);
  for (int i = 0; i < 5; ++i) EXPECT_FALSE(q.push(i));
  std::vector<int> out;
  q.drain(out);
  EXPECT_EQ(out, (std::vector<int>{0, 1, 2, 3, 4}));
  EXPECT_TRUE(q.empty());
  EXPECT_EQ(q.dropped(), 0u);
}

TEST(FrameFifo, DropsOldestWhenFull) {
  FrameFifo<int> q(3);
  for (int i = 0; i < 3; ++i) q.push(i);
  EXPECT_TRUE(q.push(3));
  EXPECT_TRUE(q.push(4));
  std::vector<int> out;
  q.drain(out);
  EXPECT_EQ(out, (std::vector<int>{2, 3, 4}));
  EXPECT_EQ(q.dropped(), 2u);
}

TEST(FrameFifo, DrainReplacesOutput) {
  FrameFifo<int> q(4);
  q.push(7);
  std::vector<int> out{1, 2, 3};
  q.drain(out);
  EXPECT_EQ(out, (std::vector<int>{7}));
  q.drain(out);
  EXPECT_TRUE(out.empty());
}
