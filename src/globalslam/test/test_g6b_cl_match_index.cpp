#include <gtest/gtest.h>

#include "g6b_cl_match_index.hpp"

using globalslam::g6b_cl::resolve_matched_kf_index;

TEST(G6bClMatchIndex, UsesVisualMatchWhenInRange)
{
  const auto d = resolve_matched_kf_index(/*visual=*/12, /*ref=*/3, /*n=*/100);
  EXPECT_TRUE(d.valid);
  EXPECT_EQ(d.index, 12);
  EXPECT_STREQ(d.reason, "visual_match");
}

TEST(G6bClMatchIndex, HomeTagWithoutVisualDoesNotUsePoseGraphGauge)
{
  // G6b.2: pose-graph ref is not a corresponding visual match — skip BA.
  const auto d = resolve_matched_kf_index(/*visual=*/-1, /*ref=*/7, /*n=*/282);
  EXPECT_FALSE(d.valid);
  EXPECT_EQ(d.index, -1);
  EXPECT_STREQ(d.reason, "no_corresponding_visual_match");
}

TEST(G6bClMatchIndex, RejectsOutOfRangeVisual)
{
  const auto d = resolve_matched_kf_index(/*visual=*/500, /*ref=*/-1, /*n=*/100);
  EXPECT_FALSE(d.valid);
  EXPECT_EQ(d.index, -1);
  EXPECT_STREQ(d.reason, "no_corresponding_visual_match");
}

TEST(G6bClMatchIndex, EmptyMapIsInvalid)
{
  const auto d = resolve_matched_kf_index(/*visual=*/0, /*ref=*/0, /*n=*/0);
  EXPECT_FALSE(d.valid);
  EXPECT_STREQ(d.reason, "empty_map");
}

TEST(G6bClMatchIndex, VisualAtLastAppendedSlotAllowed)
{
  const auto d = resolve_matched_kf_index(/*visual=*/0, /*ref=*/50, /*n=*/283);
  EXPECT_TRUE(d.valid);
  EXPECT_EQ(d.index, 0);
  EXPECT_STREQ(d.reason, "visual_match");
}
