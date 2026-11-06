#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "go_home_ru.hpp"

using control::go_home_ru::decide_explore;
using control::go_home_ru::ExploreDecision;
using control::go_home_ru::explore_sizing_equivalent;

TEST(GoHomeRu, ValidRadiusExploresWithEqualHalfExtents)
{
  double x_a = 0, y_a = 0;
  EXPECT_EQ(decide_explore(1.5, x_a, y_a), ExploreDecision::ExploreWithRadius);
  EXPECT_DOUBLE_EQ(x_a, 1.5);
  EXPECT_DOUBLE_EQ(y_a, 1.5);
}

TEST(GoHomeRu, ZeroRadiusExploresLikeOriginal)
{
  double x_a = 9, y_a = 9;
  EXPECT_EQ(decide_explore(0.0, x_a, y_a), ExploreDecision::ExploreWithRadius);
  EXPECT_DOUBLE_EQ(x_a, 0.0);
  EXPECT_DOUBLE_EQ(y_a, 0.0);
}

TEST(GoHomeRu, NonFiniteSkips)
{
  double x_a = 9, y_a = 9;
  EXPECT_EQ(
    decide_explore(std::numeric_limits<double>::quiet_NaN(), x_a, y_a),
    ExploreDecision::SkipUnavailable);
  EXPECT_EQ(
    decide_explore(std::numeric_limits<double>::infinity(), x_a, y_a),
    ExploreDecision::SkipUnavailable);
}

TEST(GoHomeRu, LegacyAndMatchingRuAgreeOnBranch)
{
  EXPECT_TRUE(explore_sizing_equivalent(2.25, 2.25));
  EXPECT_TRUE(explore_sizing_equivalent(0.0, 0.0));
  EXPECT_TRUE(explore_sizing_equivalent(
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(explore_sizing_equivalent(2.0, 3.0));
  EXPECT_FALSE(explore_sizing_equivalent(
    2.0, std::numeric_limits<double>::quiet_NaN()));
}

TEST(GoHomeRu, AnisotropicUsesScalarMaxAxisRadius)
{
  const double r_u = std::sqrt(std::max(0.16, 0.04));
  double x_a = 0, y_a = 0;
  ASSERT_EQ(decide_explore(r_u, x_a, y_a), ExploreDecision::ExploreWithRadius);
  EXPECT_DOUBLE_EQ(x_a, 0.4);
  EXPECT_DOUBLE_EQ(y_a, 0.4);
}
