#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "go_home_ru.hpp"
#include "odometry_radius_feedback.hpp"

using control::go_home_ru::decide_explore;
using control::go_home_ru::ExploreDecision;
using control::odometry_radius::accept_radius_if_armed;
using control::odometry_radius::apply_odometry_pose;
using control::odometry_radius::apply_robotstate_radius_only;
using control::odometry_radius::arm_after_odometry;
using control::odometry_radius::clear_and_disarm;
using control::odometry_radius::clear_radius;
using control::odometry_radius::FeedbackState;
using control::odometry_radius::Pose6;

TEST(OdometryRadius, OdometryPoseDoesNotEraseRadius)
{
  FeedbackState s;
  apply_robotstate_radius_only(s, 1.25);
  Pose6 conflicting{99, 98, 97, 1, 2, 3};
  apply_odometry_pose(s, conflicting);
  EXPECT_DOUBLE_EQ(s.pose.x, 99);
  EXPECT_DOUBLE_EQ(s.r_u, 1.25);
}

TEST(OdometryRadius, RadiusOnlyIgnoresWouldBePoseAndBothOrders)
{
  FeedbackState s;
  // Radius before pose
  apply_robotstate_radius_only(s, 0.5);
  apply_odometry_pose(s, Pose6{1, 2, 3, 0, 0, 0});
  EXPECT_DOUBLE_EQ(s.pose.x, 1);
  EXPECT_DOUBLE_EQ(s.r_u, 0.5);

  // Pose before radius (conflicting legacy pose would be 100 — not applied)
  FeedbackState s2;
  apply_odometry_pose(s2, Pose6{4, 5, 6, 0, 0, 0.1});
  apply_robotstate_radius_only(s2, 2.0);
  EXPECT_DOUBLE_EQ(s2.pose.x, 4);
  EXPECT_DOUBLE_EQ(s2.pose.y, 5);
  EXPECT_DOUBLE_EQ(s2.r_u, 2.0);
}

TEST(OdometryRadius, MissingRadiusAndClearOnReset)
{
  FeedbackState s;
  EXPECT_TRUE(std::isnan(s.r_u));
  double xa = 0, ya = 0;
  EXPECT_EQ(decide_explore(s.r_u, xa, ya), ExploreDecision::SkipUnavailable);

  apply_robotstate_radius_only(s, 3.0);
  apply_odometry_pose(s, Pose6{1, 0, 0, 0, 0, 0});
  clear_radius(s);
  EXPECT_TRUE(std::isnan(s.r_u));
  EXPECT_DOUBLE_EQ(s.pose.x, 1);  // pose retained; radius cleared
  EXPECT_EQ(decide_explore(s.r_u, xa, ya), ExploreDecision::SkipUnavailable);
}

TEST(OdometryRadius, ZeroRadiusStillExploresLikeLegacy)
{
  double xa = 0, ya = 0;
  EXPECT_EQ(decide_explore(0.0, xa, ya), ExploreDecision::ExploreWithRadius);
  EXPECT_DOUBLE_EQ(xa, 0.0);
  EXPECT_DOUBLE_EQ(ya, 0.0);
}

TEST(OdometryRadius, CoordinatedResetDropsQueuedStyleRadiusUntilArm)
{
  FeedbackState s;
  bool armed = true;
  apply_robotstate_radius_only(s, 1.0);
  clear_and_disarm(s, armed);
  EXPECT_TRUE(std::isnan(s.r_u));
  EXPECT_FALSE(armed);
  // Simulated queued pre-reset callback while disarmed
  EXPECT_FALSE(accept_radius_if_armed(s, 9.9, armed));
  EXPECT_TRUE(std::isnan(s.r_u));
  arm_after_odometry(armed);
  EXPECT_TRUE(accept_radius_if_armed(s, 0.4, armed));
  EXPECT_DOUBLE_EQ(s.r_u, 0.4);
}

TEST(OdometryRadius, ConflictingLegacyPoseFieldsNotUsedForPose)
{
  // Simulated: Robotstate would claim pose (10,20,30) with r_u=0.8,
  // Odometry claims (1,2,3). Odometry mode keeps Odometry pose + custom r_u.
  FeedbackState s;
  apply_odometry_pose(s, Pose6{1, 2, 3, 0.1, 0.2, 0.3});
  apply_robotstate_radius_only(s, 0.8);
  EXPECT_DOUBLE_EQ(s.pose.x, 1);
  EXPECT_DOUBLE_EQ(s.pose.y, 2);
  EXPECT_DOUBLE_EQ(s.pose.z, 3);
  EXPECT_DOUBLE_EQ(s.r_u, 0.8);
  // Applying another odometry update must not wipe radius
  apply_odometry_pose(s, Pose6{1.1, 2.1, 3.1, 0, 0, 0});
  EXPECT_DOUBLE_EQ(s.r_u, 0.8);
}
