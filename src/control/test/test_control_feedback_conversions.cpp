#include <cmath>

#include <gtest/gtest.h>

#include "control_feedback_conversions.hpp"

using control::feedback::ControlPoseFeedback;
using control::feedback::expected_odometry_frames;
using control::feedback::legacy_internal_to_control;
using control::feedback::odometry_pose_to_control;
using control::feedback::odometry_pose_to_internal_slam;
using control::feedback::wrap_pi;

TEST(ControlFeedback, LegacyAndOdometryEquivalentCombinedPose)
{
  // Internal SLAM sample (as Robotstate indices)
  const double x_i = 1.5, y_i = -2.5, z_i = 3.5;
  const double roll_i = 0.12, pitch_i = -0.34, yaw_i = 0.56;
  const double init_yaw_i = 0.10;
  const double init_z = 1.25;

  // Matching public odometry via A2 D and RPY flips
  const double x_p = x_i;
  const double y_p = -y_i;
  const double z_p = -z_i;
  const double roll_p = roll_i;
  const double pitch_p = -pitch_i;
  const double yaw_p = -yaw_i;
  const double init_yaw_p = -init_yaw_i;

  const auto from_legacy = legacy_internal_to_control(
    x_i, y_i, z_i, roll_i, pitch_i, yaw_i, init_yaw_i, init_z);
  const auto from_odom = odometry_pose_to_control(
    x_p, y_p, z_p, roll_p, pitch_p, yaw_p, init_yaw_p, init_z);

  EXPECT_NEAR(from_odom.x, from_legacy.x, 1e-12);
  EXPECT_NEAR(from_odom.y, from_legacy.y, 1e-12);
  EXPECT_NEAR(from_odom.z, from_legacy.z, 1e-12);
  EXPECT_NEAR(from_odom.roll, from_legacy.roll, 1e-12);
  EXPECT_NEAR(from_odom.pitch, from_legacy.pitch, 1e-12);
  EXPECT_NEAR(from_odom.yaw, from_legacy.yaw, 1e-12);

  EXPECT_NEAR(from_odom.x, 1.5, 1e-12);
  EXPECT_NEAR(from_odom.y, -2.5, 1e-12);
  EXPECT_NEAR(from_odom.z, -2.25, 1e-12);
}

TEST(ControlFeedback, YawWrapping)
{
  const double init_yaw = 3.0;
  const double yaw = -3.0;  // delta crosses ±π
  const auto c = odometry_pose_to_control(
    0, 0, 0, 0, 0, yaw, init_yaw, 0);
  EXPECT_NEAR(c.yaw, wrap_pi(yaw - init_yaw), 1e-12);
  EXPECT_LE(std::abs(c.yaw), M_PI + 1e-12);
}

TEST(ControlFeedback, DeterministicGoPointErrorsMatch)
{
  // Same logical pose via both paths → identical position/yaw errors vs target.
  const double x_i = 0.5, y_i = 0.25, z_i = 1.0;
  const double yaw_i = 0.4;
  const double init_yaw_i = 0.0;
  const double init_z = 0.0;
  const double x_p = x_i, y_p = -y_i, z_p = -z_i;
  const double yaw_p = -yaw_i, init_yaw_p = -init_yaw_i;

  const auto L = legacy_internal_to_control(
    x_i, y_i, z_i, 0, 0, yaw_i, init_yaw_i, init_z);
  const auto O = odometry_pose_to_control(
    x_p, y_p, z_p, 0, 0, yaw_p, init_yaw_p, init_z);

  const double xd = 1.0, yd = -0.5, zd = 2.0, yawd = 0.1;
  EXPECT_NEAR(xd - O.x, xd - L.x, 1e-12);
  EXPECT_NEAR(yd - O.y, yd - L.y, 1e-12);
  EXPECT_NEAR(zd - O.z, zd - L.z, 1e-12);
  EXPECT_NEAR(wrap_pi(yawd - O.yaw), wrap_pi(yawd - L.yaw), 1e-12);
}

TEST(ControlFeedback, FrameValidationAndInternalRoundTrip)
{
  EXPECT_TRUE(expected_odometry_frames("odom", "base_link", "odom", "base_link"));
  EXPECT_FALSE(expected_odometry_frames("map", "base_link", "odom", "base_link"));

  const auto s = odometry_pose_to_internal_slam(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
  EXPECT_DOUBLE_EQ(s.x, 1.0);
  EXPECT_DOUBLE_EQ(s.y, -2.0);
  EXPECT_DOUBLE_EQ(s.z, -3.0);
  EXPECT_DOUBLE_EQ(s.roll, 0.1);
  EXPECT_DOUBLE_EQ(s.pitch, -0.2);
  EXPECT_DOUBLE_EQ(s.yaw, -0.3);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
