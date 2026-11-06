#include <cmath>
#include <gtest/gtest.h>
#include <deque>
#include <string>

#include "gt_frame_conversions.hpp"

using webotbridge::gt::GpsSample;
using webotbridge::gt::ImuYawSample;
using webotbridge::gt::interpolate_gps;
using webotbridge::gt::interpolate_yaw;
using webotbridge::gt::internal_robot_init_to_public;
using webotbridge::gt::lerp_angle;
using webotbridge::gt::webots_gps_to_public_gt;
using webotbridge::gt::wrap_angle;

TEST(GtPublic, IdentityYawPureAxisDisplacements)
{
  double xp = 0, yp = 0, zp = 0;
  webots_gps_to_public_gt(1.5, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, xp, yp, zp);
  EXPECT_DOUBLE_EQ(xp, 1.5);
  EXPECT_DOUBLE_EQ(yp, 0.0);
  EXPECT_DOUBLE_EQ(zp, 0.0);
}

TEST(GtPublic, PositiveAndNegative90Heading)
{
  double xp, yp, zp;
  webots_gps_to_public_gt(1.0, 0.0, 0.0, M_PI / 2.0, 0, 0, 0, 0, 0, 0, xp, yp, zp);
  EXPECT_NEAR(xp, 0.0, 1e-12);
  EXPECT_NEAR(yp, -1.0, 1e-12);

  webots_gps_to_public_gt(1.0, 0.0, 0.0, -M_PI / 2.0, 0, 0, 0, 0, 0, 0, xp, yp, zp);
  EXPECT_NEAR(xp, 0.0, 1e-12);
  EXPECT_NEAR(yp, 1.0, 1e-12);
}

TEST(GtPublic, OriginAndInternalInitWithD)
{
  double xp, yp, zp;
  // At origin displacement 0; Robot_init internal (1,2,0.5) → public (1,-2,-0.5)
  webots_gps_to_public_gt(
    4.0, -1.0, 2.0, 0.0,
    4.0, -1.0, 2.0,
    1.0, 2.0, 0.5, xp, yp, zp);
  EXPECT_DOUBLE_EQ(xp, 1.0);
  EXPECT_DOUBLE_EQ(yp, -2.0);
  EXPECT_DOUBLE_EQ(zp, -0.5);
}

TEST(GtPublic, AttitudeIndependenceIsApiLevel)
{
  // Helper has no roll/pitch arguments — compile-time guarantee.
  double a, b, c;
  webots_gps_to_public_gt(2, 3, 4, 0.2, 0, 0, 0, 0, 0, 0, a, b, c);
  double d, e, f;
  webots_gps_to_public_gt(2, 3, 4, 0.2, 0, 0, 0, 0, 0, 0, d, e, f);
  EXPECT_DOUBLE_EQ(a, d);
  EXPECT_DOUBLE_EQ(b, e);
  EXPECT_DOUBLE_EQ(c, f);
}

TEST(GtPublic, ForwardMotionTwoHeadings)
{
  // World +X motion of 1m from origin.
  double x0, y0, z0, x90, y90, z90;
  webots_gps_to_public_gt(1, 0, 0, 0.0, 0, 0, 0, 0, 0, 0, x0, y0, z0);
  webots_gps_to_public_gt(1, 0, 0, M_PI / 2.0, 0, 0, 0, 0, 0, 0, x90, y90, z90);
  EXPECT_NEAR(x0, 1.0, 1e-12);
  EXPECT_NEAR(y0, 0.0, 1e-12);
  EXPECT_NEAR(x90, 0.0, 1e-12);
  EXPECT_NEAR(y90, -1.0, 1e-12);
}

TEST(GtPublic, ClimbMapsToPublicUp)
{
  double xp, yp, zp;
  webots_gps_to_public_gt(0, 0, 5.0, 0.0, 0, 0, 1.0, 0, 0, 0, xp, yp, zp);
  EXPECT_DOUBLE_EQ(zp, 4.0);
}

TEST(GtPublic, InternalInitMustUseD)
{
  double ox, oy, oz;
  internal_robot_init_to_public(0.0, 2.0, 3.0, ox, oy, oz);
  EXPECT_DOUBLE_EQ(oy, -2.0);
  EXPECT_DOUBLE_EQ(oz, -3.0);
}

TEST(GtMatch, InterpolateGpsAndRejectOutsideWindow)
{
  std::deque<GpsSample> h;
  h.push_back({1.0, 0, 0, 0});
  h.push_back({2.0, 2, 0, 0});
  GpsSample out;
  std::string reason;
  EXPECT_TRUE(interpolate_gps(h, 1.5, 0.2, out, reason));
  EXPECT_NEAR(out.x, 1.0, 1e-12);

  EXPECT_FALSE(interpolate_gps(h, 5.0, 0.2, out, reason));
  EXPECT_FALSE(reason.empty());
}

TEST(GtMatch, InterpolateYawWraps)
{
  std::deque<ImuYawSample> h;
  h.push_back({0.0, 3.0});
  h.push_back({1.0, -3.0});
  double yaw = 0;
  std::string reason;
  EXPECT_TRUE(interpolate_yaw(h, 0.5, 1.0, yaw, reason));
  // Shortest path across ±π boundary stays near ±π, not through 0.
  EXPECT_GT(std::abs(yaw), 2.5);
}

TEST(GtMatch, EmptyHistoryFailsWithoutSilentLatest)
{
  std::deque<GpsSample> h;
  GpsSample out;
  std::string reason;
  EXPECT_FALSE(interpolate_gps(h, 1.0, 0.1, out, reason));
  EXPECT_EQ(reason, "empty GPS history");
}

TEST(GtAngle, LerpAndWrap)
{
  EXPECT_NEAR(wrap_angle(3.2), 3.2 - 2.0 * M_PI, 1e-9);
  EXPECT_NEAR(lerp_angle(0.0, 0.0, 0.5), 0.0, 1e-12);
}
