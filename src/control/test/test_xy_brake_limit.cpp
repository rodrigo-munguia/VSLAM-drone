#include <cmath>
#include <gtest/gtest.h>

#include "xy_brake_limit.hpp"

TEST(XyBrakeLimit, FarAwayUsesCruiseVmax)
{
  double ux = 10.0;
  double uy = 0.0;
  control::apply_xy_brake_limit(ux, uy, /*dist=*/50.0, /*vmax=*/3.0, /*a=*/0.45);
  EXPECT_NEAR(ux, 3.0, 1e-9);
  EXPECT_NEAR(uy, 0.0, 1e-9);
}

TEST(XyBrakeLimit, NearWaypointTapersBelowVmax)
{
  // dist=2 m, a=0.45 → v_cap = sqrt(1.8) ≈ 1.342
  double ux = 3.0;
  double uy = 0.0;
  control::apply_xy_brake_limit(ux, uy, 2.0, 3.0, 0.45);
  EXPECT_NEAR(ux, std::sqrt(1.8), 1e-9);
}

TEST(XyBrakeLimit, PreservesDirection)
{
  double ux = 3.0;
  double uy = 4.0;
  control::apply_xy_brake_limit(ux, uy, 0.5, 3.0, 0.45);
  EXPECT_NEAR(std::hypot(ux, uy), std::sqrt(0.45), 1e-9);
  EXPECT_NEAR(ux / uy, 3.0 / 4.0, 1e-9);
}

TEST(XyBrakeLimit, ClosingSpeedTowardWaypoint)
{
  // e = (10,0), v = (2,0) → closing = 2
  EXPECT_NEAR(control::closing_speed_xy(10.0, 0.0, 2.0, 0.0), 2.0, 1e-9);
  // departing
  EXPECT_NEAR(control::closing_speed_xy(10.0, 0.0, -2.0, 0.0), -2.0, 1e-9);
}

TEST(XyBrakeLimit, ClosingTooFastTriggers)
{
  // v=3, a=0.45 → need d > 9/0.9 = 10 m
  EXPECT_TRUE(control::closing_too_fast(3.0, 8.0, 0.45));
  EXPECT_FALSE(control::closing_too_fast(3.0, 12.0, 0.45));
  EXPECT_FALSE(control::closing_too_fast(-1.0, 1.0, 0.45));
}

TEST(XyBrakeLimit, AsymmetricBrakeFasterThanAccel)
{
  double vx = 3.0;
  double vy = 0.0;
  // Brake toward 0 with a_dec=2.5, dt=0.05 → step 0.125
  control::rate_limit_xy_asymmetric(vx, vy, 0.0, 0.0, /*acc=*/1.0, /*dec=*/2.5, 0.05);
  EXPECT_NEAR(vx, 3.0 - 0.125, 1e-9);

  vx = 0.0;
  // Accel toward 3 with a_acc=1.0 → step 0.05
  control::rate_limit_xy_asymmetric(vx, vy, 3.0, 0.0, 1.0, 2.5, 0.05);
  EXPECT_NEAR(vx, 0.05, 1e-9);
}

TEST(XyBrakeLimit, CreepScalesToSpeed)
{
  double ux = 3.0;
  double uy = 4.0;
  control::scale_xy_to_speed(ux, uy, 0.5);
  EXPECT_NEAR(std::hypot(ux, uy), 0.5, 1e-9);
}
