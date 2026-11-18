#include <cmath>
#include <gtest/gtest.h>

#include "xy_traj_ref.hpp"

namespace
{
constexpr double kVmax = 3.0;
constexpr double kKp = 0.5;
constexpr double kKd = 0.85;
constexpr double kABrake = 0.55;
constexpr double kTLag = 0.40;
}  // namespace

TEST(XyTrajRef, ZeroErrorGivesZeroVelocity)
{
  double vx = 1.0;
  double vy = 1.0;
  control::xy_traj_v_target_nav(
    0.0, 0.0, 2.0, 0.0, kVmax, kKp, kKd, kABrake, kTLag, vx, vy);
  EXPECT_NEAR(vx, 0.0, 1e-12);
  EXPECT_NEAR(vy, 0.0, 1e-12);
}

TEST(XyTrajRef, LargeErrorBoundedByCruise)
{
  double vx = 0.0;
  double vy = 0.0;
  control::xy_traj_v_target_nav(
    50.0, 0.0, 0.0, 0.0, kVmax, kKp, kKd, kABrake, kTLag, vx, vy);
  EXPECT_NEAR(vx, 3.0, 1e-9);
  EXPECT_NEAR(vy, 0.0, 1e-9);
}

TEST(XyTrajRef, SpeedReducesSmoothlyWithDistance)
{
  double prev = kVmax;
  for (double d : {40.0, 20.0, 10.0, 5.0, 2.0, 0.5, 0.1}) {
    double vx = 0.0;
    double vy = 0.0;
    control::xy_traj_v_target_nav(
      d, 0.0, 0.0, 0.0, kVmax, kKp, kKd, kABrake, kTLag, vx, vy);
    const double spd = std::hypot(vx, vy);
    EXPECT_LE(spd, prev + 1e-9);
    EXPECT_LE(spd, kVmax + 1e-9);
    EXPECT_LE(spd, std::sqrt(2.0 * kABrake * d) + 1e-9);
    prev = spd;
  }
}

TEST(XyTrajRef, SignFollowsPositionError)
{
  double vx = 0.0;
  double vy = 0.0;
  control::xy_traj_v_target_nav(
    -8.0, 0.0, 0.0, 0.0, kVmax, kKp, kKd, kABrake, kTLag, vx, vy);
  EXPECT_LT(vx, 0.0);
  EXPECT_NEAR(vy, 0.0, 1e-12);

  control::xy_traj_v_target_nav(
    0.0, 8.0, 0.0, 0.0, kVmax, kKp, kKd, kABrake, kTLag, vx, vy);
  EXPECT_NEAR(vx, 0.0, 1e-12);
  EXPECT_GT(vy, 0.0);
}

TEST(XyTrajRef, ClosingVelocityShrinksBrakeEnvelope)
{
  double vx_still = 0.0;
  double vy_still = 0.0;
  control::xy_traj_v_target_nav(
    8.0, 0.0, 0.0, 0.0, kVmax, kKp, kKd, kABrake, kTLag, vx_still, vy_still);

  double vx_fast = 0.0;
  double vy_fast = 0.0;
  control::xy_traj_v_target_nav(
    8.0, 0.0, 3.0, 0.0, kVmax, kKp, kKd, kABrake, kTLag, vx_fast, vy_fast);

  // Stationary: P request kp*e=4 capped by √(2ad)≈2.97
  EXPECT_NEAR(vx_still, std::sqrt(2.0 * kABrake * 8.0), 1e-9);
  // Closing: smaller cap AND Kd·v subtracts → strictly slower toward WP
  EXPECT_LT(vx_fast, vx_still - 0.05);
  EXPECT_GT(vx_fast, -0.5);  // not a reverse chase
}

TEST(XyTrajRef, DepartingVelocityDoesNotShrinkEnvelope)
{
  double vx_still = 0.0;
  double vy_still = 0.0;
  control::xy_traj_v_target_nav(
    8.0, 0.0, 0.0, 0.0, kVmax, kKp, 0.0, kABrake, kTLag, vx_still, vy_still);

  double vx_away = 0.0;
  double vy_away = 0.0;
  // kd=0 so only envelope compared; departing must not shrink d_eff
  control::xy_traj_v_target_nav(
    8.0, 0.0, -3.0, 0.0, kVmax, kKp, 0.0, kABrake, kTLag, vx_away, vy_away);

  EXPECT_NEAR(vx_away, vx_still, 1e-9);
}

TEST(XyTrajRef, MeasuredVelocityDampsCommand)
{
  double vx_p = 0.0;
  double vy_p = 0.0;
  control::xy_traj_v_target_nav(
    4.0, 0.0, 0.0, 0.0, kVmax, kKp, kKd, kABrake, kTLag, vx_p, vy_p);

  double vx_d = 0.0;
  double vy_d = 0.0;
  control::xy_traj_v_target_nav(
    4.0, 0.0, 1.5, 0.0, kVmax, kKp, kKd, kABrake, kTLag, vx_d, vy_d);

  EXPECT_LT(vx_d, vx_p);
}

TEST(XyTrajRef, OvershootGivesBoundedCorrectionNotCruiseChase)
{
  double vx = 0.0;
  double vy = 0.0;
  control::xy_traj_v_target_nav(
    -0.5, 0.0, 2.0, 0.0, kVmax, kKp, kKd, kABrake, kTLag, vx, vy);
  // e=-0.5 → kp*e=-0.25; −kd*v = -1.7 → raw ≈ -1.95, then cap √(2a d_eff)
  // d_eff with closing=(2*(-0.5))/0.5=-2 (departing from new e) → no lag shrink
  EXPECT_LT(vx, 0.0);
  EXPECT_LT(std::abs(vx), 1.5);
}

TEST(XyTrajRef, RateLimitCapsAcceleration)
{
  double vx = 0.0;
  double vy = 0.0;
  control::rate_limit_xy_asymmetric(
    vx, vy, 3.0, 0.0, /*acc=*/1.0, /*dec=*/2.5, 0.05);
  EXPECT_NEAR(vx, 0.05, 1e-9);

  vx = 3.0;
  control::rate_limit_xy_asymmetric(
    vx, vy, 0.0, 0.0, 1.0, 2.5, 0.05);
  EXPECT_NEAR(vx, 3.0 - 0.125, 1e-9);
}

TEST(XyTrajRef, NearZeroDistanceFinite)
{
  double vx = 99.0;
  double vy = 99.0;
  control::xy_traj_v_target_nav(
    1e-12, 0.0, 5.0, 0.0, kVmax, kKp, kKd, kABrake, kTLag, vx, vy);
  EXPECT_TRUE(std::isfinite(vx));
  EXPECT_TRUE(std::isfinite(vy));
  EXPECT_NEAR(vx, 0.0, 1e-9);
  EXPECT_NEAR(vy, 0.0, 1e-9);
}

TEST(XyTrajRef, NavToBodyYaw0PreservesAxes)
{
  double bx = 0.0;
  double by = 0.0;
  control::nav_xy_to_body(2.0, -1.5, 0.0, bx, by);
  EXPECT_NEAR(bx, 2.0, 1e-12);
  EXPECT_NEAR(by, -1.5, 1e-12);
}

TEST(XyTrajRef, NavToBodyYaw90Rotates)
{
  double bx = 0.0;
  double by = 0.0;
  control::nav_xy_to_body(3.0, 0.0, M_PI / 2.0, bx, by);
  EXPECT_NEAR(bx, 0.0, 1e-12);
  EXPECT_NEAR(by, -3.0, 1e-12);
}

TEST(XyTrajRef, StepBodyContinuousThroughWaypoint)
{
  double vrx = 2.0;
  double vry = 0.0;
  for (double ex : {4.0, 2.0, 1.0, 0.2, 0.0, -0.2, -0.5}) {
    const double prev = vrx;
    const double vnav = (ex > 0.0) ? 1.5 : ((ex < 0.0) ? -0.5 : 0.0);
    control::xy_traj_step_body(
      ex, 0.0, vnav, 0.0,
      /*yaw*/ 0.0, kVmax, kKp, kKd, kABrake, kTLag,
      /*a_acc*/ 1.0, /*a_dec*/ 2.5, 0.05, vrx, vry);
    EXPECT_TRUE(std::isfinite(vrx));
    EXPECT_TRUE(std::isfinite(vry));
    EXPECT_NEAR(vry, 0.0, 1e-9);
    EXPECT_LE(std::abs(vrx - prev), 0.13 + 1e-6);
  }
  EXPECT_LT(std::abs(vrx), 1.5);
}
