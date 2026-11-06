#include <cmath>

#include <gtest/gtest.h>

#include "cl_radius.hpp"

using localslam::cl_radius::accumulate_Pc_step;
using localslam::cl_radius::pose_xy_variance_equals_Pc;
using localslam::cl_radius::radius_from_Pc;

TEST(ClRadius, FormulaIsSqrtMaxDiagonalNoMultiplier)
{
  // Anisotropic Pc: larger axis dominates; no 3-sigma factor.
  EXPECT_DOUBLE_EQ(radius_from_Pc(4.0, 1.0), 2.0);
  EXPECT_DOUBLE_EQ(radius_from_Pc(1.0, 9.0), 3.0);
  EXPECT_DOUBLE_EQ(radius_from_Pc(0.0, 0.0), 0.0);
}

TEST(ClRadius, AccumulationGateAndProcessNoise)
{
  double Pc00 = 0.0, Pc11 = 0.0;
  const double sx = 0.2, sy = 0.1, vmin = 0.05, dt = 0.5;

  // Below velocity gate: no growth
  accumulate_Pc_step(Pc00, Pc11, 0.01, 0.01, dt, sx, sy, vmin);
  EXPECT_DOUBLE_EQ(Pc00, 0.0);
  EXPECT_DOUBLE_EQ(Pc11, 0.0);

  // Above gate on vx: Pc += σ² Δt
  accumulate_Pc_step(Pc00, Pc11, 0.2, 0.0, dt, sx, sy, vmin);
  EXPECT_DOUBLE_EQ(Pc00, sx * sx * dt);
  EXPECT_DOUBLE_EQ(Pc11, sy * sy * dt);

  const double r = radius_from_Pc(Pc00, Pc11);
  EXPECT_DOUBLE_EQ(r, std::sqrt(std::max(Pc00, Pc11)));
}

TEST(ClRadius, PoseCovarianceIsNotASubstitute)
{
  // Full PSD-looking pose block with cross-terms / anisotropy ≠ Pc.
  const double P_xx = 0.25, P_yy = 0.04, P_xy = 0.03;
  (void)P_xy;  // would appear in odom pose cov; unused by legacy r_u
  double Pc00 = 0.0, Pc11 = 0.0;
  accumulate_Pc_step(Pc00, Pc11, 0.5, 0.0, 2.0, 0.2, 0.2, 0.05);
  // After 2 s: Pc00 = 0.04*2 = 0.08
  EXPECT_NEAR(Pc00, 0.08, 1e-12);
  EXPECT_FALSE(pose_xy_variance_equals_Pc(P_xx, P_yy, Pc00, Pc11));

  // Even if diagonals were forced equal, that would be coincidence — heading
  // / D transforms of P still do not implement Pc accumulation.
  EXPECT_FALSE(pose_xy_variance_equals_Pc(0.5, 0.1, Pc00, Pc11));
}

TEST(ClRadius, RotatedPublicPoseCovStillNotPc)
{
  // A2 public position cov: D * P_pos * D with D=diag(1,-1,-1).
  // For xy: pub_xx=P_xx, pub_yy=P_yy (signs cancel on diagonal).
  const double P_xx = 0.08, P_yy = 0.08;
  double Pc00 = 0.0, Pc11 = 0.0;
  // Different history: Pc grown with anisotropic sigmas
  accumulate_Pc_step(Pc00, Pc11, 1.0, 0.0, 1.0, 0.3, 0.1, 0.0);
  EXPECT_NE(radius_from_Pc(P_xx, P_yy), radius_from_Pc(Pc00, Pc11));
}
