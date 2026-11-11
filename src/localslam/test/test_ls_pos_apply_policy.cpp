#include <cmath>
#include <gtest/gtest.h>

#include "ls_pos_apply_policy.hpp"

using localslam::PosUpdateGateParams;
using localslam::applied_internal_to_public;
using localslam::plan_pos_update_apply;

TEST(LsPosApply, BaAsymmetricAppliesXyWhenEnabled)
{
  PosUpdateGateParams g;
  g.gs_xy = true;
  g.gs_z = false;
  g.gs_xy_max = 1.0;
  g.gs_z_max = 1.0;
  const auto p = plan_pos_update_apply(0.05, -0.07, 0.09, "bundle_adjustment", g);
  EXPECT_TRUE(p.apply_xy);
  EXPECT_FALSE(p.apply_z);
  EXPECT_TRUE(p.any_applied);
  EXPECT_DOUBLE_EQ(p.applied_dx, 0.05);
  EXPECT_DOUBLE_EQ(p.applied_dy, -0.07);
  EXPECT_DOUBLE_EQ(p.applied_dz, 0.0);
  EXPECT_FALSE(p.reset_pc);

  double px = 0, py = 0, pz = 0;
  applied_internal_to_public(p.applied_dx, p.applied_dy, p.applied_dz, px, py, pz);
  EXPECT_DOUBLE_EQ(px, 0.05);
  EXPECT_DOUBLE_EQ(py, 0.07);
  EXPECT_DOUBLE_EQ(pz, 0.0);
}

TEST(LsPosApply, DefaultZDisabledAndMagnitudeBoundary)
{
  PosUpdateGateParams g;
  g.gs_xy = true;
  g.gs_z = false;
  g.gs_xy_max = 0.1;
  g.gs_z_max = 0.1;
  // XY magnitude exactly at boundary is rejected (< max).
  auto p = plan_pos_update_apply(0.1, 0.0, 0.05, "bundle_adjustment", g);
  EXPECT_FALSE(p.apply_xy);
  EXPECT_FALSE(p.any_applied);

  p = plan_pos_update_apply(0.099, 0.0, -0.5, "bundle_adjustment", g);
  EXPECT_TRUE(p.apply_xy);
  EXPECT_FALSE(p.apply_z);  // z gate off
}

TEST(LsPosApply, NegativeZUsesAbsoluteMagnitude)
{
  PosUpdateGateParams g;
  g.gs_xy = false;
  g.gs_z = true;
  g.gs_z_max = 0.1;
  // Historical signed check would accept large negative Z; abs rejects it.
  auto p = plan_pos_update_apply(0, 0, -0.5, "bundle_adjustment", g);
  EXPECT_FALSE(p.apply_z);
  EXPECT_FALSE(p.any_applied);

  p = plan_pos_update_apply(0, 0, -0.05, "bundle_adjustment", g);
  EXPECT_TRUE(p.apply_z);
  EXPECT_DOUBLE_EQ(p.applied_dz, -0.05);
}

TEST(LsPosApply, CloseLoopResetsPcOnlyWhenApplied)
{
  PosUpdateGateParams g;
  g.cl_xy = true;
  g.cl_z = false;
  g.cl_xy_max = 10.0;
  auto p = plan_pos_update_apply(1.0, 2.0, 0.5, "close_loop", g);
  EXPECT_TRUE(p.any_applied);
  EXPECT_TRUE(p.reset_pc);

  g.cl_xy = false;
  p = plan_pos_update_apply(1.0, 2.0, 0.5, "close_loop", g);
  EXPECT_FALSE(p.any_applied);
  EXPECT_FALSE(p.reset_pc);
}

TEST(LsPosApply, PublicDCannotCancelAsymmetric)
{
  double px, py, pz;
  applied_internal_to_public(1.25, -7.44, 0.04, px, py, pz);
  EXPECT_DOUBLE_EQ(px, 1.25);
  EXPECT_DOUBLE_EQ(py, 7.44);
  EXPECT_DOUBLE_EQ(pz, -0.04);
}
