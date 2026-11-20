#include <cmath>
#include <gtest/gtest.h>

#include "gmap_relative_kf_init.hpp"

using gmap_relative_kf_init::CamPose;
using gmap_relative_kf_init::InitMode;
using gmap_relative_kf_init::compose;
using gmap_relative_kf_init::invert;
using gmap_relative_kf_init::propagate;
using gmap_relative_kf_init::propagate_xy_rel_z_abs;
using gmap_relative_kf_init::apply_cam_pose;
using gmap_relative_kf_init::from_keyframe;
using gmap_relative_kf_init::kInitRelativeKfPoseDefault;
using gmap_relative_kf_init::kInitHybridKfZAbsoluteDefault;
using gmap_relative_kf_init::resolve_init_relative_kf_pose_enable;
using gmap_relative_kf_init::resolve_init_hybrid_kf_z_absolute_enable;
using gmap_relative_kf_init::select_init_mode;

static CamPose make_pose(double yaw, double x, double y, double z)
{
  CamPose T;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  T.Rc2n.zeros();
  T.Rc2n(0, 0) = c;
  T.Rc2n(0, 1) = -s;
  T.Rc2n(1, 0) = s;
  T.Rc2n(1, 1) = c;
  T.Rc2n(2, 2) = 1.0;
  T.t_c2n(0) = x;
  T.t_c2n(1) = y;
  T.t_c2n(2) = z;
  return T;
}

/** Non-yaw-only attitude: small roll about X then yaw about Z. */
static CamPose make_pose_rp(
  double roll, double yaw, double x, double y, double z)
{
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  arma::mat::fixed<3, 3> Rx;
  Rx.zeros();
  Rx(0, 0) = 1.0;
  Rx(1, 1) = cr;
  Rx(1, 2) = -sr;
  Rx(2, 1) = sr;
  Rx(2, 2) = cr;
  arma::mat::fixed<3, 3> Rz;
  Rz.zeros();
  Rz(0, 0) = cy;
  Rz(0, 1) = -sy;
  Rz(1, 0) = sy;
  Rz(1, 1) = cy;
  Rz(2, 2) = 1.0;
  CamPose T;
  T.Rc2n = Rz * Rx;
  T.t_c2n(0) = x;
  T.t_c2n(1) = y;
  T.t_c2n(2) = z;
  return T;
}

TEST(RelativeKfInit, InvertComposeIdentity)
{
  CamPose T = make_pose(0.3, 1.0, -2.0, 0.5);
  CamPose I = compose(T, invert(T));
  EXPECT_NEAR(arma::norm(I.t_c2n), 0.0, 1e-9);
  EXPECT_NEAR(arma::norm(I.Rc2n - arma::eye(3, 3)), 0.0, 1e-9);
}

TEST(RelativeKfInit, PropagateMatchesComposition)
{
  // GS prev corrected; EKF absolute drifted.
  CamPose T_gs_prev = make_pose(0.1, 0.0, 0.0, 1.0);
  CamPose T_ekf_prev = make_pose(0.1, 0.2, 0.0, 1.0);
  CamPose T_ekf_curr = make_pose(0.25, 1.2, 0.5, 1.1);
  CamPose T_gs_curr = propagate(T_gs_prev, T_ekf_prev, T_ekf_curr);

  CamPose T_rel = compose(invert(T_ekf_prev), T_ekf_curr);
  CamPose expect = compose(T_gs_prev, T_rel);
  EXPECT_NEAR(arma::norm(T_gs_curr.t_c2n - expect.t_c2n), 0.0, 1e-9);
  EXPECT_NEAR(arma::norm(T_gs_curr.Rc2n - expect.Rc2n), 0.0, 1e-9);
}

TEST(RelativeKfInit, ConfigA1DefaultIsRelative)
{
  EXPECT_TRUE(kInitRelativeKfPoseDefault);
  EXPECT_TRUE(kInitHybridKfZAbsoluteDefault);  // KF-Z-A5
  // No env → YAML/declare value wins (production default true).
  EXPECT_TRUE(resolve_init_relative_kf_pose_enable(true, nullptr));
  EXPECT_TRUE(resolve_init_relative_kf_pose_enable(true, ""));
  EXPECT_FALSE(resolve_init_relative_kf_pose_enable(false, nullptr));
}

TEST(RelativeKfInit, ConfigA1EnvOverrideAbsoluteAndRelative)
{
  EXPECT_FALSE(resolve_init_relative_kf_pose_enable(true, "0"));
  EXPECT_FALSE(resolve_init_relative_kf_pose_enable(true, "false"));
  EXPECT_TRUE(resolve_init_relative_kf_pose_enable(false, "1"));
  EXPECT_TRUE(resolve_init_relative_kf_pose_enable(false, "true"));
  EXPECT_TRUE(resolve_init_relative_kf_pose_enable(false, "YES"));
}

TEST(RelativeKfInit, ApplyDerivesBodyFromExtrinsics)
{
  KEYFRAME kf;
  kf.Rr2c = arma::eye(3, 3);
  kf.t_c2r.zeros();
  kf.t_c2r(2) = 0.1;  // camera below body along +z_cam / check formula
  CamPose T = make_pose(0.0, 3.0, 4.0, 5.0);
  apply_cam_pose(kf, T);
  EXPECT_NEAR(kf.t_c2n(0), 3.0, 1e-12);
  EXPECT_NEAR(kf.t_c2n(1), 4.0, 1e-12);
  EXPECT_NEAR(kf.t_c2n(2), 5.0, 1e-12);
  // Rn2r = Rr2c^T * Rn2c = Rn2c when Rr2c=I
  EXPECT_NEAR(arma::norm(kf.Rn2r - kf.Rn2c), 0.0, 1e-12);
  // r_N = t_c2n - Rn2r^T * t_c2r
  arma::vec::fixed<3> r_expect = kf.t_c2n - kf.Rn2r.t() * kf.t_c2r;
  EXPECT_NEAR(arma::norm(kf.r_N - r_expect), 0.0, 1e-12);
  CamPose back = from_keyframe(kf);
  EXPECT_NEAR(arma::norm(back.t_c2n - T.t_c2n), 0.0, 1e-12);
}

// -------------------- KF-Z-A2 hybrid --------------------

TEST(HybridKfZ, ZAnchorsToLocalCameraZ)
{
  CamPose T_gs_prev = make_pose(0.0, 10.0, 20.0, 7.5);   // BA-ish Z
  CamPose T_ekf_prev = make_pose(0.0, 10.1, 20.0, 0.1);
  CamPose T_ekf_curr = make_pose(0.05, 15.0, 22.0, -0.4);  // Local Z

  CamPose T_full = propagate(T_gs_prev, T_ekf_prev, T_ekf_curr);
  CamPose T_hyb = propagate_xy_rel_z_abs(T_gs_prev, T_ekf_prev, T_ekf_curr);

  EXPECT_NEAR(T_hyb.t_c2n(2), T_ekf_curr.t_c2n(2), 1e-12);
  EXPECT_NEAR(T_hyb.t_c2n(0), T_full.t_c2n(0), 1e-12);
  EXPECT_NEAR(T_hyb.t_c2n(1), T_full.t_c2n(1), 1e-12);
  EXPECT_NEAR(arma::norm(T_hyb.Rc2n - T_full.Rc2n), 0.0, 1e-12);
  // Full-relative inherits GS Z offset; hybrid must not.
  EXPECT_GT(std::abs(T_full.t_c2n(2) - T_ekf_curr.t_c2n(2)), 1.0);
}

TEST(HybridKfZ, RotatedRelativeMatchesFullRelativeXyAndR)
{
  CamPose T_gs_prev = make_pose_rp(0.08, 0.35, 1.0, -2.0, 4.0);
  CamPose T_ekf_prev = make_pose_rp(0.05, 0.30, 1.2, -1.8, 0.2);
  CamPose T_ekf_curr = make_pose_rp(0.12, 0.55, 3.5, 0.4, -1.7);

  CamPose T_full = propagate(T_gs_prev, T_ekf_prev, T_ekf_curr);
  CamPose T_hyb = propagate_xy_rel_z_abs(T_gs_prev, T_ekf_prev, T_ekf_curr);

  EXPECT_NEAR(T_hyb.t_c2n(0), T_full.t_c2n(0), 1e-12);
  EXPECT_NEAR(T_hyb.t_c2n(1), T_full.t_c2n(1), 1e-12);
  EXPECT_NEAR(arma::norm(T_hyb.Rc2n - T_full.Rc2n), 0.0, 1e-12);
  EXPECT_NEAR(T_hyb.t_c2n(2), T_ekf_curr.t_c2n(2), 1e-12);
}

TEST(HybridKfZ, BaModifiedPreviousZNotInherited)
{
  // Simulate BA displacing previous Global Z while EKF Z stayed flat.
  CamPose T_gs_prev = make_pose(0.2, 5.0, 5.0, 9.0);  // BA Z = 9
  CamPose T_ekf_prev = make_pose(0.2, 5.1, 5.0, 0.0);
  CamPose T_ekf_curr = make_pose(0.25, 8.0, 6.0, 0.05);

  CamPose T_full = propagate(T_gs_prev, T_ekf_prev, T_ekf_curr);
  CamPose T_hyb = propagate_xy_rel_z_abs(T_gs_prev, T_ekf_prev, T_ekf_curr);

  EXPECT_NEAR(T_full.t_c2n(2), 9.05, 1e-9);  // inherits ~BA offset
  EXPECT_NEAR(T_hyb.t_c2n(2), 0.05, 1e-12);   // Local only
  EXPECT_NEAR(T_hyb.t_c2n(0), T_full.t_c2n(0), 1e-12);
  EXPECT_NEAR(T_hyb.t_c2n(1), T_full.t_c2n(1), 1e-12);
}

TEST(HybridKfZ, FirstKfAndMissingHistorySelectAbsolute)
{
  EXPECT_EQ(
    select_init_mode(true, true, false, 0), InitMode::Absolute);
  EXPECT_EQ(
    select_init_mode(true, true, true, 0), InitMode::Absolute);
  EXPECT_EQ(
    select_init_mode(true, false, false, 5), InitMode::Absolute);
}

TEST(HybridKfZ, AbsoluteModeIgnoresHybridFlag)
{
  EXPECT_EQ(
    select_init_mode(false, true, true, 3), InitMode::Absolute);
  EXPECT_EQ(
    select_init_mode(false, false, true, 3), InitMode::Absolute);
}

TEST(HybridKfZ, ConfigMatrixFourCombinations)
{
  // rel=0 hybrid=0 → absolute
  EXPECT_EQ(select_init_mode(false, false, true, 2), InitMode::Absolute);
  // rel=0 hybrid=1 → absolute (hybrid ignored)
  EXPECT_EQ(select_init_mode(false, true, true, 2), InitMode::Absolute);
  // rel=1 hybrid=0 → full relative
  EXPECT_EQ(select_init_mode(true, false, true, 2), InitMode::FullRelative);
  // rel=1 hybrid=1 → hybrid
  EXPECT_EQ(
    select_init_mode(true, true, true, 2), InitMode::HybridXyRelZAbs);
}

TEST(HybridKfZ, HybridEnvOverride)
{
  EXPECT_FALSE(resolve_init_hybrid_kf_z_absolute_enable(false, nullptr));
  EXPECT_FALSE(resolve_init_hybrid_kf_z_absolute_enable(true, "0"));
  EXPECT_TRUE(resolve_init_hybrid_kf_z_absolute_enable(false, "1"));
  EXPECT_TRUE(resolve_init_hybrid_kf_z_absolute_enable(false, "true"));
}

TEST(HybridKfZ, ExtrinsicConsistencyAfterApply)
{
  KEYFRAME kf;
  kf.Rr2c = arma::eye(3, 3);
  kf.t_c2r.zeros();
  kf.t_c2r(2) = 0.15;
  kf.t_c2r(0) = 0.02;

  CamPose T_gs_prev = make_pose(0.1, 0.0, 0.0, 3.0);
  CamPose T_ekf_prev = make_pose(0.1, 0.1, 0.0, 0.0);
  CamPose T_ekf_curr = make_pose(0.2, 2.0, 1.0, -0.55);
  CamPose T_hyb =
    propagate_xy_rel_z_abs(T_gs_prev, T_ekf_prev, T_ekf_curr);
  apply_cam_pose(kf, T_hyb);

  EXPECT_NEAR(kf.t_c2n(2), T_ekf_curr.t_c2n(2), 1e-12);
  arma::vec::fixed<3> r_expect = kf.t_c2n - kf.Rn2r.t() * kf.t_c2r;
  EXPECT_NEAR(arma::norm(kf.r_N - r_expect), 0.0, 1e-12);
  EXPECT_NEAR(arma::norm(kf.Rn2c - T_hyb.Rc2n.t()), 0.0, 1e-12);
}

TEST(HybridKfZ, DefaultsShipRelativeOnHybridOn)
{
  // KF-Z-A5: no overrides → hybrid (relative ON + hybrid ON).
  EXPECT_TRUE(kInitRelativeKfPoseDefault);
  EXPECT_TRUE(kInitHybridKfZAbsoluteDefault);
  EXPECT_EQ(
    select_init_mode(
      kInitRelativeKfPoseDefault,
      kInitHybridKfZAbsoluteDefault,
      true,
      1),
    InitMode::HybridXyRelZAbs);
  // Explicit hybrid off → full-relative.
  EXPECT_EQ(
    select_init_mode(true, false, true, 1), InitMode::FullRelative);
  // Env-style hybrid on with relative on → hybrid.
  EXPECT_TRUE(resolve_init_hybrid_kf_z_absolute_enable(false, "1"));
  EXPECT_FALSE(resolve_init_hybrid_kf_z_absolute_enable(true, "0"));
  // Relative off → absolute regardless of hybrid.
  EXPECT_EQ(
    select_init_mode(false, true, true, 1), InitMode::Absolute);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
